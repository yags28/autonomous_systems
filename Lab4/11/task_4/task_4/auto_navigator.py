#!/usr/bin/env python3

import math
from queue import PriorityQueue

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class AutoNavigator(Node):
    """Robot navigator that plans paths on an inflated occupancy grid."""

    def __init__(self):
        super().__init__('auto_navigator')

        self.inflation_radius_m = 0.2
        self.occupied_threshold = int(self.declare_parameter('occupied_threshold', 50).value)

        self.distance_tolerance = 0.1
        self.heading_tolerance = 0.1

        self.current_pose = None
        self.initial_pose = None
        self.goal_pose = None
        self.path_points = []
        self.active_waypoint = 0
        self.latest_scan = None

        self.map_resolution = None
        self.map_origin = (0.0, 0.0)
        self.map_width = 0
        self.map_height = 0
        self.occupancy = None
        self.inflated = None
        self.inflation_radius_cells = 1
        self.map_frame_id = 'map'

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.calc_time_pub = self.create_publisher(Float32, 'astar_time', 10)

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, map_qos)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.goal_subscriptions = []
        for topic in ('/goal_pose', '/move_base_simple/goal'):
            self.goal_subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    topic,
                    lambda msg, src=topic: self.goal_callback(msg, src),
                    10,
                )
            )

        self.planner_start_time = None
        self._map_logged = False

        self.get_logger().info('AutoNavigator node configured')

    # --- Subscriber callbacks -------------------------------------------------

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        if self.initial_pose is None:
            self.initial_pose = self.current_pose
            self.get_logger().info('Initial pose stored')

        if self.path_points and self.active_waypoint < len(self.path_points):
            self.follow_path()

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def map_callback(self, msg: OccupancyGrid):
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_frame_id = msg.header.frame_id or 'map'

        grid = np.array(msg.data, dtype=np.int16).reshape((self.map_height, self.map_width))
        grid = np.where(grid < 0, 100, grid)
        occupancy = (grid >= self.occupied_threshold).astype(np.uint8)
        self.occupancy = occupancy

        cells = max(1, int(math.ceil(self.inflation_radius_m / self.map_resolution)))
        self.inflation_radius_cells = cells
        self.inflated = self.inflate_obstacles(occupancy, cells)

        if not self._map_logged:
            self._map_logged = True
            diam = self.inflation_radius_cells * self.map_resolution
            self.get_logger().info(f'Map received, inflation radius {self.inflation_radius_cells} cells (~{diam:.2f} m)')

    def goal_callback(self, msg: PoseStamped, source_topic: str):
        self.get_logger().info(f'Goal from {source_topic}')
        self.set_goal(msg.pose.position.x, msg.pose.position.y)

    # --- Planning -------------------------------------------------------------

    def set_goal(self, x: float, y: float):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        self.goal_pose = pose
        self.get_logger().info(f'Stored goal at ({x:.2f}, {y:.2f})')

        if self.current_pose is None:
            self.get_logger().warn('Cannot plan yet, robot pose unavailable')
            return
        if self.inflated is None:
            self.get_logger().warn('Cannot plan yet, map not processed')
            return
        self.plan_path()

    def plan_path(self):
        self.planner_start_time = self.get_clock().now().nanoseconds * 1e-9

        start_cell = self.world_to_cell(self.current_pose.position.x, self.current_pose.position.y)
        goal_cell = self.world_to_cell(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)

        if start_cell is None or goal_cell is None:
            self.get_logger().warn('Start or goal is outside the occupancy grid')
            self.clear_path()
            return

        if self.cell_blocked(start_cell):
            self.get_logger().warn('Start location is not navigable after inflation')
            self.clear_path()
            return

        if self.cell_blocked(goal_cell):
            self.get_logger().warn('Goal location is not navigable after inflation')
            self.clear_path()
            return

        frontier = PriorityQueue()
        frontier.put((0.0, start_cell))
        came_from = {start_cell: None}
        g_cost = {start_cell: 0.0}

        goal_reached = False

        while not frontier.empty():
            _, current = frontier.get()
            if current == goal_cell:
                goal_reached = True
                break

            for neighbor in self.get_neighbors(current):
                if self.cell_blocked(neighbor):
                    continue

                new_cost = g_cost[current] + self.transition_cost(current, neighbor)
                if neighbor not in g_cost or new_cost < g_cost[neighbor]:
                    g_cost[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal_cell)
                    frontier.put((priority, neighbor))
                    came_from[neighbor] = current

        if not goal_reached:
            self.get_logger().warn('Planner could not connect start and goal')
            self.clear_path()
            return

        path_cells = []
        cell = goal_cell
        while cell is not None:
            path_cells.append(cell)
            cell = came_from[cell]
        path_cells.reverse()

        self.path_points = [self.cell_to_world(r, c) for r, c in path_cells]
        self.active_waypoint = 0
        self.publish_path()

        duration = self.get_clock().now().nanoseconds * 1e-9 - self.planner_start_time
        self.planner_start_time = None
        time_msg = Float32()
        time_msg.data = float(duration)
        self.calc_time_pub.publish(time_msg)

        self.get_logger().info(f'New plan with {len(self.path_points)} waypoints')

    def clear_path(self):
        self.path_points = []
        self.active_waypoint = 0
        self.publish_path()

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = self.map_frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        poses = []
        for x, y in self.path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            poses.append(pose)

        path_msg.poses = poses
        self.path_pub.publish(path_msg)

    # --- Path following -------------------------------------------------------

    def follow_path(self):
        if not self.path_points or self.active_waypoint >= len(self.path_points):
            return

        target_x, target_y = self.path_points[self.active_waypoint]
        pose = self.current_pose
        robot_x = pose.position.x
        robot_y = pose.position.y
        yaw = self.quaternion_to_yaw(pose.orientation)

        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_heading - yaw)

        cmd = Twist()

        if abs(heading_error) > self.heading_tolerance:
            cmd.angular.z = 0.5 if heading_error > 0.0 else -0.5
        else:
            cmd.linear.x = min(0.2, distance)
            cmd.angular.z = 0.2 * heading_error

        self.cmd_vel_pub.publish(cmd)

        if distance < self.distance_tolerance:
            self.active_waypoint += 1
            if self.active_waypoint >= len(self.path_points):
                self.get_logger().info('Arrived at final waypoint')
                self.cmd_vel_pub.publish(Twist())

    # --- Helpers --------------------------------------------------------------

    def inflate_obstacles(self, grid: np.ndarray, radius: int) -> np.ndarray:
        if radius <= 0:
            return grid.copy()
        inflated = grid.copy()
        occupied_indices = np.argwhere(grid > 0)
        for r, c in occupied_indices:
            r_min = max(0, r - radius)
            r_max = min(self.map_height, r + radius + 1)
            c_min = max(0, c - radius)
            c_max = min(self.map_width, c + radius + 1)
            inflated[r_min:r_max, c_min:c_max] = 1
        return inflated

    def world_to_cell(self, x: float, y: float):
        if self.map_resolution is None:
            return None
        col = int((x - self.map_origin[0]) / self.map_resolution)
        row = int((y - self.map_origin[1]) / self.map_resolution)
        if 0 <= row < self.map_height and 0 <= col < self.map_width:
            return (row, col)
        return None

    def cell_to_world(self, row: int, col: int):
        x = self.map_origin[0] + (col + 0.5) * self.map_resolution
        y = self.map_origin[1] + (row + 0.5) * self.map_resolution
        return (x, y)

    def cell_blocked(self, cell):
        if self.inflated is None:
            return True
        r, c = cell
        return self.inflated[r, c] != 0

    def get_neighbors(self, cell):
        r, c = cell
        candidates = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ]
        neighbors = []
        for dr, dc in candidates:
            nr = r + dr
            nc = c + dc
            if nr < 0 or nr >= self.map_height or nc < 0 or nc >= self.map_width:
                continue
            if dr != 0 and dc != 0:
                if self.inflated[r, nc] != 0 or self.inflated[nr, c] != 0:
                    continue
            neighbors.append((nr, nc))
        return neighbors

    def transition_cost(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = AutoNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down AutoNavigator')
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
