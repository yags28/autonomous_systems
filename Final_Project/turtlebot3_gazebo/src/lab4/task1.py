#!/usr/bin/env python3

import math
import random
import threading
import time
import heapq
import numpy as np
try:
    import scipy.interpolate as si
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import tf2_ros

# ================================================================
# Parameters (all in-code)
# ================================================================
# These two mainly control “how tight” the robot sticks to the path.
LOOKAHEAD_DISTANCE = 0.20   # base pure pursuit lookahead distance [m]
SPEED               = 0.20  # nominal linear speed [m/s]

EXPANSION_SIZE      = 4     # obstacle inflation radius [cells]
TARGET_ERROR        = 0.10  # distance threshold for goal reached [m]

# rotation speed when aligning to a new path (rad/s)
ROTATE_SPEED        = 1.15

# simple reactive collision handling
FRONT_ARC_DEG       = 40.0  # cone in front of robot to check [deg]
COLLISION_DIST      = 0.35  # if obstacle closer than this -> back up + replan [m]
BACK_SPEED          = 0.10  # backwards linear speed [m/s]
BACK_TIME           = 0.7   # seconds to back up when obstacle is detected

# Occupancy values
OCC_THRESH          = 50      # >= this => obstacle
UNKNOWN_VAL         = -1      # from SLAM toolbox

# steering gain for pure pursuit (higher => tighter tracking)
STEER_GAIN          = 1.6
MAX_ANG_VEL         = 1.0     # rad/s clamp for angular velocity

# clear stubborn unknown cells robot drives through
CLEAR_UNKNOWN_RADIUS_CELLS = 3  # cells around robot to force free

# RRT parameters (removed — we will not use RRT)
# RRT_MAX_ITERS, RRT_STEP_SIZE, etc removed

# Global holder used between exploration() and control loop
pathGlobal = 0


# ================================================================
# Utility functions
# ================================================================
def normalize_angle(a):
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def heuristic(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def astar(array, start, goal):
    """
    A* on a 2D grid:
      array[y, x] == 1   => obstacle
      array[y, x] == 0   => free (we treat unknown as 0 as well)

    Kept as the primary planner in this A*-only version.
    array is a 2D numpy array with shape [H, W]. start and goal are (row, col).
    Returns path as list of (row, col) from start to goal inclusive, or False if not found.
    """
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0),
                 (1, 1), (1, -1), (-1, 1), (-1, -1)]

    close_set = set()
    came_from = {}
    gscore = {start: 0.0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            data = data[::-1]
            return data

        close_set.add(current)

        for i, j in neighbors:
            neighbor = (current[0] + i, current[1] + j)
            tentative_g_score = gscore[current] + heuristic(current, neighbor)

            if not (0 <= neighbor[0] < array.shape[0] and
                    0 <= neighbor[1] < array.shape[1]):
                continue

            if array[neighbor[0], neighbor[1]] == 1:
                continue  # obstacle

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                continue

            if tentative_g_score < gscore.get(neighbor, float('inf')) or \
               neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return False


def bspline_planning(array, sn):
    """
    B-spline smoothing for a path (world coordinates).
    If SciPy is unavailable or something goes wrong, returns the original path.
    """
    if not HAS_SCIPY:
        return array

    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        N = 2
        t = range(len(x))
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)

        x_list = list(x_tup)
        xl = x.tolist()
        x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

        y_list = list(y_tup)
        yl = y.tolist()
        y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

        ipl_t = np.linspace(0.0, len(x) - 1, sn)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)
        path = [(rx[i], ry[i]) for i in range(len(rx))]
    except Exception:
        path = array
    return path


def calc_target_index(cx, cy, x, y, Lf):
    """
    Always recompute nearest point on the path, then walk forward
    until we are ~Lf away. This helps keep the robot centered and
    avoids drifting index issues.
    """
    dx = [cx[i] - x for i in range(len(cx))]
    dy = [cy[i] - y for i in range(len(cy))]
    d  = [math.hypot(dx[i], dy[i]) for i in range(len(dx))]

    ind = int(np.argmin(d))

    while ind < len(cx) and d[ind] < Lf:
        ind += 1
        if ind < len(cx):
            dx[ind] = cx[ind] - x
            dy[ind] = cy[ind] - y
            d[ind]  = math.hypot(dx[ind], dy[ind])

    return ind


def pure_pursuit_control(cx, cy, x, y, yaw):
    """
    Dynamic lookahead + speed:
      - Far from goal: normal lookahead and speed.
      - Near goal: shorter lookahead and slower speed.

    Steering is scaled by STEER_GAIN and clamped to MAX_ANG_VEL.
    """
    gx, gy = cx[-1], cy[-1]
    dist_goal = math.hypot(gx - x, gy - y)

    if dist_goal < 0.5:
        Lf = max(0.08, LOOKAHEAD_DISTANCE * 0.6)
        v  = max(0.06, SPEED * (dist_goal / 0.5))
    else:
        Lf = LOOKAHEAD_DISTANCE
        v  = SPEED

    ind = calc_target_index(cx, cy, x, y, Lf)

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - y, tx - x) - yaw
    alpha = normalize_angle(alpha)
    delta = math.atan2(2.0 * math.sin(alpha), 1.0)

    w = STEER_GAIN * delta
    w = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, w))

    return v, w


def frontierB(matrix):
    """Mark free cells that touch unknown as frontier (value 2)."""
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if matrix[i][j] == 0.0:
                if i > 0 and matrix[i - 1][j] < 0:
                    matrix[i][j] = 2
                elif i < len(matrix) - 1 and matrix[i + 1][j] < 0:
                    matrix[i][j] = 2
                elif j > 0 and matrix[i][j - 1] < 0:
                    matrix[i][j] = 2
                elif j < len(matrix[i]) - 1 and matrix[i][j + 1] < 0:
                    matrix[i][j] = 2
    return matrix


def dfs(matrix, i, j, group, groups):
    if i < 0 or i >= len(matrix) or j < 0 or j >= len(matrix[0]):
        return group
    if matrix[i][j] != 2:
        return group

    groups.setdefault(group, []).append((i, j))
    matrix[i][j] = 0

    dfs(matrix, i + 1, j, group, groups)
    dfs(matrix, i - 1, j, group, groups)
    dfs(matrix, i, j + 1, group, groups)
    dfs(matrix, i, j - 1, group, groups)
    dfs(matrix, i + 1, j + 1, group, groups)
    dfs(matrix, i - 1, j - 1, group, groups)
    dfs(matrix, i - 1, j + 1, group, groups)
    dfs(matrix, i + 1, j - 1, group, groups)

    return group + 1


def assign_groups(matrix):
    group = 1
    groups = {}
    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            if matrix[i][j] == 2:
                group = dfs(matrix, i, j, group, groups)
    return matrix, groups


def fGroups(groups):
    sorted_groups = sorted(groups.items(), key=lambda x: len(x[1]), reverse=True)
    top_groups = [g for g in sorted_groups[:5] if len(g[1]) > 2]
    return top_groups


def calculate_centroid(x_coords, y_coords):
    n = len(x_coords)
    sum_x = sum(x_coords)
    sum_y = sum(y_coords)
    mean_x = sum_x / n
    mean_y = sum_y / n
    centroid = (int(mean_x), int(mean_y))
    return centroid


def pathLength(path):
    pts = np.array([(p[0], p[1]) for p in path])
    if len(pts) < 2:
        return 0.0
    diffs = np.diff(pts, axis=0)
    d = np.hypot(diffs[:, 0], diffs[:, 1])
    return float(np.sum(d))


# ================================================================
# Costmap / frontier helpers
# ================================================================
def costmap(data, width, height, resolution):
    """
    Returns:
      data_metric: float array (shape [H,W]) (0 free / 100 occ / -1 unknown) with inflation.
      inflated_occ: int8 array for RViz OccupancyGrid.
    """
    arr = np.array(data, dtype=np.int16).reshape(height, width)
    arr_unknown = (arr == UNKNOWN_VAL)

    # inflate obstacles
    wall_y, wall_x = np.where(arr == 100)
    for i in range(-EXPANSION_SIZE, EXPANSION_SIZE + 1):
        for j in range(-EXPANSION_SIZE, EXPANSION_SIZE + 1):
            if i == 0 and j == 0:
                continue
            y = np.clip(wall_y + i, 0, height - 1)
            x = np.clip(wall_x + j, 0, width - 1)
            arr[y, x] = 100

    inflated_occ = arr.copy()
    inflated_occ[arr_unknown] = UNKNOWN_VAL

    data_metric = arr.astype(float)
    return data_metric, inflated_occ


def best_viewpoint_for_group(matrix, group_cells, unknown_map,
                             max_steps=6, info_radius=3):
    """
    matrix:   2D, obstacles 1, free/unknown 0
    unknown_map: 2D, unknown < 0, others >= 0
    BFS from frontier cluster centroid, preferring deeper cells into unknown.
    """
    from collections import deque

    h, w = matrix.shape

    cy, cx = calculate_centroid(
        [p[0] for p in group_cells],
        [p[1] for p in group_cells]
    )

    cy = int(np.clip(cy, 0, h - 1))
    cx = int(np.clip(cx, 0, w - 1))

    q = deque()
    visited = set()
    q.append((cy, cx, 0))
    visited.add((cy, cx))

    best_cell = (cy, cx)
    best_info = -1

    while q:
        y, x, d = q.popleft()
        if d > max_steps:
            break

        if matrix[y, x] != 1:   # not obstacle
            y0 = max(0, y - info_radius)
            y1 = min(h - 1, y + info_radius)
            x0 = max(0, x - info_radius)
            x1 = min(w - 1, x + info_radius)
            window = unknown_map[y0:y1+1, x0:x1+1]
            info = int(np.sum(window < 0))

            if info > best_info:
                best_info = info
                best_cell = (y, x)

        for dy, dx in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            ny = y + dy
            nx = x + dx
            if 0 <= ny < h and 0 <= nx < w and (ny, nx) not in visited:
                visited.add((ny, nx))
                q.append((ny, nx, d + 1))

    return best_cell


def extend_into_unknown(start_cell, current_cell, unknown_map, max_extra=8):
    """
    From a base frontier cell, push the goal deeper into unknown along the
    ray from robot -> frontier, as long as the cells ahead are still unknown.
    """
    h, w = unknown_map.shape
    cy, cx = current_cell
    sy, sx = start_cell

    dy = sy - cy
    dx = sx - cx
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return start_cell

    stepy = dy / norm
    stepx = dx / norm

    best = start_cell
    for k in range(1, max_extra + 1):
        ny = int(round(sy + stepy * k))
        nx = int(round(sx + stepx * k))
        if not (0 <= ny < h and 0 <= nx < w):
            break
        if unknown_map[ny, nx] < 0:  # still unknown -> deeper is better
            best = (ny, nx)
        else:
            break

    return best


# ================================================================
# findClosestGroup uses A* now (replacing RRT calls)
# ================================================================
def findClosestGroup(matrix, groups, current, resolution,
                     originX, originY, unknown_map):
    """
    Choose best frontier cluster, and plan path to a viewpoint using A*.

    matrix: 2D grid for planning (1=obstacle, 0=free+unknown)
    current: (row, col) robot cell in matrix
    """
    targetP = None
    distances = []
    paths = []
    scores = []
    max_score = -1

    # world coords of robot start
    start_world = (
        current[1] * resolution + originX,
        current[0] * resolution + originY
    )

    for i in range(len(groups)):
        group_cells = groups[i][1]

        base_cell = best_viewpoint_for_group(matrix, group_cells, unknown_map)
        # push target deeper inside unknown in direction of this group
        target_cell = extend_into_unknown(base_cell, current, unknown_map)

        # convert target cell to world
        goal_world = (
            target_cell[1] * resolution + originX,
            target_cell[0] * resolution + originY
        )

        # --- A* planning on grid first (matrix is already 1 obstacle / 0 free+unknown)
        # convert start and goal into grid coords for astar: start and goal are (row, col)
        try:
            path_cells = astar(matrix, current, target_cell)
        except Exception:
            path_cells = None

        world_path = None
        if path_cells:
            # Convert cells (row,col) to world coordinates (x,y)
            world_path = [(p[1] * resolution + originX,
                           p[0] * resolution + originY)
                          for p in path_cells]
        else:
            # fallback to attempt an A* from robot to a random cell of the group (existing fallback behavior)
            path_cells_fb = astar(matrix, current, (group_cells[0][0], group_cells[0][1]))
            if path_cells_fb:
                world_path = [(p[1] * resolution + originX,
                               p[0] * resolution + originY)
                              for p in path_cells_fb]
            else:
                world_path = None

        if not world_path:
            distances.append(0.0)
            paths.append(None)
            continue

        total_distance = pathLength(world_path)
        distances.append(total_distance)
        paths.append(world_path)

    # Score = (cluster size / path length), ignore too tiny goals
    for i in range(len(distances)):
        if distances[i] == 0.0 or paths[i] is None:
            scores.append(0.0)
        else:
            scores.append(len(groups[i][1]) / distances[i])

    for i in range(len(distances)):
        if distances[i] > TARGET_ERROR * 3:
            if max_score == -1 or scores[i] > scores[max_score]:
                max_score = i

    if max_score != -1:
        targetP = paths[max_score]
    else:
        # fallback: random frontier cell (no deep extension here)
        index  = random.randint(0, len(groups) - 1)
        target = groups[index][1]
        target = target[random.randint(0, len(target) - 1)]
        path_cells = astar(matrix, current, target)
        if path_cells:
            targetP = [(p[1] * resolution + originX,
                        p[0] * resolution + originY) for p in path_cells]
        else:
            targetP = None

    return targetP


# --- helper: clear unknown around robot cell --------------------
def clear_unknown_around_robot(grid, row, col, radius):
    """
    Any unknown cell (value < 0) in a small neighborhood around the
    robot is treated as free (0). Obstacles (>=0, in particular 100)
    are untouched.
    """
    h, w = grid.shape
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            ry = row + dy
            rx = col + dx
            if 0 <= ry < h and 0 <= rx < w:
                if grid[ry, rx] < 0:   # unknown only
                    grid[ry, rx] = 0.0


def exploration(node, data, width, height, resolution, col, row, originX, originY):
    """
    Frontier-based exploration:
    - inflate obstacles
    - detect frontier clusters
    - choose a viewpoint *inside* unknown
    - plan with A* (world coords derived from grid cells)
    - treat cells the robot passes through as free unless obstacle
    """
    global pathGlobal

    grid, inflated_occ = costmap(data, width, height, resolution)
    node.publish_costmap(inflated_occ, width, height, resolution, originX, originY)

    # robot cell = free
    grid[row, col] = 0

    # clear stubborn unknown cells around robot path
    clear_unknown_around_robot(grid, row, col, CLEAR_UNKNOWN_RADIUS_CELLS)

    # unknown_map keeps -1 where unknown (after clearing)
    unknown_map = grid.copy()

    # for frontier detection and A*:
    #   >50 -> obstacle (1)
    #   0 or -1 -> free or unknown
    grid_binary = grid.copy()
    grid_binary[grid_binary > OCC_THRESH] = 1

    # frontier detection sees:
    #   0 => free
    #   <0 => unknown
    frontier_grid = frontierB(grid_binary.copy())
    frontier_grid, groups = assign_groups(frontier_grid)
    groups = fGroups(groups)

    if len(groups) == 0:
        pathGlobal = -1
        return

    # planning matrix: 1 obstacle, 0 free+unknown
    plan_grid = grid_binary.copy()
    plan_grid[plan_grid < 0] = 0

    path = findClosestGroup(
        plan_grid,
        groups,
        (row, col),
        resolution,
        originX,
        originY,
        unknown_map
    )

    if path is not None:
        path = bspline_planning(path, max(len(path) * 5, len(path)))
    else:
        path = -1

    pathGlobal = path


# ================================================================
# Navigation node (pure pursuit + simple back-up-on-collision)
# ================================================================
class navigationControl(Node):
    def __init__(self):
        super().__init__('task1_algorithm')

        self.create_subscription(OccupancyGrid, 'map',  self.map_callback, 10)
        self.create_subscription(Odometry,      'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan,     'scan', self.scan_callback, 10)

        self.cmd_pub     = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_pub    = self.create_publisher(Path,  '/planned_path', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/inflated_map', 10)

        # TF buffer to get pose in MAP frame (map -> base_footprint)
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.kesif = True

        print("[INFO] Task1 exploration node (frontier + deep unknown goals + A* + backup)")
        threading.Thread(target=self.exp, daemon=True).start()

    # -------------- callbacks --------------
    def scan_callback(self, msg):
        self.scan_data = msg

    def map_callback(self, msg):
        self.map_data_msg = msg
        self.width        = msg.info.width
        self.height       = msg.info.height
        self.resolution   = msg.info.resolution
        self.originX      = msg.info.origin.position.x
        self.originY      = msg.info.origin.position.y
        self.data         = msg.data

    def odom_callback(self, msg):
        # Only store raw odom; pose (x,y,yaw) comes from TF in map frame
        self.odom_data = msg

    # -------------- pose from TF (map frame) --------------
    def update_pose_from_tf(self):
        """
        Use TF to get robot pose in the map frame so that
        path (computed in map) and control use the same frame.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time())
        except Exception:
            return False

        self.x = t.transform.translation.x
        self.y = t.transform.translation.y
        q = t.transform.rotation
        _, _, self.yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        return True

    # -------------- map helpers --------------
    def world_to_map(self, x, y):
        ix = int((x - self.originX) / self.resolution)
        iy = int((y - self.originY) / self.resolution)
        return ix, iy

    def in_bounds(self, ix, iy):
        return 0 <= ix < self.width and 0 <= iy < self.height

    # -------------- RViz costmap publisher --------------
    def publish_costmap(self, inflated_occ, width, height, resolution, originX, originY):
        msg = OccupancyGrid()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = resolution
        msg.info.width      = width
        msg.info.height     = height
        msg.info.origin.position.x = originX
        msg.info.origin.position.y = originY
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = inflated_occ.reshape(-1).astype(np.int8).tolist()
        self.costmap_pub.publish(msg)

    # -------------- path validity (cancel only when *real* obstacles appear) --------------
    def path_still_valid(self, path):
        """
        Validate against SAME obstacle representation as the map
        (no extra inflation here), so we don't kill paths just
        because they run close to walls.
        """
        if path is None or isinstance(path, int):
            return False
        if not hasattr(self, 'data'):
            return False

        arr = np.array(self.data, dtype=np.int16).reshape(self.height, self.width)
        grid = np.zeros_like(arr, dtype=np.int8)
        grid[arr > OCC_THRESH] = 1          # 1 = obstacle

        for k, (x, y) in enumerate(path):
            ix, iy = self.world_to_map(x, y)
            if not self.in_bounds(ix, iy):
                self.get_logger().warn("[WARN] Path leaves map; cancelling and replanning.")
                return False

            # ignore the first few points near the robot to avoid self-inflation noise
            if k < 3:
                continue

            if grid[iy, ix] == 1:
                self.get_logger().warn("[WARN] Path cell became occupied; cancelling and replanning.")
                return False

        return True

    # -------------- rotate in place to face the new path --------------
    def rotate_towards_path(self, cx, cy):
        """
        Rotate in place to face a lookahead point on the new path
        before starting to follow it.
        """
        if len(cx) < 2:
            return

        look_idx = min(5, len(cx) - 1)
        tx = cx[look_idx]
        ty = cy[look_idx]

        twist = Twist()
        while rclpy.ok():
            # keep pose in map frame updated while rotating
            if not self.update_pose_from_tf():
                time.sleep(0.05)
                continue

            dx = tx - self.x
            dy = ty - self.y
            desired_yaw = math.atan2(dy, dx)
            err = normalize_angle(desired_yaw - self.yaw)

            if abs(err) < 0.05:  # ~3 degrees
                break

            twist.linear.x  = 0.0
            twist.angular.z = ROTATE_SPEED if err > 0.0 else -ROTATE_SPEED
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    # -------------- check front lidar for obstacle --------------
    def front_obstacle_detected(self):
        if not hasattr(self, 'scan_data'):
            return False

        scan   = self.scan_data
        ranges = np.array(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment

        front_mask   = np.abs(angles) < math.radians(FRONT_ARC_DEG)
        front_ranges = ranges[front_mask]
        front_ranges = front_ranges[np.isfinite(front_ranges)]

        if front_ranges.size == 0:
            return False

        min_front = front_ranges.min()
        return min_front < COLLISION_DIST

    # -------------- perform back-up maneuver --------------
    def backup_from_obstacle(self):
        self.get_logger().warn("[WARN] Obstacle detected in front; backing up and replanning.")
        twist = Twist()
        t_end = time.time() + BACK_TIME
        while time.time() < t_end and rclpy.ok():
            twist.linear.x  = -BACK_SPEED
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

        twist.linear.x  = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    # -------------- main exploration loop --------------
    def exp(self):
        global pathGlobal
        twist = Twist()

        while True:
            if (not hasattr(self, 'data') or
                not hasattr(self, 'odom_data') or
                not hasattr(self, 'scan_data')):
                time.sleep(0.1)
                continue

            # update pose in MAP frame; if TF not ready, skip this cycle
            if not self.update_pose_from_tf():
                time.sleep(0.05)
                continue

            if self.kesif:
                # get / recompute path if needed
                if isinstance(pathGlobal, int) and pathGlobal == 0:
                    col = int((self.x - self.originX) / self.resolution)
                    row = int((self.y - self.originY) / self.resolution)
                    exploration(self, self.data, self.width, self.height,
                                self.resolution, col, row,
                                self.originX, self.originY)
                    self.path = pathGlobal
                else:
                    self.path = pathGlobal

                if isinstance(self.path, int) and self.path == -1:
                    twist.linear.x  = 0.0
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    print("[INFO] Exploration finished.")
                    self.kesif = False
                    continue

                cx = [p[0] for p in self.path]
                cy = [p[1] for p in self.path]

                # publish planned path to RViz
                path_msg = Path()
                path_msg.header.stamp    = self.get_clock().now().to_msg()
                path_msg.header.frame_id = "map"
                for px, py in zip(cx, cy):
                    ps = PoseStamped()
                    ps.header.frame_id = "map"
                    ps.pose.position.x = px
                    ps.pose.position.y = py
                    ps.pose.position.z = 0.0
                    ps.pose.orientation.w = 1.0
                    path_msg.poses.append(ps)
                self.path_pub.publish(path_msg)

                # rotate to face new path (no curved initial motion)
                self.rotate_towards_path(cx, cy)

                step_counter = 0

                while True:
                    if not self.kesif:
                        break

                    # keep pose updated while following path
                    if not self.update_pose_from_tf():
                        time.sleep(0.05)
                        continue

                    # --- clean stop at end of path ---
                    dist_goal = math.hypot(cx[-1] - self.x, cy[-1] - self.y)
                    if dist_goal < TARGET_ERROR:
                        twist.linear.x  = 0.0
                        twist.angular.z = 0.0
                        self.cmd_pub.publish(twist)
                        break

                    # --- front obstacle -> backup + replan ---
                    if self.front_obstacle_detected():
                        self.backup_from_obstacle()
                        pathGlobal = 0
                        break

                    # every few steps, re-check that path is still valid
                    if step_counter % 10 == 0:
                        if not self.path_still_valid(self.path):
                            pathGlobal = 0
                            break

                    # tighter pure pursuit tracking
                    v, w = pure_pursuit_control(
                        cx, cy,
                        self.x, self.y, self.yaw
                    )

                    twist.linear.x  = v
                    twist.angular.z = w
                    self.cmd_pub.publish(twist)

                    step_counter += 1
                    time.sleep(0.05)

                pathGlobal = 0


# ================================================================
# main
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = navigationControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # stop robot on shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
