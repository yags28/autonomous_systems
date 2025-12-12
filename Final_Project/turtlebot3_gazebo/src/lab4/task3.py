#!/usr/bin/env python3
import math
import heapq
import random
import threading
import time
import os
import yaml
from collections import deque
from enum import Enum

import numpy as np
if not hasattr(np, 'float'):
    np.float = float

import cv2 
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Point, PointStamped
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge

try:
    from rclpy._rclpy_pybind11 import RCLError
except Exception:
    RCLError = Exception

# -------------------- PID Controller -------------------- #
class PIDController:
    def __init__(self, kp, ki, kd, max_out):
        self.kp = kp; self.ki = ki; self.kd = kd; self.max_out = max_out
        self.prev_error = 0.0; self.integral = 0.0; self.last_time = None

    def compute(self, error, current_time):
        if self.last_time is None: dt = 0.1
        else: dt = current_time - self.last_time
        if dt <= 0: dt = 0.001
        self.integral += error * dt
        self.integral = max(min(self.integral, 1.0), -1.0)
        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error; self.last_time = current_time
        return max(min(output, self.max_out), -self.max_out)
    
    def reset(self):
        self.prev_error = 0.0; self.integral = 0.0; self.last_time = None

# -------------------- Utility -------------------- #
def get_yaw_from_quaternion(q):
    quat = [q.x, q.y, q.z, q.w]
    _, _, yaw = euler_from_quaternion(quat)
    return yaw

def normalize_angle(a):
    while a > math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a

# -------------------- Graph + A* -------------------- #
class GraphNode:
    def __init__(self, name: str):
        self.name = name; self.neighbors = []; self.costs = []
    def add_edges(self, nums, ws):
        self.neighbors.extend(nums); self.costs.extend(ws)

class GridGraph:
    def __init__(self): self.nodes = {}

class AStar:
    def __init__(self, graph: GridGraph):
        self.graph = graph; self.prev = {}; self.g = {}
    
    @staticmethod
    def _parse(name: str):
        r, c = name.split(',')
        return int(r), int(c)

    def plan(self, start_name: str, goal_name: str):
        self.prev = {}; self.g = {}
        if start_name not in self.graph.nodes or goal_name not in self.graph.nodes: return False
        gr, gc = self._parse(goal_name)
        def h(nm: str):
            r, c = self._parse(nm)
            return math.hypot(gr - r, gc - c)
        open_heap = []
        heapq.heappush(open_heap, (h(start_name), start_name))
        self.g[start_name] = 0.0
        closed = set()
        while open_heap:
            _, cur = heapq.heappop(open_heap)
            if cur in closed: continue
            closed.add(cur)
            if cur == goal_name: return True
            node = self.graph.nodes[cur]
            curg = self.g[cur]
            for nb_node, w in zip(node.neighbors, node.costs):
                nb = nb_node.name; ng = curg + float(w)
                if (nb not in self.g) or (ng < self.g[nb]):
                    self.g[nb] = ng; self.prev[nb] = cur
                    heapq.heappush(open_heap, (ng + h(nb), nb))
        return False

    def reconstruct(self, start_name: str, goal_name: str):
        if goal_name not in self.g: return []
        out = []; cur = goal_name
        while cur is not None:
            out.append(cur); cur = self.prev.get(cur, None)
        out.reverse()
        if not out or out[0] != start_name: return []
        return out

# -------------------- RRT* Node -------------------- #
class RRTStarNode:
    def __init__(self, x, y, parent=None, cost=0.0):
        self.x = x; self.y = y; self.parent = parent; self.cost = cost

# -------------------- Task3 Navigator -------------------- #
class Task3Navigator(Node):
    def __init__(self):
        super().__init__('task3_algorithm')
        self.get_logger().info("[INIT] Task3: LiDAR Fusion + Publishing /color_pos topics")

        # ---------- Waypoints ----------
        self.waypoints = [
            (8.157, -2.360), (6.894, 1.730), (2.996, 2.959),
            (0.544, 2.926), (-4.076, 2.908), (-4.438, -2.303)
        ]
        self.wp_idx = 0

        # ---------- Map settings ----------
        self.treat_unknown_as_occupied = True
        self.robot_radius = 0.19
        self.extra_inflation_strict = 0.11
        self.extra_inflation_relaxed = 0.07

        # ---------- Controller (pure pursuit) ----------
        self.lookahead = 0.35
        self.wp_tol = 0.22
        self.max_lin = 0.20
        self.max_ang = 0.85
        self.rotate_threshold = 0.75  # rad
        self.smoothing_alpha = 0.40
        self.v_last = 0.0; self.w_last = 0.0

        # ---------- Planning State ----------
        self.current_astar_path = Path()
        self.astar_progress_idx = 0
        self.astar_frozen = False
        self.astar_using_relaxed = False

        self.ASTAR_MAX_TRIES = 2
        self.RRT_MAX_TRIES = 2
        self.FAIL_COOLDOWN_SEC = 1.0
        self.SKIP_WP_AFTER_TOTAL_FAILS = 8
        self.astar_try_count = 0; self.rrt_try_count = 0
        self.total_fail_count = 0; self.last_fail_time = -1e9

        # ---------- RRT* params ----------
        self.rrt_trigger_dist = 0.75
        self.rrt_clearance = 0.35
        self.rrt_max_iters = 2600
        self.rrt_step_size = 0.35
        self.rrt_neighbor_radius = 0.7
        self.rrt_goal_radius = 0.35
        self.rrt_goal_sample_rate = 0.10
        self.rrt_collision_step = 0.03
        self.rrt_path = Path()
        self.rrt_has_plan = False
        self.rrt_waypoint_idx = 0
        self.rrt_waypoint_tol = 0.12

        self.state = "WAIT"
        self.backup_steps_remaining = 0

        # ---------- Obstacles ----------
        self.dynamic_marks = []
        self.dynamic_ttl = 8.0
        self.permanent_obstacles = [] 
        self.no_look_zones = [] # (x, y, radius)

        # ---------- Vision/Inspection State ----------
        self.bridge = CvBridge()
        self.camera_model = None
        self.frame_count = 0
        self.track_cx = 0.0
        self.track_w = 0.0
        self.track_last_time = 0.0
        self.img_width = 640
        self.target_ball_color = None
        self.tracking_pose = None
        self.processed_balls = []
        self.inspect_start_time = 0.0
        
        # Buffers for averaging
        self.ball_relative_buffer = [] 
        self.pose_buffer = []          
        
        self.pid_heading = PIDController(1.2, 0.01, 0.3, 1.0)
        
        self.colors = {
            "Red":   ([0, 100, 100], [10, 255, 255]),
            "Green": ([40, 70, 70], [80, 255, 255]),
            "Blue":  ([90, 70, 70], [130, 255, 255])
        }
        self.red_upper_range = ([160, 100, 100], [180, 255, 255])
        self.BALL_REAL_DIAMETER = 0.15
        self.STOP_DISTANCE = 0.6

        # ---------- Scanning State ----------
        self.scan_start_yaw = 0.0
        self.scan_accumulated = 0.0
        self.scan_complete = False

        # ---------- ROS ----------
        self.map_loaded = False
        self.have_pose = False
        self.ttbot_pose = PoseStamped()
        self.scan_msg = None
        self.map_occ = None; self.res = 0.05; self.ox = 0.0; self.oy = 0.0; self.W = 0; self.H = 0
        
        self.inflated_strict = None; self.inflated_relaxed = None
        self.graph_strict = None; self.graph_relaxed = None
        self.astar_strict = None; self.astar_relaxed = None

        # Subscribers
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._pose_cb, 10)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        self.create_subscription(Image, '/camera/image_raw', self._image_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self._info_cb, 10)

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, 'map', self._map_cb, map_qos)

        # Publishers
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.wp_marker_pub = self.create_publisher(MarkerArray, '/task3/waypoints', 1)
        self.goal_marker_pub = self.create_publisher(Marker, '/task3/current_goal', 1)
        self.found_marker_pub = self.create_publisher(MarkerArray, '/found_objects', 10)
        self.spotted_pub = self.create_publisher(MarkerArray, '/spotted_debug', 10)
        self.zone_debug_pub = self.create_publisher(MarkerArray, '/no_look_zones', 10)

        # [NEW] Coordinate Publishers
        self.red_pub = self.create_publisher(Point, '/red_pos', 10)
        self.blue_pub = self.create_publisher(Point, '/blue_pos', 10)
        self.green_pub = self.create_publisher(Point, '/green_pos', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.ctrl_timer = self.create_timer(0.1, self._control_loop)
        self.marker_timer = self.create_timer(1.0, self._publish_markers)

        self.get_logger().info("[READY] Waiting for map + amcl_pose...")

    # ---------------- Callbacks ---------------- #
    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.ttbot_pose = ps
        self.have_pose = True

    def _scan_cb(self, msg: LaserScan): self.scan_msg = msg
    def _info_cb(self, msg: CameraInfo): self.camera_model = msg

    def _map_cb(self, msg: OccupancyGrid):
        self.res = msg.info.resolution; self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y; self.W = msg.info.width; self.H = msg.info.height
        occ = np.array(msg.data, dtype=np.int16).reshape(self.H, self.W)
        if self.treat_unknown_as_occupied: occ[occ < 0] = 100
        self.map_occ = occ
        self.inflated_strict = self._inflate(occ, self.extra_inflation_strict)
        self.inflated_relaxed = self._inflate(occ, self.extra_inflation_relaxed)
        self.graph_strict = self._build_graph_from_inflated(self.inflated_strict)
        self.graph_relaxed = self._build_graph_from_inflated(self.inflated_relaxed)
        self.astar_strict = AStar(self.graph_strict)
        self.astar_relaxed = AStar(self.graph_relaxed)
        self.map_loaded = True
        self.get_logger().info(f"[MAP] Loaded {self.W}x{self.H}")
        self._publish_markers()

    # ---------------- Vision ---------------- #
    def _image_cb(self, msg: Image):
        if self.camera_model is None: return
        self.img_width = msg.width
        if self.state not in ["FOLLOW_ASTAR", "FOLLOW_RRT", "SCANNING", "TRACKING", "INSPECTING"]: return
        if self.frame_count % 3 != 0: 
            self.frame_count += 1
            return
        self.frame_count += 1

        try: cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        h, w = cv_img.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.circle(mask, (w // 2, h // 2), int(min(h, w) * 0.45), 255, -1)
        cv_img = cv2.bitwise_and(cv_img, cv_img, mask=mask)
        blurred = cv2.GaussianBlur(cv_img, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask_r1 = cv2.inRange(hsv, np.array(self.colors["Red"][0]), np.array(self.colors["Red"][1]))
        mask_r2 = cv2.inRange(hsv, np.array(self.red_upper_range[0]), np.array(self.red_upper_range[1]))
        self._process_color(cv2.bitwise_or(mask_r1, mask_r2), "Red", msg.header.frame_id)
        for cname, (low, high) in self.colors.items():
            if cname == "Red": continue
            self._process_color(cv2.inRange(hsv, np.array(low), np.array(high)), cname, msg.header.frame_id)

    def _process_color(self, mask, cname, frame_id):
        mask = cv2.erode(mask, None, iterations=2); mask = cv2.dilate(mask, None, iterations=2)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > 300:
                ((cx_f, cy_f), radius_f) = cv2.minEnclosingCircle(c)
                cx = int(cx_f); cy = int(cy_f)
                width_f = radius_f * 2.0 

                perimeter = cv2.arcLength(c, True)
                if perimeter == 0: return
                circularity = 4*math.pi*(area/(perimeter**2))
                x, y, w_b, h_b = cv2.boundingRect(c)
                ratio = float(w_b)/h_b
                
                if ratio < 0.7 or ratio > 1.3: return
                if circularity < 0.6: return

                if self.state in ["TRACKING", "INSPECTING"] and cname == self.target_ball_color:
                    self.track_cx = cx; self.track_w = width_f
                    self.track_last_time = time.time()
                    return

                if self.state in ["FOLLOW_ASTAR", "FOLLOW_RRT", "SCANNING", "WAIT"]:
                    bp = self.calc_ball_coords_visual_only(cx, cy, width_f, frame_id)
                    if bp:
                        if self.is_static_obstacle(bp[0], bp[1]): return 
                        if self.is_in_no_look_zone(bp[0], bp[1]): return

                        self.get_logger().info(f"SPOTTED NEW {cname} Ball! Switching to Visual Tracking.")
                        self.publish_spotted_marker(bp[0], bp[1], cname)
                        self.target_ball_color = cname
                        self.tracking_pose = bp
                        self.track_cx = cx; self.track_w = width_f
                        self.track_last_time = time.time()
                        self.pid_heading.reset()
                        self.state = "TRACKING"

    # ---------------- Geometry/Math Helpers ---------------- #
    
    # Get exact Relative (Distance, Angle) from LiDAR
    def get_lidar_closest_point(self, approx_angle_rad):
        if self.scan_msg is None: return None
        
        # Search Sector +/- 20 degrees
        sector = 0.35 
        min_d = float('inf')
        exact_angle = 0.0
        found = False
        
        angle_curr = self.scan_msg.angle_min
        angle_inc = self.scan_msg.angle_increment
        
        for r in self.scan_msg.ranges:
            if (approx_angle_rad - sector) <= angle_curr <= (approx_angle_rad + sector):
                if 0.15 < r < self.STOP_DISTANCE + 0.5:
                    if r < min_d:
                        min_d = r
                        exact_angle = angle_curr
                        found = True
            angle_curr += angle_inc
            
        if not found: return None
        
        # Return distance to center (surface + radius) and the exact angle
        return (min_d + self.BALL_REAL_DIAMETER/2.0, exact_angle)

    def calc_ball_coords_visual_only(self, u, v, w, frame_id):
        if self.camera_model is None: return None
        K = self.camera_model.k
        Z = (K[0] * self.BALL_REAL_DIAMETER) / w
        if Z > 3.5: return None 
        X = (u - K[2]) * Z / K[0]; Y = (v - K[5]) * (K[4] * self.BALL_REAL_DIAMETER / w) / K[4]
        
        # Transform for rough estimation
        pt = PointStamped(); pt.header.frame_id = frame_id; pt.header.stamp = Time().to_msg()
        pt.point.x = float(X); pt.point.y = float(Y); pt.point.z = float(Z)
        try:
            if not self.tf_buffer.can_transform('map', frame_id, Time(), Duration(seconds=0.1)): return None
            trans = self.tf_buffer.lookup_transform('map', frame_id, Time())
            pt_map = tf2_geometry_msgs.do_transform_point(pt, trans)
            return (pt_map.point.x, pt_map.point.y)
        except Exception: return None

    def is_in_no_look_zone(self, x, y):
        for (zx, zy, zr) in self.no_look_zones:
            if math.hypot(x - zx, y - zy) < zr: return True
        return False
    
    def publish_zones(self):
        ma = MarkerArray()
        for i, (zx, zy, zr) in enumerate(self.no_look_zones):
            m = Marker(); m.header.frame_id = "map"; m.id = i + 5000; m.type = Marker.CYLINDER
            m.action = Marker.ADD; m.pose.position.x = zx; m.pose.position.y = zy; m.pose.position.z = 0.05
            m.scale.x = zr * 2.0; m.scale.y = zr * 2.0; m.scale.z = 0.1
            m.color.r = 0.3; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.3 
            ma.markers.append(m)
        self.zone_debug_pub.publish(ma)

    def is_static_obstacle(self, x, y):
        if self.map_occ is None: return True
        c, r = self.world_to_grid(x, y)
        check_radius = 3 
        for dr in range(-check_radius, check_radius + 1):
            for dc in range(-check_radius, check_radius + 1):
                nr, nc = r+dr, c+dc
                if 0<=nr<self.H and 0<=nc<self.W:
                    if self.map_occ[nr, nc] > 50: return True
        return False

    def publish_spotted_marker(self, x, y, cname):
        ma = MarkerArray()
        m = Marker(); m.header.frame_id="map"; m.header.stamp=self.get_clock().now().to_msg()
        m.ns="spotted"; m.id=999; m.type=Marker.SPHERE; m.action=Marker.ADD
        m.pose.position.x=x; m.pose.position.y=y; m.pose.position.z=0.2
        m.scale.x=0.3; m.scale.y=0.3; m.scale.z=0.3; m.color.a=0.5
        if cname=="Red": m.color.r=1.0
        elif cname=="Green": m.color.g=1.0
        elif cname=="Blue": m.color.b=1.0
        ma.markers.append(m)
        self.spotted_pub.publish(ma)

    # ---------------- Map / Path Helpers ---------------- #
    def _inflate(self, occ_grid: np.ndarray, extra_inflation: float):
        H, W = occ_grid.shape
        base = np.where(occ_grid >= 50, 100, 0).astype(np.uint8)
        r_cells = int(math.ceil((self.robot_radius + float(extra_inflation)) / self.res))
        yy, xx = np.ogrid[-r_cells:r_cells+1, -r_cells:r_cells+1]
        disk = (xx*xx + yy*yy) <= (r_cells*r_cells)
        dil = base.copy()
        for dy in range(-r_cells, r_cells+1):
            for dx in range(-r_cells, r_cells+1):
                if not disk[dy+r_cells, dx+r_cells]: continue
                src_y0=max(0,-dy); src_y1=H-max(0,dy); src_x0=max(0,-dx); src_x1=W-max(0,dx)
                dst_y0=max(0,dy); dst_y1=H-max(0,-dy); dst_x0=max(0,dx); dst_x1=W-max(0,-dx)
                np.maximum(dil[dst_y0:dst_y1, dst_x0:dst_x1], base[src_y0:src_y1, src_x0:src_x1], out=dil[dst_y0:dst_y1, dst_x0:dst_x1])
        return dil

    def _build_graph_from_inflated(self, inflated: np.ndarray):
        H, W = inflated.shape; g = GridGraph()
        for r in range(H):
            for c in range(W):
                if inflated[r, c] == 0: g.nodes[f"{r},{c}"] = GraphNode(f"{r},{c}")
        nbrs = [(-1,0,1.0), (1,0,1.0), (0,-1,1.0), (0,1,1.0),
                (-1,-1,1.414), (-1,1,1.414), (1,-1,1.414), (1,1,1.414)]
        for r in range(H):
            for c in range(W):
                if inflated[r, c] != 0: continue
                parent = g.nodes.get(f"{r},{c}")
                if not parent: continue
                neighbors, weights = [], []
                for dr, dc, w in nbrs:
                    nr, nc = r+dr, c+dc
                    if 0<=nr<H and 0<=nc<W and inflated[nr,nc]==0:
                        neighbors.append(g.nodes[f"{nr},{nc}"]); weights.append(w)
                if neighbors: parent.add_edges(neighbors, weights)
        return g

    def _in_map_world(self, x, y):
        return (self.ox <= x < self.ox + self.W*self.res) and (self.oy <= y < self.oy + self.H*self.res)
    def world_to_grid(self, x, y):
        c = int((x-self.ox)/self.res); r = int((y-self.oy)/self.res)
        return max(0, min(self.W-1, c)), max(0, min(self.H-1, r))
    def grid_to_world(self, c, r):
        return self.ox+(c+0.5)*self.res, self.oy+(r+0.5)*self.res
    def _nearest_free_in(self, inflated: np.ndarray, col, row):
        H, W = self.H, self.W
        if 0<=row<H and 0<=col<W and inflated[row, col]==0: return col, row
        visited = np.zeros((H, W), dtype=np.uint8)
        q = deque(); c0=max(0,min(W-1,col)); r0=max(0,min(H-1,row))
        q.append((c0, r0)); visited[r0, c0]=1
        while q:
            c, r = q.popleft()
            if inflated[r, c]==0: return c, r
            for dc, dr in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nc, nr = c+dc, r+dr
                if 0<=nc<W and 0<=nr<H and not visited[nr, nc]:
                    visited[nr, nc]=1; q.append((nc, nr))
        return c0, r0

    # ---------------- A* / RRT* / Control ---------------- #
    def _plan_astar_frozen(self, gx, gy):
        if not self.map_loaded or not self._in_map_world(gx, gy): return False
        use_relaxed = (self.astar_try_count >= 1)
        self.astar_using_relaxed = use_relaxed
        graph = self.graph_relaxed if use_relaxed else self.graph_strict
        astar = self.astar_relaxed if use_relaxed else self.astar_strict
        inflated = self.inflated_relaxed if use_relaxed else self.inflated_strict

        sx, sy = self.ttbot_pose.pose.position.x, self.ttbot_pose.pose.position.y
        sc, sr = self.world_to_grid(sx, sy); gc, gr = self.world_to_grid(gx, gy)
        sc, sr = self._nearest_free_in(inflated, sc, sr); gc, gr = self._nearest_free_in(inflated, gc, gr)
        s_name = f"{sr},{sc}"; g_name = f"{gr},{gc}"

        if s_name not in graph.nodes or g_name not in graph.nodes: return False
        if not astar.plan(s_name, g_name): return False
        names = astar.reconstruct(s_name, g_name)
        if not names: return False

        path = Path(); path.header.frame_id = 'map'
        for nm in names:
            r, c = map(int, nm.split(','))
            wx, wy = self.grid_to_world(c, r)
            ps = PoseStamped(); ps.header.frame_id = 'map'
            ps.pose.position.x = float(wx); ps.pose.position.y = float(wy); ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self.current_astar_path = path
        self.astar_progress_idx = 0
        self.astar_frozen = True
        self.path_pub.publish(self.current_astar_path)
        return True

    def _closest_path_index(self, path: Path, start_idx: int):
        if not path.poses: return 0
        n = len(path.poses); lo = max(0, start_idx - 25); hi = min(n - 1, start_idx + 80)
        rx = self.ttbot_pose.pose.position.x; ry = self.ttbot_pose.pose.position.y
        best_i = start_idx; best_d2 = float('inf')
        for i in range(lo, hi + 1):
            px = path.poses[i].pose.position.x; py = path.poses[i].pose.position.y
            d2 = (px-rx)**2 + (py-ry)**2
            if d2 < best_d2: best_d2 = d2; best_i = i
        return best_i

    def _lookahead_index(self, path: Path, from_idx: int, L: float):
        n = len(path.poses)
        if n == 0: return 0
        acc = 0.0; i = from_idx
        while i < n - 1 and acc < L:
            x1 = path.poses[i].pose.position.x; y1 = path.poses[i].pose.position.y
            x2 = path.poses[i+1].pose.position.x; y2 = path.poses[i+1].pose.position.y
            acc += math.hypot(x2-x1, y2-y1); i += 1
        return i

    def _pure_pursuit_cmd(self, path: Path):
        if not path.poses: return 0.0, 0.0
        self.astar_progress_idx = self._closest_path_index(path, self.astar_progress_idx)
        li = self._lookahead_index(path, self.astar_progress_idx, self.lookahead)
        goal = path.poses[li].pose.position
        rx = self.ttbot_pose.pose.position.x; ry = self.ttbot_pose.pose.position.y
        yaw = get_yaw_from_quaternion(self.ttbot_pose.pose.orientation)
        dx = goal.x - rx; dy = goal.y - ry
        c = math.cos(-yaw); s = math.sin(-yaw)
        x_r = c * dx - s * dy; y_r = s * dx + c * dy
        Ld = math.hypot(x_r, y_r)
        if Ld < 1e-6: return 0.0, 0.0
        heading_err = math.atan2(y_r, x_r)
        if x_r < 0.0 or abs(heading_err) > self.rotate_threshold:
            v = 0.0; w = max(-self.max_ang, min(self.max_ang, 1.6 * heading_err))
        else:
            curvature = (2.0 * y_r) / (Ld * Ld)
            v = self.max_lin; w = curvature * v
            w = max(-self.max_ang, min(self.max_ang, w))
            turn_scale = max(0.35, 1.0 - 0.7 * min(1.0, abs(w) / self.max_ang))
            v *= turn_scale
        self.v_last = (1.0 - self.smoothing_alpha) * self.v_last + self.smoothing_alpha * v
        self.w_last = (1.0 - self.smoothing_alpha) * self.w_last + self.smoothing_alpha * w
        return self.v_last, self.w_last

    def front_min_distance(self, deg_min=-40, deg_max=40):
        if self.scan_msg is None: return float('inf')
        ranges = np.array(self.scan_msg.ranges, dtype=np.float32)
        angle_min = self.scan_msg.angle_min; angle_inc = self.scan_msg.angle_increment
        angles = angle_min + angle_inc * np.arange(len(ranges))
        a0 = math.radians(deg_min); a1 = math.radians(deg_max)
        mask = (angles >= a0) & (angles <= a1)
        sector = ranges[mask]; sector = sector[np.isfinite(sector)]
        return float(np.min(sector)) if sector.size > 0 else float('inf')

    def estimate_front_obstacle_world(self, deg_min=-30, deg_max=30):
        if self.scan_msg is None or not self.have_pose: return None
        ranges = np.array(self.scan_msg.ranges, dtype=np.float32)
        angles = self.scan_msg.angle_min + self.scan_msg.angle_increment * np.arange(len(ranges))
        a0 = math.radians(deg_min); a1 = math.radians(deg_max)
        mask = (angles >= a0) & (angles <= a1)
        rr = ranges[mask]; aa = angles[mask]; finite = np.isfinite(rr)
        if not finite.any(): return None
        rr2 = rr[finite]; aa2 = aa[finite]
        i = int(np.argmin(rr2)); r = float(rr2[i]); ang = float(aa2[i])
        if r <= 0.05: return None
        rx = self.ttbot_pose.pose.position.x; ry = self.ttbot_pose.pose.position.y
        yaw = get_yaw_from_quaternion(self.ttbot_pose.pose.orientation)
        return rx + r*math.cos(yaw+ang), ry + r*math.sin(yaw+ang), r

    def _is_free_world(self, inflated: np.ndarray, x, y):
        if inflated is None or (not self._in_map_world(x, y)): return False
        c, r = self.world_to_grid(x, y)
        return inflated[r, c] == 0

    def _collision_free_segment(self, inflated: np.ndarray, x1, y1, x2, y2):
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist < 1e-6: return self._is_free_world(inflated, x1, y1)
        steps = int(dist / self.rrt_collision_step) + 1
        for i in range(steps + 1):
            t = i / max(steps, 1); x = x1 + t*(x2-x1); y = y1 + t*(y2-y1)
            if not self._is_free_world(inflated, x, y): return False
        return True

    def _rrt_star_plan(self, inflated: np.ndarray, start_xy, goal_xy):
        sx, sy = start_xy; gx, gy = goal_xy
        if not self._is_free_world(inflated, sx, sy) or not self._is_free_world(inflated, gx, gy): return [], None
        nodes = [RRTStarNode(sx, sy, parent=None, cost=0.0)]
        goal_idx = None; best_cost = float('inf'); found_once = False
        x_min, x_max = self.ox, self.ox + self.W * self.res
        y_min, y_max = self.oy, self.oy + self.H * self.res

        for it in range(self.rrt_max_iters):
            if np.random.rand() < self.rrt_goal_sample_rate: x_rand, y_rand = gx, gy
            else: x_rand, y_rand = np.random.uniform(x_min, x_max), np.random.uniform(y_min, y_max)
            if not self._is_free_world(inflated, x_rand, y_rand): continue

            idx_near, _ = min([(i, (n.x-x_rand)**2+(n.y-y_rand)**2) for i, n in enumerate(nodes)], key=lambda t: t[1])
            n_near = nodes[idx_near]
            theta = math.atan2(y_rand - n_near.y, x_rand - n_near.x)
            dist = math.hypot(x_rand - n_near.x, y_rand - n_near.y)
            step = min(self.rrt_step_size, dist)
            x_new = n_near.x + step * math.cos(theta); y_new = n_near.y + step * math.sin(theta)

            if not self._is_free_world(inflated, x_new, y_new) or not self._collision_free_segment(inflated, n_near.x, n_near.y, x_new, y_new): continue
            new_node = RRTStarNode(x_new, y_new, parent=idx_near, cost=n_near.cost + step)
            
            idx_neighbors = [i for i, ni in enumerate(nodes) if math.hypot(ni.x-x_new, ni.y-y_new) <= self.rrt_neighbor_radius]
            
            for i_n in idx_neighbors:
                ni = nodes[i_n]
                d = math.hypot(ni.x - x_new, ni.y - y_new)
                if ni.cost + d < new_node.cost and self._collision_free_segment(inflated, ni.x, ni.y, x_new, y_new):
                    new_node.cost = ni.cost + d; new_node.parent = i_n
            nodes.append(new_node); new_idx = len(nodes) - 1

            for i_n in idx_neighbors:
                ni = nodes[i_n]; d = math.hypot(ni.x - x_new, ni.y - y_new)
                if new_node.cost + d < ni.cost and self._collision_free_segment(inflated, x_new, y_new, ni.x, ni.y):
                    ni.cost = new_node.cost + d; ni.parent = new_idx

            d_goal = math.hypot(x_new - gx, y_new - gy)
            if d_goal <= self.rrt_goal_radius and self._collision_free_segment(inflated, x_new, y_new, gx, gy):
                goal_cost = new_node.cost + d_goal
                if goal_cost < best_cost:
                    goal_node = RRTStarNode(gx, gy, parent=new_idx, cost=goal_cost)
                    nodes.append(goal_node); goal_idx = len(nodes) - 1; best_cost = goal_cost; found_once = True
            
            if found_once and it > self.rrt_max_iters * 0.6: break
        return nodes, goal_idx

    def _plan_rrt_to_waypoint(self, gx, gy):
        infl = self._inflated_strict_with_dynamic()
        if infl is None: return False
        sx = self.ttbot_pose.pose.position.x; sy = self.ttbot_pose.pose.position.y
        sc, sr = self.world_to_grid(sx, sy); gc, gr = self.world_to_grid(gx, gy)
        sc, sr = self._nearest_free_in(infl, sc, sr); gc, gr = self._nearest_free_in(infl, gc, gr)
        sx, sy = self.grid_to_world(sc, sr); gx, gy = self.grid_to_world(gc, gr)
        nodes, goal_idx = self._rrt_star_plan(infl, (sx, sy), (gx, gy))
        if goal_idx is None: return False
        pts = []; idx = goal_idx
        while idx is not None:
            n = nodes[idx]; pts.append((n.x, n.y)); idx = n.parent
        pts.reverse()
        path = Path(); path.header.frame_id = 'map'
        for (wx, wy) in pts:
            ps = PoseStamped(); ps.header.frame_id = 'map'
            ps.pose.position.x = float(wx); ps.pose.position.y = float(wy); ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.rrt_path = path; self.rrt_has_plan = bool(path.poses); self.rrt_waypoint_idx = 0
        if self.rrt_has_plan: self.path_pub.publish(self.rrt_path)
        return self.rrt_has_plan

    def _follow_rrt_step(self):
        if not self.rrt_has_plan or not self.rrt_path.poses: return 0.0, 0.0
        if self.rrt_waypoint_idx >= len(self.rrt_path.poses): self.rrt_waypoint_idx = len(self.rrt_path.poses) - 1
        wp = self.rrt_path.poses[self.rrt_waypoint_idx].pose.position
        rx = self.ttbot_pose.pose.position.x; ry = self.ttbot_pose.pose.position.y
        yaw = get_yaw_from_quaternion(self.ttbot_pose.pose.orientation)
        dist = math.hypot(wp.x - rx, wp.y - ry); tgt = math.atan2(wp.y - ry, wp.x - rx); err = normalize_angle(tgt - yaw)
        if abs(err) > self.rotate_threshold: v = 0.0; w = max(-self.max_ang, min(self.max_ang, 1.6 * err))
        else: v = self.max_lin * max(0.0, math.cos(err)); w = max(-self.max_ang, min(self.max_ang, 1.2 * err))
        if dist < self.rrt_waypoint_tol:
            if self.rrt_waypoint_idx < len(self.rrt_path.poses) - 1: self.rrt_waypoint_idx += 1
        self.v_last = (1.0 - self.smoothing_alpha) * self.v_last + self.smoothing_alpha * v
        self.w_last = (1.0 - self.smoothing_alpha) * self.w_last + self.smoothing_alpha * w
        return self.v_last, self.w_last

    # ---------------- MAIN CONTROL ---------------- #
    def _control_loop(self):
        if not self.map_loaded or not self.have_pose: return

        # 1. State Machine: SCANNING
        if self.state == "SCANNING":
            rx_yaw = get_yaw_from_quaternion(self.ttbot_pose.pose.orientation)
            if self.scan_start_yaw is None:
                self.scan_start_yaw = rx_yaw; self.scan_accumulated = 0.0; return
            diff = normalize_angle(rx_yaw - self.scan_start_yaw)
            delta = normalize_angle(rx_yaw - self.last_valid_yaw) if hasattr(self, 'last_valid_yaw') else 0.0
            self.scan_accumulated += abs(delta); self.last_valid_yaw = rx_yaw
            if self.scan_accumulated >= 2.0 * math.pi - 0.2:
                self.get_logger().info("Scan Complete. Moving to next WP.")
                self.wp_idx += 1; self.state = "WAIT"; self.scan_start_yaw = None; self.move_ttbot(0.0, 0.0)
            else: self.move_ttbot(0.0, 0.5)
            return

        # 2. State Machine: TRACKING
        if self.state == "TRACKING":
            if time.time() - self.track_last_time > 1.0:
                self.get_logger().warn("Visual Tracking Lost! Resuming Patrol.")
                self.state = "WAIT"; return
            
            center_x = self.img_width / 2.0
            pixel_error = center_x - self.track_cx
            norm_error = pixel_error / (self.img_width / 2.0)
            angle_error = norm_error * 0.5 
            w = self.pid_heading.compute(angle_error, time.time())
            
            if self.camera_model:
                fx = self.camera_model.k[0]
                width_px = max(1.0, self.track_w)
                dist = (fx * self.BALL_REAL_DIAMETER) / width_px
            else: dist = 99.0
            
            if abs(angle_error) > 0.5: v = 0.0 
            else: v = min(self.max_lin, 0.5 * dist)
            
            if dist < self.STOP_DISTANCE:
                self.move_ttbot(0,0)
                self.get_logger().info(f"Reached {self.target_ball_color} Ball. Inspecting...")
                self.state = "INSPECTING"; self.inspect_start_time = time.time()
                # Clear Buffers
                self.pose_buffer = []
                self.ball_relative_buffer = []
            else: self.move_ttbot(v, w)
            return

        # 3. State Machine: INSPECTING (AVERAGING MODE)
        if self.state == "INSPECTING":
            self.move_ttbot(0, 0) # STOP
            
            # Wait 0.5s for settling
            if time.time() - self.inspect_start_time < 0.5: return

            # COLLECT DATA
            # 1. Average Robot Pose (AMCL)
            px = self.ttbot_pose.pose.position.x
            py = self.ttbot_pose.pose.position.y
            pyaw = get_yaw_from_quaternion(self.ttbot_pose.pose.orientation)
            self.pose_buffer.append((px, py, pyaw))

            # 2. Average Ball Relative Position (Camera Angle + LiDAR Dist)
            if self.camera_model:
                K = self.camera_model.k; fx = K[0]; cx_principal = K[2]
                angle_rad_visual = -math.atan((self.track_cx - cx_principal) / fx)
                
                # Get precise (distance, exact_angle) from LiDAR
                lidar_data = self.get_lidar_closest_point(angle_rad_visual)
                if lidar_data:
                    self.ball_relative_buffer.append(lidar_data) # (dist, angle_lidar)

            # FINISH LOGIC
            if time.time() - self.inspect_start_time > 2.0:
                final_x, final_y = None, None

                # METHOD 1: Explicit Geometry (Preferred)
                if len(self.pose_buffer) > 5 and len(self.ball_relative_buffer) > 5:
                    # Avg Robot Pose
                    avg_rx = sum(p[0] for p in self.pose_buffer) / len(self.pose_buffer)
                    avg_ry = sum(p[1] for p in self.pose_buffer) / len(self.pose_buffer)
                    # Avg Yaw (Handle Wrap Around)
                    yaws = [p[2] for p in self.pose_buffer]
                    avg_ryaw = math.atan2(sum(math.sin(y) for y in yaws), sum(math.cos(y) for y in yaws))

                    # Avg Ball Relative
                    avg_dist = sum(b[0] for b in self.ball_relative_buffer) / len(self.ball_relative_buffer)
                    # Avg Relative Angle
                    rel_angs = [b[1] for b in self.ball_relative_buffer]
                    avg_rel_ang = math.atan2(sum(math.sin(a) for a in rel_angs), sum(math.cos(a) for a in rel_angs))

                    # CALCULATE GLOBAL
                    final_x = avg_rx + avg_dist * math.cos(avg_ryaw + avg_rel_ang)
                    final_y = avg_ry + avg_dist * math.sin(avg_ryaw + avg_rel_ang)

                # METHOD 2: Fallback to Visual Estimate (Safety)
                elif self.tracking_pose is not None:
                    self.get_logger().warn("LiDAR/Pose Avg failed. Fallback to Visual Estimate.")
                    final_x, final_y = self.tracking_pose
                
                else:
                    self.get_logger().error("Lost tracking. Creating blind spot.")
                    self.no_look_zones.append((self.ttbot_pose.pose.position.x, self.ttbot_pose.pose.position.y, 1.0))
                    self.state = "WAIT"
                    return

                # Lock & Publish
                if self.is_in_no_look_zone(final_x, final_y):
                     self.state = "WAIT"
                     return

                self.permanent_obstacles.append((final_x, final_y))
                self.processed_balls.append((final_x, final_y, self.target_ball_color))
                self.no_look_zones.append((final_x, final_y, 2.0))
                
                self.get_logger().info(f"LOCKED ACCURATE: {self.target_ball_color} at ({final_x:.3f}, {final_y:.3f})")
                print(f"Object Color: {self.target_ball_color}, X: {final_x:.3f}, Y: {final_y:.3f}")
                
                # [NEW] Publish to Coordinate Topic
                p_msg = Point()
                p_msg.x = float(final_x)
                p_msg.y = float(final_y)
                p_msg.z = 0.0
                if self.target_ball_color == "Red": self.red_pub.publish(p_msg)
                elif self.target_ball_color == "Blue": self.blue_pub.publish(p_msg)
                elif self.target_ball_color == "Green": self.green_pub.publish(p_msg)

                ma = MarkerArray()
                m = Marker(); m.header.frame_id = "map"; m.id = len(self.processed_balls) + 100
                m.type = Marker.SPHERE; m.action = Marker.ADD
                m.pose.position.x = final_x; m.pose.position.y = final_y; m.pose.position.z = 0.1
                m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = 0.2; m.color.a = 1.0
                if self.target_ball_color == "Red": m.color.r = 1.0
                elif self.target_ball_color == "Green": m.color.g = 1.0
                elif self.target_ball_color == "Blue": m.color.b = 1.0
                ma.markers.append(m)
                self.found_marker_pub.publish(ma)

                self.pose_buffer = []
                self.ball_relative_buffer = []
                self.state = "WAIT"
            return

        # 4. Standard Navigation
        if self.wp_idx >= len(self.waypoints):
            self.move_ttbot(0.0, 0.0); return

        gx, gy = self.waypoints[self.wp_idx]
        rx = self.ttbot_pose.pose.position.x; ry = self.ttbot_pose.pose.position.y
        self.last_valid_yaw = get_yaw_from_quaternion(self.ttbot_pose.pose.orientation)

        if math.hypot(gx - rx, gy - ry) <= self.wp_tol:
            self.get_logger().info(f"[WP] Reached {self.wp_idx + 1}/{len(self.waypoints)}. Scanning...")
            self.state = "SCANNING"; self.scan_start_yaw = None; self._publish_markers(); self.move_ttbot(0.0, 0.0); return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.state == "WAIT":
            if (now - self.last_fail_time) < self.FAIL_COOLDOWN_SEC and (self.last_fail_time > 0): self.move_ttbot(0.0, 0.0); return
            if self.astar_try_count < self.ASTAR_MAX_TRIES:
                self.get_logger().info(f"[PLAN] A* try {self.astar_try_count + 1} (wp={self.wp_idx})")
                ok = self._plan_astar_frozen(gx, gy)
                if ok: self.state = "FOLLOW_ASTAR"; return
                else: self.astar_try_count += 1; self.total_fail_count += 1; self.last_fail_time = now; return
            if self.rrt_try_count < self.RRT_MAX_TRIES:
                self.get_logger().warn(f"[PLAN] A* failed. Trying RRT* (wp={self.wp_idx})")
                self.backup_steps_remaining = 18; self.state = "BACKUP_RRT"; return
            if self.total_fail_count >= self.SKIP_WP_AFTER_TOTAL_FAILS:
                self.get_logger().warn(f"[FAIL] Skipping waypoint {self.wp_idx}"); self.wp_idx += 1
                self.state = "WAIT"; self.astar_try_count = 0; self.rrt_try_count = 0; self.total_fail_count = 0
                self.move_ttbot(0.0, 0.0); return
            self.get_logger().warn("[FAIL] RRT* exhausted -> returning to A* strict"); self.astar_try_count = 0; self.rrt_try_count = 0
            self.last_fail_time = now; self.move_ttbot(0.0, 0.0); return

        if self.state == "FOLLOW_ASTAR":
            d_front = self.front_min_distance(-40, 40)
            if d_front < self.rrt_trigger_dist:
                est = self.estimate_front_obstacle_world(-30, 30)
                if est is not None:
                    ox, oy, d = est
                    if not self._point_is_static_occupied(ox, oy) and not self._point_is_inflated_occupied(self.inflated_strict, ox, oy):
                        self.get_logger().warn(f"[RRT-TRIGGER] Obstacle at ({ox:.2f},{oy:.2f}) -> RRT* backup")
                        self.add_dynamic_obstacle_mark(ox, oy)
                        self.backup_steps_remaining = 18; self.state = "BACKUP_RRT"; self.move_ttbot(0.0, 0.0); return

        if self.state == "BACKUP_RRT":
            if self.backup_steps_remaining > 0:
                self.backup_steps_remaining -= 1; self.move_ttbot(-0.08, 0.0); return
            ok = self._plan_rrt_to_waypoint(gx, gy)
            self.rrt_try_count += 1
            if ok: self.get_logger().info(f"[RRT*] Planned -> FOLLOW_RRT"); self.state = "FOLLOW_RRT"; return
            self.total_fail_count += 1; self.last_fail_time = now; self.state = "WAIT"; self.move_ttbot(0.0, 0.0); return

        if self.state == "FOLLOW_ASTAR":
            if not self.astar_frozen or (not self.current_astar_path.poses): self.state = "WAIT"; self.move_ttbot(0.0, 0.0); return
            v, w = self._pure_pursuit_cmd(self.current_astar_path)
            self.move_ttbot(v, w); return

        if self.state == "FOLLOW_RRT":
            if not self.rrt_has_plan or (not self.rrt_path.poses): self.state = "WAIT"; self.move_ttbot(0.0, 0.0); return
            v, w = self._follow_rrt_step()
            self.move_ttbot(v, w); return

    def move_ttbot(self, v, w):
        try:
            cmd = Twist(); cmd.linear.x = float(v); cmd.angular.z = float(w)
            self.cmd_vel_pub.publish(cmd)
        except RCLError: pass

def main(args=None):
    rclpy.init(args=args)
    node = Task3Navigator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        try:
            if rclpy.ok(): node.move_ttbot(0.0, 0.0)
        except Exception: pass
        try: node.destroy_node()
        except Exception: pass
        try:
            if rclpy.ok(): rclpy.shutdown()
        except Exception: pass

if __name__ == "__main__":
    main()