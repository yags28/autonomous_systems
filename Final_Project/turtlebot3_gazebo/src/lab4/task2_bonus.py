#!/usr/bin/env python3
import math
import heapq
import random
import threading
import time
import os
import yaml
from collections import deque

import numpy as np
if not hasattr(np, 'float'):
    np.float = float
import cv2 
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory

# Only import Path (removed OccupancyGrid)
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, ColorRGBA
from visualization_msgs.msg import Marker 
from tf_transformations import euler_from_quaternion
import tf2_ros
try:
    import scipy.interpolate as si
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

# -------------------- Parameters -------------------- #
# Robot Dimensions
ROBOT_RADIUS_SAFE   = 0.35  # Standard safety margin
ROBOT_RADIUS_NARROW = 0.16  # Reduced margin for corridors

# Navigation
LOOKAHEAD_DISTANCE = 0.45   
GOAL_TOLERANCE     = 0.15
MAX_SPEED          = 0.22
ROTATE_SPEED       = 1.0
MAX_ANG_VEL        = 1.0
ROTATE_THRESHOLD   = 0.6    

# RRT*
RRT_TRIGGER_DIST   = 0.75   
RRT_CLEARANCE      = 0.55   

# -------------------- Utility functions -------------------- #

def get_yaw_from_quaternion(q):
    quat = [q.x, q.y, q.z, q.w]
    _, _, yaw = euler_from_quaternion(quat)
    return yaw

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def heuristic(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])

# -------------------- CLOTHOID / BEZIER SMOOTHING -------------------- #

def clothoid_smoothing(path_poses):
    if len(path_poses) < 3: return path_poses
    new_poses = []
    points = [(p.pose.position.x, p.pose.position.y) for p in path_poses]
    new_poses.append(path_poses[0])

    for i in range(1, len(points) - 1):
        p_prev = points[i-1]; p_curr = points[i]; p_next = points[i+1]
        v1 = np.array([p_curr[0] - p_prev[0], p_curr[1] - p_prev[1]])
        v2 = np.array([p_next[0] - p_curr[0], p_next[1] - p_curr[1]])
        len1 = np.linalg.norm(v1); len2 = np.linalg.norm(v2)
        
        if len1 < 0.1 or len2 < 0.1: new_poses.append(path_poses[i]); continue
        u1 = v1 / len1; u2 = v2 / len2
        angle = math.acos(np.clip(np.dot(u1, u2), -1.0, 1.0))
        if abs(angle) < 0.1: new_poses.append(path_poses[i]); continue
            
        d = min(len1, len2) * 0.4 
        P0 = (p_curr[0] - u1[0]*d, p_curr[1] - u1[1]*d) 
        P1 = p_curr                                     
        P2 = (p_curr[0] + u2[0]*d, p_curr[1] + u2[1]*d) 
        
        steps = 15
        for t_step in range(steps + 1):
            t = t_step / float(steps)
            bx = (1-t)**2 * P0[0] + 2*(1-t)*t * P1[0] + t**2 * P2[0]
            by = (1-t)**2 * P0[1] + 2*(1-t)*t * P1[1] + t**2 * P2[1]
            ps = PoseStamped(); ps.header = path_poses[0].header
            ps.pose.position.x = bx; ps.pose.position.y = by; ps.pose.orientation.w = 1.0
            new_poses.append(ps)
            
    new_poses.append(path_poses[-1])
    return new_poses

# -------------------- Graph classes (A*) -------------------- #

class GraphNode:
    def __init__(self, name):
        self.name = name; self.neighbors = []; self.costs = []
    def add_edges(self, nums, ws):
        self.neighbors.extend(nums); self.costs.extend(ws)

class GridGraph:
    def __init__(self): self.nodes = {}

class AStar:
    def __init__(self, graph: GridGraph):
        self.graph = graph; self.g = {}; self.h = {}; self.prev = {}

    def _reset(self):
        self.g = {n: float('inf') for n in self.graph.nodes}
        self.h = {n: 0.0 for n in self.graph.nodes}
        self.prev = {n: None for n in self.graph.nodes}

    def _calc_heuristic(self, goal):
        gr, gc = map(int, goal.split(','))
        for name in self.graph.nodes:
            r, c = map(int, name.split(','))
            self.h[name] = math.hypot(gr - r, gc - c)

    def plan(self, start_name, goal):
        if start_name not in self.graph.nodes or goal not in self.graph.nodes: return
        self._reset(); self._calc_heuristic(goal)
        self.g[start_name] = 0.0
        open_heap = [(self.h[start_name], start_name)]
        closed = set()

        while open_heap:
            _, cur = heapq.heappop(open_heap)
            if cur in closed: continue
            closed.add(cur)
            if cur == goal: break
            node = self.graph.nodes[cur]
            for neighbor, w in zip(node.neighbors, node.costs):
                nn = neighbor.name; newg = self.g[cur] + float(w)
                if newg < self.g[nn]:
                    self.g[nn] = newg; self.prev[nn] = cur
                    heapq.heappush(open_heap, (newg + self.h[nn], nn))

    def reconstruct(self, start_name, goal):
        if self.g.get(goal, float('inf')) == float('inf'): return []
        path = []; cur = goal
        while cur is not None:
            path.append(cur); cur = self.prev[cur]
        path.reverse()
        return path

# -------------------- RRT* Structures -------------------- #

class RRTStarNode:
    def __init__(self, x, y, parent=None, cost=0.0):
        self.x = x; self.y = y; self.parent = parent; self.cost = cost

# -------------------- Navigation Node -------------------- #

class Navigation(Node):
    def __init__(self, node_name='task2_algorithm'):
        super().__init__(node_name)
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.get_logger().info("[INIT] Task2 Hybrid (Local Map + Marker Viz) started")

        self.max_ang = MAX_ANG_VEL
        self.rotate_threshold = ROTATE_THRESHOLD
        self.smoothing_alpha = 0.3 
        self.v_last = 0.0; self.w_last = 0.0

        self.goal_pose = None
        self.ttbot_pose = PoseStamped()
        
        self.path = Path()
        self.planner_mode = "ASTAR"
        self.mode = "FOLLOW"
        self.arrived = True
        
        self.rrt_has_plan = False
        self.rrt_path = Path()
        self.current_global_path = Path()
        self.local_rrt_goal = None
        self.backup_steps_remaining = 0
        self.retry_count = 0 
        
        self.map_loaded = False
        self.res = 0.05; self.ox = 0.0; self.oy = 0.0; self.W = 0; self.H = 0
        self.raw_occ = None         
        self.inflated_safe = None; self.inflated_narrow = None
        self.graph_safe = None; self.graph_narrow = None
        self.astar_safe = None; self.astar_narrow = None
        self.current_inflated = None

        self.scan_msg = None

        self.rrt_max_iters = 3000
        self.rrt_step_size = 0.15 
        self.rrt_neighbor_radius = 1.0 
        self.rrt_collision_step = 0.05

        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.__goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.__ttbot_pose_cbk, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.rrt_pub = self.create_publisher(Path, 'local_rrt_plan', 10)
        
        # CHANGED: Replaced costmap_pub (OccupancyGrid) with map_debug_pub (Marker)
        self.map_debug_pub = self.create_publisher(Marker, '/map_debug_markers', 10)
        
        self.rrt_debug_pub = self.create_publisher(Marker, '/rrt_debug', 10)
        self.lookahead_pub = self.create_publisher(Marker, '/lookahead_marker', 10)
        self.closest_pub = self.create_publisher(Marker, '/closest_point_marker', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.load_map_from_file()

        threading.Thread(target=self.run_loop, daemon=True).start()

    def load_map_from_file(self):
        try:
            pkg_path = get_package_share_directory('turtlebot3_gazebo')
            yaml_path = os.path.join(pkg_path, 'maps', 'map.yaml')
            
            with open(yaml_path, 'r') as f:
                map_yaml = yaml.safe_load(f)
            
            self.res = map_yaml['resolution']
            self.ox = map_yaml['origin'][0]
            self.oy = map_yaml['origin'][1]
            pgm_filename = map_yaml['image']
            
            pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_filename)
            img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
            
            if img is None:
                self.get_logger().error(f"Failed to load PGM file at {pgm_path}")
                return

            img = cv2.flip(img, 0)
            self.H, self.W = img.shape
            
            self.raw_occ = np.zeros_like(img, dtype=np.int8)
            self.raw_occ[img <= 10] = 100 
            self.raw_occ[img >= 250] = 0 
            self.raw_occ[(img > 10) & (img < 250)] = 100

            self.get_logger().info(f"Map Loaded from file: {self.W}x{self.H}, Res: {self.res}")
            self.get_logger().info("Building Graphs...")
            
            self.inflated_safe = self._inflate(self.raw_occ, ROBOT_RADIUS_SAFE)
            self.graph_safe = self._build_graph_from_inflated(self.inflated_safe)
            self.astar_safe = AStar(self.graph_safe)

            self.inflated_narrow = self._inflate(self.raw_occ, ROBOT_RADIUS_NARROW)
            self.graph_narrow = self._build_graph_from_inflated(self.inflated_narrow)
            self.astar_narrow = AStar(self.graph_narrow)

            self.map_loaded = True
            self.current_inflated = self.inflated_safe 
            
            self.publish_costmap(self.inflated_safe)
            self.get_logger().info("Map Graphs Ready.")

        except Exception as e:
            self.get_logger().error(f"Failed to load map file: {e}")

    # ---------------- Callbacks ---------------- #

    def __goal_pose_cbk(self, data):
        self.goal_pose = data; self.arrived = False
        self.planner_mode = "ASTAR"; self.mode = "FOLLOW"
        self.get_logger().info(f"New goal received.")

    def __ttbot_pose_cbk(self, data):
        ps = PoseStamped(); ps.header = data.header; ps.pose = data.pose.pose
        self.ttbot_pose = ps

    def scan_callback(self, msg): self.scan_msg = msg

    def _inflate(self, occ_grid, r_m):
        H, W = occ_grid.shape
        base = np.where(occ_grid >= 50, 100, 0).astype(np.uint8)
        r_cells = int(math.ceil(r_m / self.res))
        yy, xx = np.ogrid[-r_cells:r_cells+1, -r_cells:r_cells+1]
        disk = (xx**2 + yy**2) <= (r_cells*r_cells)
        dil = base.copy()
        for dy in range(-r_cells, r_cells+1):
            for dx in range(-r_cells, r_cells+1):
                if not disk[dy+r_cells, dx+r_cells]: continue
                src_y0=max(0,-dy); src_y1=H-max(0,dy); src_x0=max(0,-dx); src_x1=W-max(0,dx)
                dst_y0=max(0,dy); dst_y1=H-max(0,-dy); dst_x0=max(0,dx); dst_x1=W-max(0,-dx)
                dil[dst_y0:dst_y1, dst_x0:dst_x1] = np.maximum(dil[dst_y0:dst_y1, dst_x0:dst_x1], base[src_y0:src_y1, src_x0:src_x1])
        return dil

    def _build_graph_from_inflated(self, inflated):
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
                ns, ws = [], []
                for dr, dc, w in nbrs:
                    nr, nc = r+dr, c+dc
                    if 0<=nr<H and 0<=nc<W and inflated[nr,nc]==0:
                        ns.append(g.nodes[f"{nr},{nc}"]); ws.append(w)
                if ns: parent.add_edges(ns, ws)
        return g

    def publish_costmap(self, occ):
        # Visualize using MARKERS
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.scale.x = 0.05; m.scale.y = 0.05 
        m.color.r = 0.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0 
        
        y_idxs, x_idxs = np.where(occ == 100)
        step = 2 
        for i in range(0, len(y_idxs), step):
            r = y_idxs[i]; c = x_idxs[i]
            wx = self.ox + (c + 0.5) * self.res; wy = self.oy + (r + 0.5) * self.res
            p = Point(); p.x = float(wx); p.y = float(wy); p.z = 0.0
            m.points.append(p)
        self.map_debug_pub.publish(m)

    def world_to_grid(self, x, y):
        c=int((x-self.ox)/self.res); r=int((y-self.oy)/self.res)
        return max(0, min(self.W-1, c)), max(0, min(self.H-1, r))
    def grid_to_world(self, c, r): return self.ox+(c+0.5)*self.res, self.oy+(r+0.5)*self.res

    def _nearest_free(self, c, r, inflated_map):
        if inflated_map[r, c] == 0: return c, r
        q = deque([(c, r)]); vis = set([(c,r)])
        while q:
            cc, rr = q.popleft()
            if inflated_map[rr, cc] == 0: return cc, rr
            for dc, dr in [(0,1),(0,-1),(1,0),(-1,0),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nc, nr = cc+dc, rr+dr
                if 0<=nc<self.W and 0<=nr<self.H and (nc,nr) not in vis:
                    vis.add((nc,nr)); q.append((nc,nr))
        return c, r

    def plan_adaptive_astar(self, start_pose, end_pose):
        path = Path(); path.header.frame_id = 'map'
        sx, sy = start_pose.pose.position.x, start_pose.pose.position.y
        gx, gy = end_pose.pose.position.x, end_pose.pose.position.y
        sc, sr = self.world_to_grid(sx, sy); gc, gr = self.world_to_grid(gx, gy)
        
        sc_s, sr_s = self._nearest_free(sc, sr, self.inflated_safe)
        gc_s, gr_s = self._nearest_free(gc, gr, self.inflated_safe)
        self.astar_safe.plan(f"{sr_s},{sc_s}", f"{gr_s},{gc_s}")
        pts = self.astar_safe.reconstruct(f"{sr_s},{sc_s}", f"{gr_s},{gc_s}")
        
        if not pts:
            self.current_inflated = self.inflated_narrow 
            self.publish_costmap(self.inflated_narrow)
            sc_n, sr_n = self._nearest_free(sc, sr, self.inflated_narrow)
            gc_n, gr_n = self._nearest_free(gc, gr, self.inflated_narrow)
            self.astar_narrow.plan(f"{sr_n},{sc_n}", f"{gr_n},{gc_n}")
            pts = self.astar_narrow.reconstruct(f"{sr_n},{sc_n}", f"{gr_n},{gc_n}")
        else:
            self.current_inflated = self.inflated_safe
            self.publish_costmap(self.inflated_safe)

        if not pts: return path
        for pt in pts:
            r, c = map(int, pt.split(','))
            wx, wy = self.grid_to_world(c, r)
            ps = PoseStamped(); ps.header.frame_id='map'
            ps.pose.position.x=wx; ps.pose.position.y=wy; ps.pose.orientation.w=1.0
            path.poses.append(ps)
        self.path_pub.publish(path)
        return path

    def analyze_scan_smart(self):
        if not self.scan_msg or self.ttbot_pose.header.stamp.sec == 0: return 99.0, 0, 0
        ranges = np.array(self.scan_msg.ranges)
        min_ang = self.scan_msg.angle_min; inc = self.scan_msg.angle_increment
        idx_min = int((-0.35 - min_ang)/inc); idx_max = int((0.35 - min_ang)/inc)
        idx_min = max(0, idx_min); idx_max = min(len(ranges)-1, idx_max)
        front = ranges[idx_min:idx_max]
        if len(front)==0: return 99.0, 0, 0
        valid = np.where(np.isfinite(front))[0]
        if len(valid)==0: return 99.0, 0, 0
        local_idx = valid[np.argmin(front[valid])]
        dist = front[local_idx]
        angle = min_ang + (idx_min + local_idx) * inc
        
        rx = self.ttbot_pose.pose.position.x; ry = self.ttbot_pose.pose.position.y
        yaw = get_yaw_from_quaternion(self.ttbot_pose.pose.orientation)
        wx = rx + dist * math.cos(yaw + angle)
        wy = ry + dist * math.sin(yaw + angle)
        
        wc, wr = self.world_to_grid(wx, wy)
        is_wall = False
        for dr in [-2,-1,0,1,2]:
            for dc in [-2,-1,0,1,2]:
                if 0<=wr+dr<self.H and 0<=wc+dc<self.W:
                    if self.raw_occ[wr+dr, wc+dc] == 100: is_wall = True
        if is_wall: return 99.0, 0, 0
        return dist, wx, wy

    def mark_obstacle_on_maps(self, obs_x, obs_y):
        mc, mr = self.world_to_grid(obs_x, obs_y)
        r = 10
        for dr in range(-r, r+1):
            for dc in range(-r, r+1):
                if 0<=mr+dr<self.H and 0<=mc+dc<self.W:
                    if dr*dr + dc*dc <= r*r:
                        self.inflated_safe[mr+dr, mc+dc] = 100
                        self.inflated_narrow[mr+dr, mc+dc] = 100
        self.publish_costmap(self.current_inflated)

    def get_side_goal(self, obs_x, obs_y, rx, ry, gx, gy):
        yaw_to_obs = math.atan2(obs_y - ry, obs_x - rx)
        dist_side = 1.0
        candidates = []
        
        offset = math.radians(60) 
        
        for side_angle in [offset, -offset]:
            check_ang = yaw_to_obs + side_angle
            best_d = None
            for d in np.arange(0.5, 1.5, 0.1):
                cx = obs_x + d * math.cos(check_ang)
                cy = obs_y + d * math.sin(check_ang)
                c_raw = int((cx - self.ox) / self.res)
                r_raw = int((cy - self.oy) / self.res)
                if 0 <= c_raw < self.W and 0 <= r_raw < self.H:
                    if self.inflated_narrow[r_raw, c_raw] == 0:
                        best_d = d; break 
            if best_d:
                cx = obs_x + best_d * math.cos(check_ang)
                cy = obs_y + best_d * math.sin(check_ang)
                dist_to_goal = math.hypot(cx-gx, cy-gy)
                candidates.append((dist_to_goal, (cx, cy)))

        m = Marker(); m.header.frame_id = "map"; m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD; m.scale.x=0.15; m.scale.y=0.15; m.scale.z=0.15
        m.color.r=1.0; m.color.a=1.0; m.id = 999; m.ns="side_candidates"
        for c in candidates:
            m.points.append(Point(x=c[1][0], y=c[1][1], z=0.2))
        self.rrt_debug_pub.publish(m)

        if not candidates:
            self.get_logger().warn("No side candidates found!")
            return None
        
        candidates.sort(key=lambda x: x[0])
        return candidates[0][1]

    def is_free_world(self, x, y):
        c, r = self.world_to_grid(x, y)
        if not (0<=r<self.H and 0<=c<self.W): return False
        return self.current_inflated[r, c] == 0

    def is_free_world_narrow(self, x, y):
        c, r = self.world_to_grid(x, y)
        if not (0<=r<self.H and 0<=c<self.W): return False
        return self.inflated_narrow[r, c] == 0

    def _find_valid_point(self, x, y):
        c, r = self.world_to_grid(x, y)
        if self.is_free_world(x, y): return x, y
        q = deque([(c, r)]); vis = set([(c,r)])
        while q:
            cc, rr = q.popleft()
            wx, wy = self.grid_to_world(cc, rr)
            if self.is_free_world(wx, wy): return wx, wy
            for dc, dr in [(0,1),(0,-1),(1,0),(-1,0),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nc, nr = cc+dc, rr+dr
                if 0<=nc<self.W and 0<=nr<self.H and (nc,nr) not in vis:
                    vis.add((nc,nr)); q.append((nc,nr))
            if len(vis) > 500: break
        return None, None

    def collision_free_segment(self, x1, y1, x2, y2):
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist < 1e-6: return self.is_free_world(x1, y1)
        steps = int(dist / self.rrt_collision_step) + 1
        for i in range(steps + 1):
            t = i / max(steps, 1)
            if not self.is_free_world(x1 + t*(x2-x1), y1 + t*(y2-y1)): return False
        return True

    def shortcut_path(self, poses):
        if len(poses) < 3: return poses
        new_poses = [poses[0]]
        curr_idx = 0
        while curr_idx < len(poses) - 1:
            last_valid = curr_idx + 1
            for i in range(len(poses) - 1, curr_idx + 1, -1):
                p1 = new_poses[-1].pose.position; p2 = poses[i].pose.position
                if self.collision_free_segment(p1.x, p1.y, p2.x, p2.y):
                    last_valid = i; break
            new_poses.append(poses[last_valid]); curr_idx = last_valid
        
        final_poses = [new_poses[0]]
        curr_idx = 0
        while curr_idx < len(new_poses) - 1:
            last_valid = curr_idx + 1
            for i in range(len(new_poses) - 1, curr_idx + 1, -1):
                p1 = final_poses[-1].pose.position; p2 = new_poses[i].pose.position
                if self.collision_free_segment(p1.x, p1.y, p2.x, p2.y):
                    last_valid = i; break
            final_poses.append(new_poses[last_valid]); curr_idx = last_valid
        return final_poses

    def plan_rrt_star(self, start, end):
        path = Path(); path.header.frame_id='map'
        sx, sy = start.pose.position.x, start.pose.position.y
        gx, gy = end.pose.position.x, end.pose.position.y
        
        sx, sy = self._find_valid_point(sx, sy)
        gx, gy = self._find_valid_point(gx, gy)
        if sx is None or gx is None: return path
        
        m = Marker(); m.header.frame_id="map"; m.type=Marker.SPHERE_LIST; m.action=Marker.ADD
        m.scale.x=0.2; m.scale.y=0.2; m.scale.z=0.2; m.color.r=1.0; m.color.a=1.0
        m.points=[Point(x=sx, y=sy), Point(x=gx, y=gy)]
        self.rrt_debug_pub.publish(m)

        nodes = [RRTStarNode(sx, sy)]; goal_idx = None
        for _ in range(self.rrt_max_iters):
            if random.random() < 0.15: rx, ry = gx, gy
            else: rx = random.uniform(self.ox, self.ox+self.W*self.res); ry = random.uniform(self.oy, self.oy+self.H*self.res)
            
            near_idx = min(range(len(nodes)), key=lambda i: math.hypot(nodes[i].x-rx, nodes[i].y-ry))
            near = nodes[near_idx]
            theta = math.atan2(ry-near.y, rx-near.x)
            nx = near.x + self.rrt_step_size * math.cos(theta)
            ny = near.y + self.rrt_step_size * math.sin(theta)
            if not self.is_free_world(nx, ny): continue
            
            new_node = RRTStarNode(nx, ny, parent=near_idx, cost=near.cost+0.3)
            nodes.append(new_node)
            if math.hypot(nx-gx, ny-gy) < 0.3:
                goal_idx = len(nodes)
                nodes.append(RRTStarNode(gx, gy, parent=len(nodes)-1, cost=new_node.cost))
                break
        
        if goal_idx is None: return path
        idx = goal_idx; raw_poses = []
        while idx is not None:
            n = nodes[idx]; ps = PoseStamped(); ps.pose.position.x=n.x; ps.pose.position.y=n.y
            raw_poses.append(ps); idx = n.parent
        raw_poses.reverse()
        
        pruned_poses = self.shortcut_path(raw_poses)
        path.poses = clothoid_smoothing(pruned_poses) 
        self.rrt_pub.publish(path)
        return path

    def get_path_idx(self, path, vehicle_pose, lookahead):
        if not path.poses: return 0
        rx = vehicle_pose.pose.position.x; ry = vehicle_pose.pose.position.y
        closest_dist = 9999.0; closest_idx = 0
        for i, ps in enumerate(path.poses):
            d = math.hypot(ps.pose.position.x - rx, ps.pose.position.y - ry)
            if d < closest_dist: closest_dist = d; closest_idx = i
        
        m = Marker(); m.header.frame_id = "map"; m.type = Marker.SPHERE
        m.action = Marker.ADD; m.scale.x=0.2; m.scale.y=0.2; m.scale.z=0.2; m.color.b=1.0; m.color.a=1.0; m.id = 0; m.ns="closest"
        m.pose = path.poses[closest_idx].pose
        self.closest_pub.publish(m)

        target_idx = closest_idx
        for i in range(closest_idx, len(path.poses)):
            ps = path.poses[i]
            d = math.hypot(ps.pose.position.x - rx, ps.pose.position.y - ry)
            if d > lookahead: target_idx = i; break
        if target_idx == closest_idx and len(path.poses)>0: target_idx = len(path.poses)-1
        
        m.color.b=0.0; m.color.g=1.0; m.id=1; m.ns="lookahead"
        m.pose = path.poses[target_idx].pose
        self.lookahead_pub.publish(m)
        return target_idx

    def path_follower(self, vehicle_pose, current_goal_pose):
        rx = vehicle_pose.pose.position.x; ry = vehicle_pose.pose.position.y
        gx = current_goal_pose.pose.position.x; gy = current_goal_pose.pose.position.y
        ryaw = get_yaw_from_quaternion(vehicle_pose.pose.orientation)
        tgt_yaw = math.atan2(gy - ry, gx - rx)
        yaw_err = normalize_angle(tgt_yaw - ryaw)
        
        if abs(yaw_err) > self.rotate_threshold: v=0.0; w=max(-1.0, min(1.0, 1.5*yaw_err))
        else: v=MAX_SPEED*math.cos(yaw_err); w=max(-1.0, min(1.0, 1.2*yaw_err))
        return v, w

    def rrt_path_follower(self, vehicle_pose, current_goal_pose):
        rx = vehicle_pose.pose.position.x; ry = vehicle_pose.pose.position.y
        gx = current_goal_pose.pose.position.x; gy = current_goal_pose.pose.position.y
        ryaw = get_yaw_from_quaternion(vehicle_pose.pose.orientation)
        tgt_yaw = math.atan2(gy - ry, gx - rx)
        yaw_err = normalize_angle(tgt_yaw - ryaw)
        
        if abs(yaw_err) > 0.4: v=0.0; w=0.8*(yaw_err/abs(yaw_err))
        else: v=(MAX_SPEED*0.7)*math.cos(yaw_err); w=2.5*yaw_err
        w = max(-self.max_ang, min(self.max_ang, w))
        if v < 0: v=0
        return v, w

    def move_ttbot(self, v, w):
        cmd = Twist(); cmd.linear.x=float(v); cmd.angular.z=float(w)
        self.cmd_vel_pub.publish(cmd)

    def update_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time(), Duration(seconds=1.0))
            ps = PoseStamped(); ps.header=t.header
            ps.pose.position.x=t.transform.translation.x; ps.pose.position.y=t.transform.translation.y
            ps.pose.orientation=t.transform.rotation
            self.ttbot_pose = ps
            return True
        except: return False

    def run_loop(self):
        while rclpy.ok():
            time.sleep(0.1)
            if not self.update_pose(): continue
            if not self.map_loaded: continue
            
            if self.goal_pose is None or self.arrived:
                self.move_ttbot(0,0); continue

            gx = self.goal_pose.pose.position.x; gy = self.goal_pose.pose.position.y
            rx = self.ttbot_pose.pose.position.x; ry = self.ttbot_pose.pose.position.y
            
            if math.hypot(gx-rx, gy-ry) < GOAL_TOLERANCE:
                self.arrived = True; self.move_ttbot(0,0); self.get_logger().info("Goal Reached!")
                continue

            scan_dist, obs_x, obs_y = self.analyze_scan_smart()
            
            if self.planner_mode == "ASTAR":
                path = self.plan_adaptive_astar(self.ttbot_pose, self.goal_pose)
                self.current_global_path = path; self.path = path
                
                if scan_dist < RRT_TRIGGER_DIST:
                    self.get_logger().warn("New Obstacle! RRT*")
                    self.planner_mode = "RRTSTAR"
                    self.rrt_has_plan = False
                    self.mode = "BACKUP"
                    self.backup_steps_remaining = 20
                    self.retry_count = 0
                    
                    self.mark_obstacle_on_maps(obs_x, obs_y)
                    
                    local_pose = self.get_side_goal(obs_x, obs_y, rx, ry, gx, gy)
                    if local_pose:
                        self.local_rrt_goal = PoseStamped()
                        self.local_rrt_goal.header.frame_id = 'map'
                        self.local_rrt_goal.pose.position.x = local_pose[0]
                        self.local_rrt_goal.pose.position.y = local_pose[1]
                        self.get_logger().info(f"Lateral Goal Selected: {local_pose}")
                    else:
                        self.local_rrt_goal = self.goal_pose

            elif self.planner_mode == "RRTSTAR":
                if self.mode == "BACKUP":
                    if self.backup_steps_remaining > 0:
                        self.move_ttbot(-0.15, 0.0); self.backup_steps_remaining -= 1; continue
                    else: self.mode = "FOLLOW"
                
                if not self.rrt_has_plan:
                    path = self.plan_rrt_star(self.ttbot_pose, self.local_rrt_goal)
                    if path.poses: 
                        self.rrt_path = path; self.rrt_has_plan = True; self.retry_count = 0
                    else: 
                        self.retry_count += 1
                        if self.retry_count > 3:
                             self.get_logger().warn("RRT* Failed multiple times. Retrying Global.")
                             self.planner_mode = "ASTAR"
                        else:
                             self.get_logger().warn("RRT* Failed. Backing up more...")
                             self.mode = "BACKUP"; self.backup_steps_remaining = 15
                
                self.path = self.rrt_path
                lgx = self.local_rrt_goal.pose.position.x; lgy = self.local_rrt_goal.pose.position.y
                if math.hypot(lgx-rx, lgy-ry) < 0.2: # Reduced tolerance
                    self.get_logger().info("Lateral Pass Complete. Back to A*.")
                    self.planner_mode = "ASTAR"

            if self.path.poses:
                lookahead = 0.25 if self.planner_mode == "RRTSTAR" else LOOKAHEAD_DISTANCE
                idx = self.get_path_idx(self.path, self.ttbot_pose, lookahead)
                if self.planner_mode == "RRTSTAR":
                    v, w = self.rrt_path_follower(self.ttbot_pose, self.path.poses[idx])
                else:
                    v, w = self.path_follower(self.ttbot_pose, self.path.poses[idx])
                self.move_ttbot(v, w)
            else: self.move_ttbot(0, 0)

def main(args=None):
    rclpy.init(args=args)
    node = Navigation()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()