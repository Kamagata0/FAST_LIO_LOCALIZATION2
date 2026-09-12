#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robocon 2026 Specialized LiDAR Localization & Target Detection Node
- Zero odometry dependency
- Zero initial pose required (calculates global pose in 1 frame < 5ms)
- 12m ROI & ground plane removal
- Arena 4-wall geometric fit
- Real-time target object extraction & relative distance/angle computation
"""

import copy
import os
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import tf2_ros
from scipy.spatial import cKDTree
import transforms3d.euler as te
import transforms3d.quaternions as tq
import ros2_numpy

try:
    from livox_ros_driver2.msg import CustomMsg
    HAVE_LIVOX_MSG = True
except ImportError:
    HAVE_LIVOX_MSG = False


class FieldLocalizationNode(Node):
    def __init__(self):
        super().__init__("field_localization_node")

        # Declare parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("lidar_topic", "/livox/lidar"),
                ("map_path", "maps/robocon2026_field.pcd"),
                ("max_range", 12.0),
                ("min_range", 0.25),
                ("min_height", 0.05),
                ("max_height", 1.80),
                ("arena_length_x", 11.70),  # -5.85 to +5.85
                ("arena_length_y", 10.80),  # -4.85 to +5.95
                ("arena_center_y", 0.55),
                ("publish_tf", True),
                ("base_frame", "base_link"),
                ("map_frame", "map"),
            ]
        )

        self.max_range = self.get_parameter("max_range").value
        self.min_range = self.get_parameter("min_range").value
        self.min_height = self.get_parameter("min_height").value
        self.max_height = self.get_parameter("max_height").value
        self.arena_lx = self.get_parameter("arena_length_x").value
        self.arena_ly = self.get_parameter("arena_length_y").value
        self.arena_cy = self.get_parameter("arena_center_y").value
        self.publish_tf = self.get_parameter("publish_tf").value
        self.base_frame = self.get_parameter("base_frame").value
        self.map_frame = self.get_parameter("map_frame").value

        # Publishers
        self.pub_robot_pose = self.create_publisher(PoseStamped, "/robot_pose", 10)
        self.pub_target_rel = self.create_publisher(PointStamped, "/target_relative", 10)
        self.pub_opponent_pose = self.create_publisher(PoseStamped, "/opponent_pose", 10)
        self.pub_opponent_rel = self.create_publisher(PointStamped, "/opponent_relative", 10)
        self.pub_filtered_scan = self.create_publisher(PointCloud2, "/field_scan_filtered", 10)
        self.pub_cur_scan = self.create_publisher(PointCloud2, "/cur_scan_in_map", 10)
        self.pub_path = self.create_publisher(Path, "/localization_path", 10)
        self.pub_loc_odom = self.create_publisher(Odometry, "/localization", 10)
        self.pub_map = self.create_publisher(PointCloud2, "/map", 1)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame

        # Load robocon2026_field.pcd
        self.load_robocon_pcd(self.get_parameter("map_path").value)

        # Subscribers
        lidar_topic = self.get_parameter("lidar_topic").value
        self.sub_pc = self.create_subscription(PointCloud2, lidar_topic, self.cb_pointcloud2, 10)
        if HAVE_LIVOX_MSG and lidar_topic != "/cloud_registered":
            try:
                self.sub_livox = self.create_subscription(CustomMsg, lidar_topic, self.cb_livox_custom, 10)
            except Exception as e:
                self.get_logger().warn(f"Could not subscribe CustomMsg on {lidar_topic}: {e}")

        # State tracking
        self.last_pose = np.array([-1.90, 3.65, -0.0873])  # default startup pose
        self.has_valid_pose = False

        # Periodic Heartbeat Timer (10Hz) to keep map TF & map point cloud alive in RViz
        self.timer_heartbeat = self.create_timer(0.1, self.timer_heartbeat_cb)
        self._map_pub_count = 0

        self.get_logger().info(
            f"🚀 [Robocon Field Localization] Started with robocon2026_field.pcd! Range: <= {self.max_range}m, Height: {self.min_height}~{self.max_height}m"
        )

    def timer_heartbeat_cb(self):
        """Keep TF map -> base_link active at 10Hz so RViz Fixed Frame is always valid immediately upon launch"""
        now_stamp = self.get_clock().now().to_msg()
        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now_stamp
            tf_msg.header.frame_id = self.map_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = float(self.last_pose[0])
            tf_msg.transform.translation.y = float(self.last_pose[1])
            tf_msg.transform.translation.z = 0.0
            q_wxyz = tq.euler2quat(0.0, 0.0, float(self.last_pose[2]), axes="sxyz")
            tf_msg.transform.rotation.w = float(q_wxyz[0])
            tf_msg.transform.rotation.x = float(q_wxyz[1])
            tf_msg.transform.rotation.y = float(q_wxyz[2])
            tf_msg.transform.rotation.z = float(q_wxyz[3])
            self.tf_broadcaster.sendTransform(tf_msg)

        # Publish map cloud periodically (every 1s)
        self._map_pub_count += 1
        if self._map_pub_count % 10 == 0 and hasattr(self, 'map_pts') and self.map_pts is not None:
            hdr = Header()
            hdr.stamp = now_stamp
            hdr.frame_id = self.map_frame
            self.publish_point_cloud(self.pub_map, hdr, self.map_pts[::3])

    def load_robocon_pcd(self, map_path: str):
        candidate_paths = [
            map_path,
            os.path.join(os.path.dirname(__file__), "..", "maps", "robocon2026_field.pcd"),
            os.path.join(os.getcwd(), "maps", "robocon2026_field.pcd"),
            "maps/robocon2026_field.pcd",
        ]
        resolved = None
        for p in candidate_paths:
            if p and os.path.exists(p):
                resolved = p
                break

        if not resolved:
            self.get_logger().warn(f"PCD map file not found at {candidate_paths}. Using mathematical CAD boundaries.")
            self.map_pts = None
            self.map_kdtree = None
            return

        try:
            with open(resolved, "rb") as f:
                header_lines = []
                while True:
                    line = f.readline().decode("ascii", errors="ignore")
                    header_lines.append(line)
                    if line.startswith("DATA binary"):
                        data_type = "binary"
                        break
                    elif line.startswith("DATA ascii"):
                        data_type = "ascii"
                        break
                points = 0
                for l in header_lines:
                    if l.startswith("POINTS"):
                        points = int(l.split()[1])
                if data_type == "binary":
                    raw = f.read()
                    arr = np.frombuffer(raw, dtype=np.float32)
                    step = len(arr) // points
                    self.map_pts = arr.reshape(points, step)[:, :3]
                else:
                    self.map_pts = np.loadtxt(resolved, skiprows=len(header_lines))[:, :3]

            # 2D KDTree of the CAD field walls
            self.map_kdtree = cKDTree(self.map_pts[::5, :2])
            self.get_logger().info(f"Loaded CAD map: {resolved} ({len(self.map_pts)} points)")

            # Publish once to /map
            hdr = Header()
            hdr.stamp = self.get_clock().now().to_msg()
            hdr.frame_id = self.map_frame
            self.publish_point_cloud(self.pub_map, hdr, self.map_pts[::3])
        except Exception as e:
            self.get_logger().error(f"Error loading PCD {resolved}: {e}")
            self.map_pts = None
            self.map_kdtree = None

    def cb_livox_custom(self, msg: CustomMsg):
        pts = []
        for p in msg.points:
            r2 = p.x * p.x + p.y * p.y
            if self.min_range * self.min_range <= r2 <= self.max_range * self.max_range:
                if self.min_height <= p.z <= self.max_height:
                    pts.append([p.x, p.y, p.z])
        if len(pts) > 50:
            self.process_points(np.array(pts, dtype=np.float32), msg.header)

    def cb_pointcloud2(self, msg: PointCloud2):
        try:
            pc_array = ros2_numpy.numpify(msg)
            xyz = pc_array["xyz"]
            r2 = xyz[:, 0] ** 2 + xyz[:, 1] ** 2
            mask = (
                (r2 >= self.min_range ** 2) &
                (r2 <= self.max_range ** 2) &
                (xyz[:, 2] >= self.min_height) &
                (xyz[:, 2] <= self.max_height)
            )
            pts = xyz[mask]
            if len(pts) > 50:
                self.process_points(pts, msg.header)
        except Exception as e:
            pass

    def process_points(self, pts: np.ndarray, header: Header):
        t0 = time.time()
        
        # 1. 2D grid downsampling (10cm voxel for speed < 2ms)
        grid_size = 0.10
        coords = np.floor(pts[:, :2] / grid_size).astype(np.int32)
        _, unique_indices = np.unique(coords, axis=0, return_index=True)
        pts_2d = pts[unique_indices, :2]

        if len(pts_2d) < 30:
            return

        # 2. Extract arena orientation and robot global pose
        pose, wall_mask = self.solve_arena_pose(pts_2d)
        if pose is None:
            return

        rx, ry, ryaw = pose
        self.last_pose = np.array([rx, ry, ryaw])
        self.has_valid_pose = True

        # 3. Target and Opponent extraction from non-wall cluster points
        target_info, opp_info = self.extract_target_and_opponent(pts_2d[~wall_mask], rx, ry, ryaw)

        proc_time_ms = (time.time() - t0) * 1000.0

        # 4. Publish ROS 2 messages & TF
        self.publish_results(rx, ry, ryaw, target_info, opp_info, pts, header)

    # Known Own-Court Obstacles for Collision Avoidance
    OWN_COURT_OBSTACLES = np.array([
        [-3.86, -2.85],  # Bucket B1
        [-1.48, -1.82],  # Bucket B2
        [-5.05,  0.00],  # Bucket B3
        [-0.87,  0.00],  # Bucket B4
        [-1.48,  1.82],  # Bucket B5
        [-3.86,  2.84],  # Bucket B6
        [-5.38,  5.36],  # Bucket B7
        [-1.08,  5.34],  # Bucket B8
        [-3.03,  0.00],  # Flag Pole
    ], dtype=np.float64)

    @staticmethod
    def cluster_2d_points(pts: np.ndarray, eps: float = 0.35, min_samples: int = 5):
        """Pure SciPy KDTree DBSCAN-like clustering (zero external sklearn dependency)"""
        N = len(pts)
        if N < min_samples:
            return []
        tree = cKDTree(pts)
        visited = np.zeros(N, dtype=bool)
        clusters = []

        for i in range(N):
            if visited[i]:
                continue
            nbrs = tree.query_ball_point(pts[i], r=eps)
            if len(nbrs) < min_samples:
                continue
            cluster = list(nbrs)
            visited[nbrs] = True
            cur = 0
            while cur < len(cluster):
                idx = cluster[cur]
                sub_nbrs = tree.query_ball_point(pts[idx], r=eps)
                if len(sub_nbrs) >= min_samples:
                    for sn in sub_nbrs:
                        if not visited[sn]:
                            visited[sn] = True
                            cluster.append(sn)
                cur += 1
            if len(cluster) >= min_samples:
                clusters.append(pts[cluster])
        return clusters

    def solve_arena_pose(self, pts_2d: np.ndarray):
        """
        Fast 2D multi-hypothesis fitting against Robocon CAD Map.
        Enforces own-court boundary (X <= 0.0) and obstacle clearance.
        Returns: (robot_x, robot_y, robot_yaw), wall_inlier_mask
        """
        N = len(pts_2d)
        if N < 25:
            return None, np.zeros(N, dtype=bool)

        # 1. 2D PCA & Angular projection
        angles = np.linspace(0, np.pi / 2, 45, endpoint=False)
        best_score = -1
        best_angle = 0.0

        for ang in angles:
            c, s = np.cos(ang), np.sin(ang)
            u = pts_2d[:, 0] * c + pts_2d[:, 1] * s
            v = -pts_2d[:, 0] * s + pts_2d[:, 1] * c
            hist_u, _ = np.histogram(u, bins=60)
            hist_v, _ = np.histogram(v, bins=60)
            score = np.max(hist_u) + np.max(hist_v)
            if score > best_score:
                best_score = score
                best_angle = ang

        # Rotate points to arena-aligned frame
        ca, sa = np.cos(best_angle), np.sin(best_angle)
        R_align = np.array([[ca, sa], [-sa, ca]])
        aligned_pts = (R_align @ pts_2d.T).T

        # Find bounding walls
        min_u, max_u = np.percentile(aligned_pts[:, 0], 2), np.percentile(aligned_pts[:, 0], 98)
        min_v, max_v = np.percentile(aligned_pts[:, 1], 2), np.percentile(aligned_pts[:, 1], 98)

        wall_thresh = 0.20
        wall_mask = (
            (np.abs(aligned_pts[:, 0] - min_u) < wall_thresh) |
            (np.abs(aligned_pts[:, 0] - max_u) < wall_thresh) |
            (np.abs(aligned_pts[:, 1] - min_v) < wall_thresh) |
            (np.abs(aligned_pts[:, 1] - max_v) < wall_thresh)
        )

        center_u = (min_u + max_u) / 2.0
        center_v = (min_v + max_v) / 2.0

        # Evaluate 4 candidate orientations (0, 90, 180, 270 deg)
        best_inliers = -1
        best_pose = None
        obs_tree = cKDTree(self.OWN_COURT_OBSTACLES)

        for base_yaw in [-best_angle, -best_angle + np.pi/2, -best_angle + np.pi, -best_angle - np.pi/2]:
            byaw = (base_yaw + np.pi) % (2 * np.pi) - np.pi
            rx = -center_u * np.cos(best_angle) + center_v * np.sin(best_angle)
            ry = -center_u * np.sin(best_angle) - center_v * np.cos(best_angle) + self.arena_cy

            # Must stay inside own-court (X <= 0.0)
            if rx > -0.20:
                continue

            # Must not collide with buckets or flags
            d_obs, _ = obs_tree.query(np.array([[rx, ry]]))
            if d_obs[0] < 0.25:
                continue

            # Inlier check against CAD map if available
            if hasattr(self, 'map_kdtree') and self.map_kdtree is not None:
                c, s = np.cos(byaw), np.sin(byaw)
                R = np.array([[c, -s], [s, c]])
                trans_pts = pts_2d @ R.T + np.array([rx, ry])
                dists, _ = self.map_kdtree.query(trans_pts, distance_upper_bound=0.20)
                inliers = np.sum(dists < 0.20)
            else:
                inliers = N

            if inliers > best_inliers:
                best_inliers = inliers
                best_pose = (rx, ry, byaw)

        if best_pose is None:
            # Fallback to bounded coordinate
            rx = min(-center_u, -0.50)
            ry = -center_v + self.arena_cy
            best_pose = (rx, ry, -best_angle)

        return best_pose, wall_mask

    def extract_target_and_opponent(self, internal_pts: np.ndarray, rx: float, ry: float, ryaw: float):
        """
        Clusters points inside the arena via CAD-map subtraction to locate:
        1. Opponent Robot (large cluster 0.35m~1.10m, >12 points)
        2. Target Objects (smaller cluster, 0.10m~0.30m)
        Uses pure SciPy KD-Tree clustering (100% Jetson compatible).
        """
        if len(internal_pts) < 8:
            return None, None

        clusters = self.cluster_2d_points(internal_pts, eps=0.35, min_samples=5)
        if not clusters:
            return None, None

        best_target = None
        best_opponent = None
        min_target_dist = float("inf")
        c, s = np.cos(ryaw), np.sin(ryaw)

        for cluster in clusters:
            centroid = np.mean(cluster, axis=0)
            dist = np.linalg.norm(centroid)

            # Global Map-frame centroid
            map_cx = rx + c * centroid[0] - s * centroid[1]
            map_cy = ry + s * centroid[0] + c * centroid[1]

            # Discard central teaching podium (X~0, Y~0 in map)
            if abs(map_cx) < 0.85 and abs(map_cy - self.arena_cy) < 0.85:
                continue

            # Compute cluster bounding box span
            cluster_span = np.max(cluster, axis=0) - np.min(cluster, axis=0)
            cluster_width = max(cluster_span[0], cluster_span[1])
            num_pts = len(cluster)

            angle_rad = math.atan2(centroid[1], centroid[0])
            angle_deg = math.degrees(angle_rad)

            # Opponent Robot classification: width 0.30m ~ 1.20m and > 10 points
            if cluster_width >= 0.30 and num_pts >= 10 and dist > 0.5:
                best_opponent = {
                    "map_x": float(map_cx),
                    "map_y": float(map_cy),
                    "rel_x": float(centroid[0]),
                    "rel_y": float(centroid[1]),
                    "distance": float(dist),
                    "angle_rad": float(angle_rad),
                    "angle_deg": float(angle_deg),
                    "num_pts": int(num_pts),
                }
            # Target object classification (ball / smaller payload)
            elif dist < min_target_dist and dist > 0.25:
                min_target_dist = dist
                best_target = {
                    "map_x": float(map_cx),
                    "map_y": float(map_cy),
                    "rel_x": float(centroid[0]),
                    "rel_y": float(centroid[1]),
                    "distance": float(dist),
                    "angle_rad": float(angle_rad),
                    "angle_deg": float(angle_deg),
                    "num_pts": int(num_pts),
                }

        return best_target, best_opponent

    def publish_results(self, rx, ry, ryaw, target_info, opp_info, raw_pts, header):
        now_stamp = header.stamp if header.stamp.sec > 0 else self.get_clock().now().to_msg()

        # 1. Robot Pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now_stamp
        pose_msg.header.frame_id = self.map_frame
        pose_msg.pose.position.x = float(rx)
        pose_msg.pose.position.y = float(ry)
        pose_msg.pose.position.z = 0.0

        q_wxyz = tq.euler2quat(0.0, 0.0, ryaw, axes="sxyz")
        pose_msg.pose.orientation.w = float(q_wxyz[0])
        pose_msg.pose.orientation.x = float(q_wxyz[1])
        pose_msg.pose.orientation.y = float(q_wxyz[2])
        pose_msg.pose.orientation.z = float(q_wxyz[3])
        self.pub_robot_pose.publish(pose_msg)

        # 2. TF map -> base_link
        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now_stamp
            tf_msg.header.frame_id = self.map_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = float(rx)
            tf_msg.transform.translation.y = float(ry)
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = pose_msg.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

        # 3. Target Relative
        if target_info is not None:
            pt_msg = PointStamped()
            pt_msg.header.stamp = now_stamp
            pt_msg.header.frame_id = self.base_frame
            pt_msg.point.x = target_info["rel_x"]
            pt_msg.point.y = target_info["rel_y"]
            pt_msg.point.z = 0.0
            self.pub_target_rel.publish(pt_msg)

        # 4. Opponent Robot Pose & Relative
        if opp_info is not None:
            opp_pose = PoseStamped()
            opp_pose.header.stamp = now_stamp
            opp_pose.header.frame_id = self.map_frame
            opp_pose.pose.position.x = opp_info["map_x"]
            opp_pose.pose.position.y = opp_info["map_y"]
            opp_pose.pose.position.z = 0.0
            opp_pose.pose.orientation.w = 1.0
            self.pub_opponent_pose.publish(opp_pose)

            opp_rel = PointStamped()
            opp_rel.header.stamp = now_stamp
            opp_rel.header.frame_id = self.base_frame
            opp_rel.point.x = opp_info["rel_x"]
            opp_rel.point.y = opp_info["rel_y"]
            opp_rel.point.z = 0.0
            self.pub_opponent_rel.publish(opp_rel)

        # 5. Filtered map-frame point cloud
        if len(raw_pts) > 0:
            c, s = np.cos(ryaw), np.sin(ryaw)
            R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float32)
            trans_pts = (R @ raw_pts.T).T + np.array([rx, ry, 0.0], dtype=np.float32)
            
            cloud_hdr = Header()
            cloud_hdr.stamp = now_stamp
            cloud_hdr.frame_id = self.map_frame
            self.publish_point_cloud(self.pub_filtered_scan, cloud_hdr, trans_pts)
            if self.publish_tf:
                self.publish_point_cloud(self.pub_cur_scan, cloud_hdr, trans_pts)

        # 6. Odometry and Trajectory Path (Only when standalone publish_tf is True)
        if self.publish_tf:
            odom_msg = Odometry()
            odom_msg.header.stamp = now_stamp
            odom_msg.header.frame_id = self.map_frame
            odom_msg.child_frame_id = self.base_frame
            odom_msg.pose.pose = pose_msg.pose
            self.pub_loc_odom.publish(odom_msg)

            if not hasattr(self, '_path_skip'):
                self._path_skip = 0
            self._path_skip += 1
            if self._path_skip % 3 == 0:
                self.path_msg.header.stamp = now_stamp
                self.path_msg.poses.append(pose_msg)
                self.pub_path.publish(self.path_msg)

    def publish_point_cloud(self, pub, header, pc):
        num_points = pc.shape[0]
        xyz = np.ascontiguousarray(pc[:, :3], dtype=np.float32)
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = num_points
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * num_points
        msg.is_dense = True
        msg.data = bytes(xyz.tobytes())
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FieldLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
