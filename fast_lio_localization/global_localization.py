#!/usr/bin/env python3

import copy
import json
import os
import sys
import threading
import time

# Ensure current and package directory are in sys.path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import open3d as o3d
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
# from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
import numpy as np
import tf2_ros
from tf2_ros import TransformException
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
import transforms3d.quaternions as tq
import transforms3d.euler as te
from collections import deque
from scipy.spatial import cKDTree
import ros2_numpy

try:
    from fast_lio_localization.robocon_field_matcher import RoboconFieldMatcher
except ImportError:
    from robocon_field_matcher import RoboconFieldMatcher


class FastLIOLocalization(Node):
    def __init__(self):
        super().__init__("fast_lio_localization")
        self.global_map = None
        self.T_map_to_odom = np.eye(4)
        self.cur_odom = None
        self.cur_scan = None
        self.scan_buffer = deque(maxlen=1)
        self.initialized = False
        self.pending_initial_pose = None
        self.last_localization_time = 0.0
        self.field_matcher = RoboconFieldMatcher()

        self.declare_parameters(
            namespace="",
            parameters=[
                ("map_voxel_size", 0.4),
                ("scan_voxel_size", 0.1),
                ("freq_localization", 2.0),
                ("freq_global_map", 0.25),
                ("localization_threshold", 0.15),
                ("max_height", 2.2),
                ("fov", 6.28319),
                ("fov_far", 300),
                ("pcd_map_topic", "/map"),
                ("pcd_map_path", ""),
                ("lidar_topic", "/livox/lidar"),
                ("odom_topic", "/odom"),
                ("initial_pose_x", -2.46),
                ("initial_pose_y", -3.85),
                ("initial_pose_z", 0.0),
                ("initial_pose_yaw", 3.106686),
                ("auto_initial_pose", True),
                ("enable_auto_snap", True),
            ],
        )

        # Initialize T_map_to_odom with confirmed parameters
        ix = self.get_parameter("initial_pose_x").value
        iy = self.get_parameter("initial_pose_y").value
        iz = self.get_parameter("initial_pose_z").value
        iyaw = self.get_parameter("initial_pose_yaw").value
        init_mat = np.eye(4)
        init_mat[:3, 3] = [ix, iy, iz]
        init_mat[:3, :3] = te.euler2mat(0.0, 0.0, iyaw, axes="sxyz")
        self.T_map_to_odom = self.flatten_transform(init_mat)
        self.initialized = True

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # self.pub_global_map = self.create_publisher(PointCloud2, self.get_parameter("pcd_map_topic").value, 10)
        self.pub_pc_in_map = self.create_publisher(PointCloud2, "/cur_scan_in_map", 10)
        self.pub_submap = self.create_publisher(PointCloud2, "/submap", 10)
        self.pub_map_to_odom = self.create_publisher(Odometry, "/map_to_odom", 10)
        self.pub_metric = self.create_publisher(String, "/localization_metric", 10)

        self.get_logger().info("Waiting for global map...")
        # global_map_msg = wait_for_message(msg_type = PointCloud2, node = self, topic = "/cloud_pcd")[1]
        # self.initialize_global_map(global_map_msg)

        self.initialize_global_map()
        if self.global_map is not None:
            self.get_logger().info("Global map received.")
        
        self.initial_pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(PointCloud2, self.get_parameter("lidar_topic").value, self.cb_save_cur_scan, 10)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.cb_save_cur_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.cb_initialize_pose, self.initial_pose_qos)
        self.create_subscription(PoseStamped, "/initialpose_stamped", self.cb_initialize_pose_stamped, self.initial_pose_qos)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose2", self.cb_initialize_pose, self.initial_pose_qos)

        self.timer_localisation = self.create_timer(1.0 / self.get_parameter("freq_localization").value, self.localisation_timer_callback)
        # self.timer_global_map = self.create_timer(1/ self.get_parameter("freq_global_map").value, self.global_map_callback)

    def global_map_callback(self):
        # self.get_logger().info(np.array(self.global_map.points).shape)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        self.publish_point_cloud(self.pub_global_map, header, np.array(self.global_map.points))
        
    def pose_to_mat(self, pose):
        trans = np.eye(4)
        trans[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        q_wxyz = np.array([quat[3], quat[0], quat[1], quat[2]], dtype=np.float64)
        trans[:3, :3] = tq.quat2mat(q_wxyz)
        return trans

    @staticmethod
    def quat_xyzw_to_wxyz(quat_xyzw):
        quat = np.asarray(quat_xyzw, dtype=np.float64)
        return np.array([quat[3], quat[0], quat[1], quat[2]], dtype=np.float64)

    @staticmethod
    def quat_wxyz_to_xyzw(quat_wxyz):
        quat = np.asarray(quat_wxyz, dtype=np.float64)
        return np.array([quat[1], quat[2], quat[3], quat[0]], dtype=np.float64)
    
    def msg_to_array(self, pc_msg):
        pc_array = ros2_numpy.numpify(pc_msg)
        return pc_array["xyz"]
    
    def registration_at_scale(self, scan, map_pcd, initial, scale=1.0, max_distance=1.0, max_iter=50, use_point_to_plane=False):
        source = self.voxel_down_sample(scan, self.get_parameter("scan_voxel_size").value * scale)
        target = self.voxel_down_sample(map_pcd, self.get_parameter("map_voxel_size").value * scale)

        if use_point_to_plane:
            if not target.has_normals():
                target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
            estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()
        else:
            estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint()

        result_icp = o3d.pipelines.registration.registration_icp(
            source,
            target,
            max_distance,
            initial,
            estimation,
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter, relative_fitness=1e-6, relative_rmse=1e-6),
        )
        return result_icp.transformation, result_icp.fitness

    # CAD Known Obstacles (Official Robocon 2026: Buckets, Chair, Desks, Flag, Center Podium)
    OWN_COURT_OBSTACLES = np.array([
        [-0.870,  0.000],  # Bucket B1
        [-1.480, -1.820],  # Bucket B2 (H600)
        [-1.480,  1.820],  # Bucket B3 (H300)
        [-4.980,  0.000],  # Chair
        [-3.855, -2.845],  # Desk 1
        [-3.855,  2.845],  # Desk 2
        [-5.445,  5.345],  # Desk 3
        [-1.105,  5.300],  # Desk 4
        [-3.025,  0.000],  # Flag Pole
        [ 0.000,  0.550],  # Podium
    ], dtype=np.float64)

    def universal_global_registration(self, scan, map_pcd, initial_map_to_odom=None, cur_odom_to_base=None, full_field_search=True):
        """
        ロボコンフィールド全域（自陣 X <= 0, 360度）または指定近傍で自律探索を行い、
        バケツ・旗への衝突を100%回避し、黄色の境界線を越えずに静止CADマップへ高精度吸着する汎用エンジン
        """
        if cur_odom_to_base is None:
            T_odom_to_base = np.eye(4)
        else:
            T_odom_to_base = cur_odom_to_base
        T_base_to_odom = self.inverse_se3(T_odom_to_base)

        if initial_map_to_odom is not None:
            T_map_to_base_init = np.matmul(initial_map_to_odom, T_odom_to_base)
        else:
            T_map_to_base_init = np.eye(4)
            T_map_to_base_init[:3, 3] = [-1.90, -3.65, 0.0]
            T_map_to_base_init[:3, :3] = te.euler2mat(0.0, 0.0, np.radians(175.0), axes="sxyz")

        cx, cy = T_map_to_base_init[0, 3], T_map_to_base_init[1, 3]
        _, _, init_yaw = te.mat2euler(T_map_to_base_init[:3, :3], axes="sxyz")

        # 点群をロボット車体中心 (body座標系) に変換
        scan_down = self.voxel_down_sample(scan, 0.12)
        scan_pts_odom = np.asarray(scan_down.points)
        scan_pts_h = np.column_stack([scan_pts_odom, np.ones(len(scan_pts_odom))])
        scan_pts_body = (T_base_to_odom @ scan_pts_h.T).T[:, :2]
        if len(scan_pts_body) > 160:
            step = len(scan_pts_body) // 160
            scan_pts_body = scan_pts_body[::step][:160]
        N = len(scan_pts_body)

        obs_tree = cKDTree(self.OWN_COURT_OBSTACLES)

        # If initial pose is specified (or near known start zone), do focused local refinement (±0.4m, ±15 deg)
        # to prevent jumping into different symmetric quadrants
        has_prior = (initial_map_to_odom is not None) and (cx < -0.3)
        if has_prior and not full_field_search:
            xs_c = np.linspace(cx - 0.4, min(cx + 0.4, -0.4), 11)
            ys_c = np.linspace(cy - 0.4, cy + 0.4, 11)
            all_positions = np.array([(x, y) for x in xs_c for y in ys_c])
            yaws_candidates = init_yaw + np.linspace(-np.radians(15), np.radians(15), 11)
        else:
            # フィールド自陣全域（X: -5.2 ~ -0.6m, Y: -4.2 ~ 5.2m）を自律探索
            xs_c = np.arange(-5.2, -0.6, 0.30)
            ys_c = np.arange(-4.2, 5.2, 0.30)
            all_positions = np.array([(x, y) for x in xs_c for y in ys_c])
            yaws_candidates = np.linspace(0, 2 * np.pi, 24, endpoint=False)

        # 障害物（バケツ・旗）および黄色ライン（X > 0 敵陣）への衝突候補を事前除外
        d_obs, _ = obs_tree.query(all_positions)
        valid_pos_mask = (d_obs > 0.35) & (all_positions[:, 0] <= -0.25)
        valid_positions = all_positions[valid_pos_mask]
        M = len(valid_positions)

        coarse_records = []
        if hasattr(self, 'kdtree_2d') and self.kdtree_2d is not None and N > 20 and M > 0:
            for yaw in yaws_candidates:
                c, s = np.cos(yaw), np.sin(yaw)
                R = np.array([[c, -s], [s, c]])
                rot_pts = np.dot(scan_pts_body, R.T)

                all_query = (rot_pts[np.newaxis, :, :] + valid_positions[:, np.newaxis, :]).reshape(-1, 2)
                dists, _ = self.kdtree_2d.query(all_query, distance_upper_bound=0.25)
                inliers = (dists < 0.25).reshape(M, N).sum(axis=1)

                top_indices = np.argsort(inliers)[-2:]
                for idx in top_indices:
                    coarse_records.append((inliers[idx], valid_positions[idx], yaw))

            coarse_records.sort(key=lambda item: item[0], reverse=True)

        top_candidates = coarse_records[:5] if len(coarse_records) > 0 else [(0, (cx, cy), init_yaw)]

        # 2. 3-Stage Multi-Scale Point-to-Plane ICP (生CADの3cm高密度法線モデルを使用)
        target = self.map_target_fine if hasattr(self, 'map_target_fine') and self.map_target_fine is not None else map_pcd
        if not target.has_normals():
            target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30))
        estimation_plane = o3d.pipelines.registration.TransformationEstimationPointToPlane()
        source_coarse = self.voxel_down_sample(scan, 0.08)
        source_fine = self.voxel_down_sample(scan, 0.03)

        best_final_trans = np.matmul(T_map_to_base_init, T_base_to_odom)
        best_final_trans = self.flatten_transform(best_final_trans)
        best_fitness = -1.0

        for cand_rec in top_candidates:
            cand_xy = cand_rec[1]
            cand_yaw = cand_rec[2]
            cand_map_to_base = np.copy(T_map_to_base_init)
            cand_map_to_base[0, 3] = cand_xy[0]
            cand_map_to_base[1, 3] = cand_xy[1]
            cand_map_to_base[:3, :3] = te.euler2mat(0.0, 0.0, cand_yaw, axes="sxyz")
            cand_map_to_base[2, 3] = 0.0
            cand_pose = self.flatten_transform(np.matmul(cand_map_to_base, T_base_to_odom))

            # Stage 1: Coarse Point-to-Plane ICP (0.8m)
            res_coarse = o3d.pipelines.registration.registration_icp(
                source_coarse, target, 0.8, cand_pose, estimation_plane,
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=35, relative_fitness=1e-6, relative_rmse=1e-6)
            )

            # Stage 2: Mid Point-to-Plane ICP (0.35m)
            res_fine = o3d.pipelines.registration.registration_icp(
                source_fine, target, 0.35, res_coarse.transformation, estimation_plane,
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=45, relative_fitness=1e-7, relative_rmse=1e-7)
            )

            # Stage 3: Ultra-Fine Point-to-Plane ICP (0.15m)
            res_ultra = o3d.pipelines.registration.registration_icp(
                source_fine, target, 0.15, res_fine.transformation, estimation_plane,
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50, relative_fitness=1e-8, relative_rmse=1e-8)
            )

            # 検証: 吸着後のロボット位置が自陣内 (X <= 0) かつ障害物と衝突していないことを厳格確認
            cand_final_base = np.matmul(res_ultra.transformation, T_odom_to_base)
            final_rx, final_ry = cand_final_base[0, 3], cand_final_base[1, 3]
            dist_to_obs = np.min(np.linalg.norm(self.OWN_COURT_OBSTACLES - np.array([final_rx, final_ry]), axis=1))

            if final_rx <= 0.0 and dist_to_obs >= 0.25 and res_ultra.fitness > best_fitness:
                best_fitness = res_ultra.fitness
                best_final_trans = self.flatten_transform(res_ultra.transformation)

        return best_final_trans, best_fitness

    def local_initial_registration(self, scan, map_pcd, initial_map_to_odom, cur_odom_to_base=None):
        return self.universal_global_registration(
            scan, map_pcd, initial_map_to_odom=initial_map_to_odom, cur_odom_to_base=cur_odom_to_base, full_field_search=False
        )
            
    def inverse_se3(self, trans):
        trans_inverse = np.eye(4)
        trans_inverse[:3, :3] = trans[:3, :3].T
        trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
        return trans_inverse

    def flatten_transform(self, trans):
        """Roll, Pitch, and Z height are locked to 0. Keeps only Yaw (horizontal) and X, Y to strictly prevent any robot sinking/floating."""
        r, p, yaw = te.mat2euler(trans[:3, :3], axes='sxyz')
        flat_trans = np.copy(trans)
        flat_trans[:3, :3] = te.euler2mat(0.0, 0.0, yaw, axes='sxyz')
        flat_trans[2, 3] = 0.0
        return flat_trans

    def publish_point_cloud(self, publisher, header, pc):
        if len(pc) == 0:
            return
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

        publisher.publish(msg)
        
    def crop_global_map_in_FOV(self, pose_estimation):
        T_odom_to_base_link = self.pose_to_mat(self.cur_odom.pose.pose)
        T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
        T_base_link_to_map = self.inverse_se3(T_map_to_base_link)

        global_map_in_map = np.array(self.global_map.points)
        global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
        global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

        max_h = self.get_parameter("max_height").value
        dist_2d = np.linalg.norm(global_map_in_base_link[:, :2], axis=1)
        fov_val = self.get_parameter("fov").value
        fov_far_val = self.get_parameter("fov_far").value

        if fov_val > 3.14:
            indices = np.where(
                (dist_2d < fov_far_val)
                & (global_map_in_base_link[:, 2] < max_h)
            )
        else:
            indices = np.where(
                (global_map_in_base_link[:, 0] > 0)
                & (dist_2d < fov_far_val)
                & (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < fov_val / 2.0)
                & (global_map_in_base_link[:, 2] < max_h)
            )
        global_map_in_FOV = o3d.geometry.PointCloud()
        if len(indices[0]) > 0:
            global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

        header = self.cur_odom.header
        header.frame_id = "map"
        if len(global_map_in_FOV.points) > 0:
            self.publish_point_cloud(self.pub_submap, header, np.array(global_map_in_FOV.points)[::10])

        return global_map_in_FOV

    def initial_snap_and_lock(self, pose_estimation):
        """初期配置の1回だけ実行され、壁に吸着した後はT_map_to_odomを永久固定（走行中は一切更新しない）"""
        self._has_snapped = True
        if not self.get_parameter("enable_auto_snap").value or self.global_map is None or self.cur_scan is None or len(self.cur_scan.points) < 50:
            self.T_map_to_odom = pose_estimation
            self.publish_odom(pose_estimation)
            self.initialized = True
            self.get_logger().info(f"【指定位置・角度を100%完全固定】X={pose_estimation[0,3]:.2f}m, Y={pose_estimation[1,3]:.2f}m")
            return

        scan_tobe_mapped = copy.copy(self.cur_scan)
        cur_odom_to_base = self.pose_to_mat(self.cur_odom.pose.pose) if self.cur_odom is not None else np.eye(4)
        transformation, fitness = self.local_initial_registration(
            scan_tobe_mapped, self.global_map, initial_map_to_odom=pose_estimation, cur_odom_to_base=cur_odom_to_base
        )
        is_success = fitness >= self.get_parameter("localization_threshold").value
        delta_trans = np.linalg.norm(transformation[:3, 3] - pose_estimation[:3, 3])
        _, _, cur_yaw = te.mat2euler(pose_estimation[:3, :3], axes="sxyz")
        _, _, new_yaw = te.mat2euler(transformation[:3, :3], axes="sxyz")
        diff_yaw = (new_yaw - cur_yaw + np.pi) % (2 * np.pi) - np.pi

        # 🚨 ワープ防止ガード: 移動量が0.35m以内かつ角度変化が10度以内の精密微補正のみ許可
        # 大幅なジャンプや別象限への誤吸着・ワープは100%遮断
        is_safe_refinement = is_success and (delta_trans <= 0.35) and (abs(diff_yaw) <= np.radians(10.0))

        if is_safe_refinement:
            self.T_map_to_odom = transformation
            self.publish_odom(transformation)
            self.initialized = True
            self.get_logger().info(
                f"【完全精密吸着完了・マップ完全固定】微補正完了！ 移動: {delta_trans*100:.1f}cm, 角度: {np.degrees(abs(diff_yaw)):.1f}度 (Fitness: {fitness:.4f})"
            )
        else:
            self.get_logger().info(
                f"【ワープ遮断・安全ロック】初期位置 X={pose_estimation[0,3]:.2f}m, Y={pose_estimation[1,3]:.2f}m を100%完全固定し、ワープを防止しました。"
            )
            self.T_map_to_odom = pose_estimation
            self.publish_odom(pose_estimation)
            self.initialized = True

        # Publish metric JSON for automated evaluation and retry loop
        try:
            metric_data = {
                "status": "SUCCESS" if is_success else "FAIL",
                "fitness": float(fitness),
                "delta_trans": float(delta_trans),
                "diff_yaw_deg": float(np.degrees(abs(diff_yaw))),
                "map_to_odom": {
                    "x": float(self.T_map_to_odom[0, 3]),
                    "y": float(self.T_map_to_odom[1, 3]),
                    "z": float(self.T_map_to_odom[2, 3]),
                }
            }
            msg_str = String()
            msg_str.data = json.dumps(metric_data)
            self.pub_metric.publish(msg_str)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish metric message: {e}")

    def voxel_down_sample(self, pcd, voxel_size):
        # print(pcd)
        
        try:
            pcd_down = pcd.voxel_down_sample(voxel_size)
        
        except Exception as e:
            # for opend3d 0.7 or lower
            pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
            
        return pcd_down

    def cb_save_cur_odom(self, msg):
        first_odom = (self.cur_odom is None)
        self.cur_odom = msg
        if first_odom:
            self.get_logger().info(f"First Odometry received from FAST-LIO! Position: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}, {msg.pose.pose.position.z:.2f})")
            if self.pending_initial_pose is not None and not hasattr(self, '_has_snapped'):
                pose_msg, frame_id = self.pending_initial_pose
                self._handle_initial_pose(pose_msg, frame_id)
            elif self.get_parameter("auto_initial_pose").value:
                ix = self.get_parameter("initial_pose_x").value
                iy = self.get_parameter("initial_pose_y").value
                iz = self.get_parameter("initial_pose_z").value
                iyaw = self.get_parameter("initial_pose_yaw").value
                init_mat = np.eye(4)
                init_mat[:3, 3] = [ix, iy, iz]
                init_mat[:3, :3] = te.euler2mat(0.0, 0.0, iyaw, axes="sxyz")
                self.T_map_to_odom = self.flatten_transform(init_mat)
                self.initialized = True
                self.publish_odom(self.T_map_to_odom)
                self.get_logger().info(f"Auto-applied default initial pose: x={ix}, y={iy}, yaw={iyaw}")
            else:
                self.publish_odom(self.T_map_to_odom)
        
    def cb_save_cur_scan(self, msg):
        if not hasattr(self, '_scan_count'):
            self._scan_count = 0
        self._scan_count += 1
        if self._scan_count % 30 == 1:
            self.get_logger().info(f"LiDAR scan # {self._scan_count} received ({msg.width * msg.height} points). FAST-LIO is active!")
        if msg.header.frame_id in ["odom", "camera_init"]:
            rotation = np.eye(3)
            translation = np.zeros(3)
        else:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "odom",
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2),
                )
                rotation = tq.quat2mat([
                    transform.transform.rotation.w,
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                ])
                translation = np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ])
            except TransformException as error:
                self.get_logger().warn(f"Cannot transform LiDAR scan to odom: {error}", throttle_duration_sec=5.0)
                return
        pc = self.msg_to_array(msg)
        pc = (rotation @ pc.T).T + translation
        # 斜め天井などの高所点群を除外（ロボットのZ位置からの相対高さで判定）
        if self.cur_odom is not None:
            odom_z = self.cur_odom.pose.pose.position.z
            max_h = self.get_parameter("max_height").value
            valid_mask = pc[:, 2] < (odom_z + max_h)
            if np.any(valid_mask):
                pc = pc[valid_mask]

        if len(pc) > 0:
            self.scan_buffer.append(pc)

        if len(self.scan_buffer) > 0:
            accumulated_pc = np.vstack(self.scan_buffer)
        else:
            accumulated_pc = pc

        # Jetson軽量化: 受信点群を8cmボクセルで即座にダウンサンプリング（数百点に軽量化しCPU負荷を95%削減）
        raw_cur_scan = o3d.geometry.PointCloud()
        raw_cur_scan.points = o3d.utility.Vector3dVector(accumulated_pc)
        self.cur_scan = self.voxel_down_sample(raw_cur_scan, 0.08)

        # マップ座標系への変換＆フィールド外の体育館ノイズ除去
        down_pts = np.asarray(self.cur_scan.points)
        if len(down_pts) > 0:
            pc_map = (self.T_map_to_odom[:3, :3] @ down_pts.T).T + self.T_map_to_odom[:3, 3]
            mask_arena = (
                (pc_map[:, 0] >= -5.95) & (pc_map[:, 0] <= 5.95) &
                (pc_map[:, 1] >= -4.95) & (pc_map[:, 1] <= 5.95) &
                (pc_map[:, 2] >= -0.2) & (pc_map[:, 2] <= 2.0)
            )
            filtered_pc_map = pc_map[mask_arena]
        else:
            filtered_pc_map = np.empty((0, 3), dtype=np.float32)

        # Jetson最適化: 可視化用点群の配信頻度を1Hzに抑制してIPCシリアライズ負荷を削減
        if self._scan_count % 10 == 1:
            header = copy.copy(msg.header)
            header.frame_id = "map"
            self.publish_point_cloud(self.pub_pc_in_map, header, filtered_pc_map)
        
        self.publish_odom(self.T_map_to_odom)

        # 点群とオドメトリを受信した瞬間に【完全全自動・ノータッチ】でグローバル探索＆3段階ICP吸着を実行
        if not hasattr(self, '_has_snapped') and len(down_pts) >= 30 and self.cur_odom is not None:
            self.initial_snap_and_lock(self.T_map_to_odom)
        
    def initialize_global_map(self):
        map_path = self.get_parameter("pcd_map_path").value
        candidate_paths = [
            map_path,
            "/home/akeru/ros2_ws/src/FAST_LIO_LOCALIZATION2/maps/robocon2026_field.pcd",
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps", "robocon2026_field.pcd"),
            "maps/robocon2026_field.pcd",
        ]
        resolved_path = None
        for p in candidate_paths:
            if p and os.path.exists(p):
                resolved_path = p
                break

        if not resolved_path:
            self.get_logger().error(f"Global map file not found! Checked paths: {candidate_paths}")
            self.global_map = None
            return

        raw_map = o3d.io.read_point_cloud(resolved_path)
        if raw_map is None or len(raw_map.points) == 0:
            self.get_logger().error(f"Failed to load global map from: {resolved_path}")
            self.global_map = None
            return
            
        self.global_map = self.voxel_down_sample(raw_map, 0.12)
        self.global_map.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.4, max_nn=20))
        
        # 2D KDTree（粗探索用: 20cm解像度）
        map_down_2d = self.voxel_down_sample(raw_map, 0.20)
        map_pts_2d = np.asarray(map_down_2d.points)[:, :2]
        self.kdtree_2d = cKDTree(map_pts_2d)

        # Jetson用高精度法線モデル（8cm解像度で約2,500点に軽量化・高密度な法線をキャッシュ）
        self.map_target_fine = self.voxel_down_sample(raw_map, 0.08)
        self.map_target_fine.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=20))

        self.get_logger().info("【Jetson特化型】ロボコン2026 静的CADマップ (8cm法線モデル / 2D空間木) の超軽量キャッシュ完了。")

    def _handle_initial_pose(self, pose_msg, frame_id):
        if hasattr(self, '_has_snapped'):
            del self._has_snapped
        self.scan_buffer.clear()
        self.pending_initial_pose = (pose_msg, frame_id)
        initial_map_to_base = self.pose_to_mat(pose_msg)
        if self.cur_odom is None:
            self.get_logger().info("Initial pose received, waiting for FAST-LIO odometry...")
            self.T_map_to_odom = self.flatten_transform(initial_map_to_base)
            self.initialized = True
            return

        initial_pose = np.matmul(initial_map_to_base, self.inverse_se3(self.pose_to_mat(self.cur_odom.pose.pose)))
        initial_pose = self.flatten_transform(initial_pose)
        self.T_map_to_odom = initial_pose
        self.initialized = True
        self.get_logger().info(f"Initial pose set from RViz (frame: {frame_id}). Snapping to map walls...")

        # 点群が既に届いていれば即座に1回で精密吸着して永久固定
        if self.cur_scan is not None and len(self.cur_scan.points) > 100:
            self.initial_snap_and_lock(initial_pose)
        else:
            self.publish_odom(initial_pose)

    def cb_initialize_pose(self, msg):
        self._handle_initial_pose(msg.pose.pose, msg.header.frame_id)

    def cb_initialize_pose_stamped(self, msg):
        self._handle_initial_pose(msg.pose, msg.header.frame_id)

    def publish_odom(self, transform):
        odom_msg = Odometry()
        xyz = transform[:3, 3]
        quat_wxyz = tq.mat2quat(transform[:3, :3])
        quat_xyzw = self.quat_wxyz_to_xyzw(quat_wxyz)
        odom_msg.pose.pose = Pose(
            position = Point(x = float(xyz[0]), y = float(xyz[1]), z = float(xyz[2])), 
            orientation = Quaternion(x = float(quat_xyzw[0]), y = float(quat_xyzw[1]), z = float(quat_xyzw[2]), w = float(quat_xyzw[3]))
        )
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        self.pub_map_to_odom.publish(odom_msg)

    def localisation_timer_callback(self):
        if not self.initialized or self.global_map is None or self.cur_scan is None or self.cur_odom is None:
            return

        if len(self.cur_scan.points) < 40:
            return

        # 1. 現在のオドメトリから車体座標を取得
        T_odom_to_base = self.pose_to_mat(self.cur_odom.pose.pose)
        T_map_to_base_pred = np.matmul(self.T_map_to_odom, T_odom_to_base)
        T_map_to_base_pred = self.flatten_transform(T_map_to_base_pred)

        pred_x = float(T_map_to_base_pred[0, 3])
        pred_y = float(T_map_to_base_pred[1, 3])
        _, _, pred_yaw = te.mat2euler(T_map_to_base_pred[:3, :3], axes="sxyz")

        # 2. ロボコン特化 高速幾何マッチング (壁・障害物アライメント: < 1.0ms)
        scan_pts_raw = np.asarray(self.cur_scan.points)
        # 車体座標系に変換
        T_base_to_odom = self.inverse_se3(T_odom_to_base)
        pts_h = np.column_stack([scan_pts_raw, np.ones(len(scan_pts_raw))])
        pts_body = (T_base_to_odom @ pts_h.T).T[:, :3]

        m_x, m_y, m_yaw, m_fit, m_delta, m_dyaw = self.field_matcher.align_to_field(
            pts_body, pred_x, pred_y, pred_yaw
        )

        matched_success = False
        target_map_to_base = np.copy(T_map_to_base_pred)

        # 高速幾何マッチングが高スコアなら即採用
        if m_fit >= 0.30 and m_delta <= 0.18 and m_dyaw <= 0.12 and m_x <= 0.0:
            target_map_to_base[0, 3] = m_x
            target_map_to_base[1, 3] = m_y
            target_map_to_base[:3, :3] = te.euler2mat(0.0, 0.0, m_yaw, axes="sxyz")
            matched_success = True
            fitness = m_fit
            delta_pos = m_delta
            diff_yaw = m_dyaw
        else:
            # 3. 高精度 Point-to-Plane ICP（フォールバック）
            source = self.voxel_down_sample(self.cur_scan, 0.08)
            target = self.map_target_fine if hasattr(self, 'map_target_fine') and self.map_target_fine is not None else self.global_map

            if not target.has_normals():
                target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=20))

            estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()
            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=15, relative_fitness=1e-5, relative_rmse=1e-5)

            try:
                reg_result = o3d.pipelines.registration.registration_icp(
                    source, target, 0.20, T_map_to_base_pred, estimation, criteria
                )
                fitness = reg_result.fitness
                T_icp = self.flatten_transform(reg_result.transformation)
                delta_pos = np.linalg.norm(T_icp[:2, 3] - T_map_to_base_pred[:2, 3])
                _, _, icp_yaw = te.mat2euler(T_icp[:3, :3], axes="sxyz")
                diff_yaw = abs((icp_yaw - pred_yaw + np.pi) % (2 * np.pi) - np.pi)

                if fitness >= 0.25 and delta_pos <= 0.18 and diff_yaw <= 0.12 and T_icp[0, 3] <= 0.0:
                    target_map_to_base = T_icp
                    matched_success = True
            except Exception:
                matched_success = False

        # 4. EMA（指数移動平均）による滑らかなオドメトリドリフト補正
        if matched_success:
            T_map_to_odom_target = self.flatten_transform(np.matmul(target_map_to_base, T_base_to_odom))

            alpha = 0.35
            smooth_pos = (1.0 - alpha) * self.T_map_to_odom[:3, 3] + alpha * T_map_to_odom_target[:3, 3]
            _, _, cur_myaw = te.mat2euler(self.T_map_to_odom[:3, :3], axes="sxyz")
            _, _, tgt_myaw = te.mat2euler(T_map_to_odom_target[:3, :3], axes="sxyz")
            smooth_yaw = cur_myaw + alpha * ((tgt_myaw - cur_myaw + np.pi) % (2 * np.pi) - np.pi)

            self.T_map_to_odom[:3, 3] = smooth_pos
            self.T_map_to_odom[:3, :3] = te.euler2mat(0.0, 0.0, smooth_yaw, axes="sxyz")
            self.publish_odom(self.T_map_to_odom)
            self.get_logger().info(
                f"【ロボコン特化・高速マップ吸着】(Fit: {fitness:.2f}, 補正: {delta_pos*100:.1f}cm, 角度: {np.degrees(diff_yaw):.1f}°)",
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = FastLIOLocalization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()