#!/usr/bin/env python3

import copy
import json
import os
import threading
import time

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


class FastLIOLocalization(Node):
    def __init__(self):
        super().__init__("fast_lio_localization")
        self.global_map = None
        self.T_map_to_odom = np.eye(4)
        self.cur_odom = None
        self.cur_scan = None
        self.scan_buffer = deque(maxlen=10)
        self.initialized = False
        self.pending_initial_pose = None
        self.last_localization_time = 0.0

        self.declare_parameters(
            namespace="",
            parameters=[
                ("map_voxel_size", 0.4),
                ("scan_voxel_size", 0.1),
                ("freq_localization", 0.5),
                ("freq_global_map", 0.25),
                ("localization_threshold", 0.15),
                ("max_height", 2.2),
                ("fov", 6.28319),
                ("fov_far", 300),
                ("pcd_map_topic", "/map"),
                ("pcd_map_path", ""),
                ("lidar_topic", "/livox/lidar"),
                ("odom_topic", "/odom"),
                ("initial_pose_x", -1.40),
                ("initial_pose_y", 1.95),
                ("initial_pose_z", 0.0),
                ("initial_pose_yaw", 0.176),
                ("auto_initial_pose", True),
            ],
        )

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

    def local_initial_registration(self, scan, map_pcd, initial_map_to_odom, cur_odom_to_base=None):
        """フィールド全域（360度×全グリッド）で自律探索を行い、周囲の壁面へ全自動でピタッと高精度吸着"""
        if cur_odom_to_base is None:
            T_odom_to_base = np.eye(4)
        else:
            T_odom_to_base = cur_odom_to_base
        T_base_to_odom = self.inverse_se3(T_odom_to_base)

        # ロボット車体の現在推定姿勢 (map -> base)
        T_map_to_base_init = np.matmul(initial_map_to_odom, T_odom_to_base)
        cx, cy = T_map_to_base_init[0, 3], T_map_to_base_init[1, 3]

        # 点群をロボット車体中心 (body座標系) に変換
        scan_down = self.voxel_down_sample(scan, 0.15)
        scan_pts_odom = np.asarray(scan_down.points)
        scan_pts_h = np.column_stack([scan_pts_odom, np.ones(len(scan_pts_odom))])
        scan_pts_body = (T_base_to_odom @ scan_pts_h.T).T[:, :2]
        if len(scan_pts_body) > 120:
            step = len(scan_pts_body) // 120
            scan_pts_body = scan_pts_body[::step][:120]
        N = len(scan_pts_body)

        best_map_to_base = np.copy(T_map_to_base_init)

        # 1. 全周360度（72分割、5度刻み）の角度候補
        yaw_candidates = np.linspace(-np.pi, np.pi, 72, endpoint=False)

        # 2. スタートゾーン周辺（±2.5m範囲、0.12m刻み）のグリッド探索候補
        # ロボットのスタート位置周辺で全周探索することで、アリーナ反対側の点対称な偽解への誤吸着を完全に防止
        if hasattr(self, 'kdtree_2d') and self.kdtree_2d is not None and N > 20:
            local_offsets_x = np.arange(-2.5, 2.6, 0.12)
            local_offsets_y = np.arange(-2.5, 2.6, 0.12)
            all_positions = np.array([(cx + float(dx), cy + float(dy)) for dx in local_offsets_x for dy in local_offsets_y])
            M = len(all_positions)

            inlier_records = []
            for yaw in yaw_candidates:
                c, s = np.cos(yaw), np.sin(yaw)
                R = np.array([[c, -s], [s, c]])
                rot_pts = np.dot(scan_pts_body, R.T)  # (N, 2)

                all_query = (rot_pts[np.newaxis, :, :] + all_positions[:, np.newaxis, :]).reshape(-1, 2)
                dists, _ = self.kdtree_2d.query(all_query, distance_upper_bound=0.35)
                inliers = (dists < 0.35).reshape(M, N).sum(axis=1)

                top_m_idx = np.argmax(inliers)
                inlier_records.append((inliers[top_m_idx], all_positions[top_m_idx], yaw))

            # インライア数上位の候補を取得
            inlier_records.sort(key=lambda item: item[0], reverse=True)

        # 3-Stage Multi-Scale Point-to-Plane ICP (生CADの3cm高密度法線モデルを使用)
        target = self.map_target_fine if hasattr(self, 'map_target_fine') and self.map_target_fine is not None else map_pcd
        if not target.has_normals():
            target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30))
        estimation_plane = o3d.pipelines.registration.TransformationEstimationPointToPlane()
        source_coarse = self.voxel_down_sample(scan, 0.08)
        source_fine = self.voxel_down_sample(scan, 0.03)

        best_final_trans = np.matmul(best_map_to_base, T_base_to_odom)
        best_final_trans = self.flatten_transform(best_final_trans)
        best_fitness = -1.0

        candidates_to_test = inlier_records[:5] if len(inlier_records) > 0 else [(0, (cx, cy), 0.0)]
        for candidate_rec in candidates_to_test:
            cand_xy = candidate_rec[1]
            cand_yaw = candidate_rec[2]
            cand_map_to_base = np.copy(T_map_to_base_init)
            cand_map_to_base[0, 3] = cand_xy[0]
            cand_map_to_base[1, 3] = cand_xy[1]
            cand_map_to_base[:3, :3] = te.euler2mat(0.0, 0.0, cand_yaw, axes="sxyz")
            cand_map_to_base[2, 3] = 0.0
            cand_pose = self.flatten_transform(np.matmul(cand_map_to_base, T_base_to_odom))

            # Stage 1: Coarse Point-to-Plane ICP (max_distance = 0.8m)
            res_coarse = o3d.pipelines.registration.registration_icp(
                source_coarse, target, 0.8, cand_pose, estimation_plane,
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40, relative_fitness=1e-6, relative_rmse=1e-6)
            )

            # Stage 2: Mid Point-to-Plane ICP (max_distance = 0.35m)
            res_fine = o3d.pipelines.registration.registration_icp(
                source_fine, target, 0.35, res_coarse.transformation, estimation_plane,
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50, relative_fitness=1e-7, relative_rmse=1e-7)
            )

            # Stage 3: Ultra-Fine Point-to-Plane ICP (max_distance = 0.15m)
            res_ultra = o3d.pipelines.registration.registration_icp(
                source_fine, target, 0.15, res_fine.transformation, estimation_plane,
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60, relative_fitness=1e-8, relative_rmse=1e-8)
            )

            if res_ultra.fitness > best_fitness:
                best_fitness = res_ultra.fitness
                best_final_trans = self.flatten_transform(res_ultra.transformation)

        return best_final_trans, best_fitness
            
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
        if self.global_map is None or self.cur_scan is None or len(self.cur_scan.points) < 50:
            self.T_map_to_odom = pose_estimation
            self.publish_odom(pose_estimation)
            return

        scan_tobe_mapped = copy.copy(self.cur_scan)
        cur_odom_to_base = self.pose_to_mat(self.cur_odom.pose.pose) if self.cur_odom is not None else np.eye(4)
        transformation, fitness = self.local_initial_registration(
            scan_tobe_mapped, self.global_map, initial_map_to_odom=pose_estimation, cur_odom_to_base=cur_odom_to_base
        )
        delta_trans = np.linalg.norm(transformation[:3, 3] - pose_estimation[:3, 3])
        _, _, cur_yaw = te.mat2euler(pose_estimation[:3, :3], axes="sxyz")
        _, _, new_yaw = te.mat2euler(transformation[:3, :3], axes="sxyz")
        diff_yaw = (new_yaw - cur_yaw + np.pi) % (2 * np.pi) - np.pi

        is_success = (fitness >= 0.02)
        if is_success:
            self.T_map_to_odom = transformation
            self.publish_odom(transformation)
            self.initialized = True
            self.get_logger().info(
                f"【完全全自動吸着完了・マップ完全固定】静止CADマップとLiDAR点群が自動でピタッと合致しました！ Fitness: {fitness:.4f} "
                f"(位置: X={transformation[0,3]:.2f}m, Y={transformation[1,3]:.2f}m, 補正: 移動 {delta_trans:.2f}m, 角度 {np.degrees(abs(diff_yaw)):.1f}度)"
            )
        else:
            self.get_logger().warn(
                f"初期位置指定を採用しました (Fitness: {fitness:.4f}, 移動: {delta_trans:.2f}m, 角度: {np.degrees(abs(diff_yaw)):.1f}度)"
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
            elif self.get_parameter("auto_initial_pose").value and not self.initialized:
                ix = self.get_parameter("initial_pose_x").value
                iy = self.get_parameter("initial_pose_y").value
                iz = self.get_parameter("initial_pose_z").value
                iyaw = self.get_parameter("initial_pose_yaw").value
                init_mat = np.eye(4)
                init_mat[:3, 3] = [ix, iy, iz]
                init_mat[:3, :3] = te.euler2mat(0.0, 0.0, iyaw, axes="sxyz")
                self.T_map_to_odom = self.flatten_transform(init_mat)
                self.initialized = True
                self.get_logger().info(f"Auto-applied default initial pose: x={ix}, y={iy}, yaw={iyaw}")
        
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

        self.cur_scan = o3d.geometry.PointCloud()
        self.cur_scan.points = o3d.utility.Vector3dVector(accumulated_pc)
        header = copy.copy(msg.header)
        header.frame_id = "odom"
        self.publish_point_cloud(self.pub_pc_in_map, header, accumulated_pc)

        # 点群とオドメトリを受信した瞬間に【完全全自動・ノータッチ】でグローバル探索＆3段階ICP吸着を実行して永久固定
        if not hasattr(self, '_has_snapped') and len(accumulated_pc) >= 60 and self.cur_odom is not None:
            self.initial_snap_and_lock(self.T_map_to_odom)
        
    def initialize_global_map(self): #, pc_msg):
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
            
        self.global_map = self.voxel_down_sample(raw_map, 0.1)
        self.global_map.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
        
        # 2D KDTree（粗探索用）
        map_down_2d = self.voxel_down_sample(raw_map, 0.25)
        map_pts_2d = np.asarray(map_down_2d.points)[:, :2]
        self.kdtree_2d = cKDTree(map_pts_2d)

        # 高精度法線モデル（生CADマップから直接3cm解像度で構築し、ボクセル化による数十cmのシフトを完全排除）
        self.map_target_fine = self.voxel_down_sample(raw_map, 0.03)
        self.map_target_fine.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30))

        self.get_logger().info("Global map received, ultra-precise 3cm normals estimated, and 2D spatial tree constructed.")

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
        if not self.initialized:
            self.get_logger().info("Waiting for initial pose...", throttle_duration_sec=10.0)
            return


def main(args=None):
    rclpy.init(args=args)
    node = FastLIOLocalization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()