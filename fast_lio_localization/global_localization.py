#!/usr/bin/env python3

import copy
import threading
import time

import open3d as o3d
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
# from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import numpy as np
import tf2_ros
from tf2_ros import TransformException
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
import transforms3d.quaternions as tq
import transforms3d.euler as te
from collections import deque
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
                ("localization_threshold", 0.3),
                ("max_height", 2.2),
                ("fov", 6.28319),
                ("fov_far", 300),
                ("pcd_map_topic", "/map"),
                ("pcd_map_path", ""),
                ("lidar_topic", "/livox/lidar"),
                ("odom_topic", "/odom"),
            ],
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # self.pub_global_map = self.create_publisher(PointCloud2, self.get_parameter("pcd_map_topic").value, 10)
        self.pub_pc_in_map = self.create_publisher(PointCloud2, "/cur_scan_in_map", 10)
        self.pub_submap = self.create_publisher(PointCloud2, "/submap", 10)
        self.pub_map_to_odom = self.create_publisher(Odometry, "/map_to_odom", 10)

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
    
    def registration_at_scale(self, scan, map, initial, scale):
        result_icp = o3d.pipelines.registration.registration_icp(
        self.voxel_down_sample(scan, self.get_parameter("scan_voxel_size").value * scale),
        self.voxel_down_sample(map, self.get_parameter("map_voxel_size").value * scale),
        1.0 * scale,
        initial,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20),
        )
        return result_icp.transformation, result_icp.fitness
            
    def inverse_se3(self, trans):
        trans_inverse = np.eye(4)
        # R
        trans_inverse[:3, :3] = trans[:3, :3].T
        # t
        trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
        return trans_inverse

    def flatten_transform(self, trans):
        """Roll and Pitch を 0 に強制し、Yaw (水平向き) のみ保持した同次変換行列を返す"""
        r, p, yaw = te.mat2euler(trans[:3, :3], axes='sxyz')
        flat_trans = np.copy(trans)
        flat_trans[:3, :3] = te.euler2mat(0.0, 0.0, yaw, axes='sxyz')
        return flat_trans

    def publish_point_cloud(self, publisher, header, pc):
        data = dict()
        data["xyz"] = pc[:, :3]
        
        if pc.shape[1] == 4:
            data["intensity"] = pc[:, 3]
        # else:
            # data["rgb"] = np.ones_like(pc)
        msg = ros2_numpy.msgify(PointCloud2, data)
        msg.header = header
        if len(msg.fields) == 4:
            msg.point_step = 16
        else:
            msg.point_step = 12
            
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

    def global_localization(self, pose_estimation):
        if self.global_map is None:
            self.get_logger().warn("Global map is not available yet. Skip localization.")
            return
        if self.cur_scan is None or len(self.cur_scan.points) < 50:
            self.get_logger().warn("Current scan is not ready or has too few points. Skip localization.")
            return

        scan_tobe_mapped = copy.copy(self.cur_scan)
        global_map_in_FOV = self.crop_global_map_in_FOV(pose_estimation)
        if len(global_map_in_FOV.points) < 50:
            self.get_logger().warn("Submap in FOV has too few points. Skip localization.")
            return

        # 初回・走行中ともに scale=1 で精密マッチング（広範囲探索による遠くの壁への誤吸着・ジャンプを防止）
        transformation, fitness = self.registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=1)
        transformation = self.flatten_transform(transformation)

        threshold = self.get_parameter("localization_threshold").value
        delta_trans = np.linalg.norm(transformation[:3, 3] - pose_estimation[:3, 3])
        _, _, cur_yaw = te.mat2euler(pose_estimation[:3, :3], axes="sxyz")
        _, _, new_yaw = te.mat2euler(transformation[:3, :3], axes="sxyz")
        diff_yaw = (new_yaw - cur_yaw + np.pi) % (2 * np.pi) - np.pi

        # 初回の初期位置合わせ判定
        if not hasattr(self, '_has_converged'):
            # ユーザーが指定した位置から 0.4m 以上または 15度 以上離れる誤吸着はブロック
            if delta_trans > 0.4 or abs(diff_yaw) > 0.26:
                self.get_logger().warn(
                    f"初期位置からの移動量が大きすぎるため補正をスキップし、指定位置を採用しました "
                    f"(移動: {delta_trans:.2f}m > 0.4m, 角度: {np.degrees(abs(diff_yaw)):.1f}度 > 15度)"
                )
                self._has_converged = True
                self.T_map_to_odom = pose_estimation
                self.publish_odom(pose_estimation)
                return

            if fitness >= threshold:
                self._has_converged = True
                self.T_map_to_odom = transformation
                self.publish_odom(transformation)
                self.get_logger().info(f"初期位置の精密合わせに成功しました！ Fitness: {fitness:.4f} (移動: {delta_trans:.2f}m)")
            else:
                self.get_logger().warn(f"Fitness ({fitness:.4f}) が閾値 ({threshold:.4f}) 未満のため、指定位置をベースにします。")
                self._has_converged = True
                self.T_map_to_odom = pose_estimation
                self.publish_odom(pose_estimation)
            return

        # 走行中の補正判定
        if fitness >= threshold:
            # 走行中に0.5m以上の急激なワープをブロック
            if delta_trans > 0.5:
                self.get_logger().warn(f"ワープ防止: 変化量が大きすぎるため補正を棄却しました ({delta_trans:.2f} m > 0.5 m)")
                return

            if abs(diff_yaw) > 0.35: # > 20度
                self.get_logger().warn(f"ワープ防止: 角度変化が大きすぎるため補正を棄却しました ({np.degrees(abs(diff_yaw)):.1f}度 > 20度)")
                return

            # スムージング (急激なカクつきを抑えて滑らかに追従)
            alpha = 0.5
            smooth_trans = np.copy(transformation)
            smooth_trans[:3, 3] = (1 - alpha) * pose_estimation[:3, 3] + alpha * transformation[:3, 3]
            smooth_yaw = cur_yaw + alpha * diff_yaw
            smooth_trans[:3, :3] = te.euler2mat(0.0, 0.0, smooth_yaw, axes="sxyz")
            self.T_map_to_odom = smooth_trans
            self.publish_odom(smooth_trans)
        else:
            self.get_logger().warn(f"Fitness score {fitness:.4f} less than threshold {threshold:.4f}")

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
            if self.pending_initial_pose is not None:
                pose_msg, frame_id = self.pending_initial_pose
                self._handle_initial_pose(pose_msg, frame_id)
        
    def cb_save_cur_scan(self, msg):
        if not hasattr(self, '_scan_count'):
            self._scan_count = 0
        self._scan_count += 1
        if self._scan_count % 30 == 1:
            self.get_logger().info(f"LiDAR scan # {self._scan_count} received ({msg.width * msg.height} points). FAST-LIO is active!")
        if msg.header.frame_id == "odom":
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

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        interval = 1.0 / self.get_parameter("freq_localization").value
        if self.initialized and (now_sec - self.last_localization_time >= interval or now_sec < self.last_localization_time):
            self.last_localization_time = now_sec
            if self.cur_scan is not None and self.cur_odom is not None:
                self.global_localization(self.T_map_to_odom)
        
    def initialize_global_map(self): #, pc_msg):
        # self.global_map = o3d.geometry.PointCloud()
        # self.global_map.points = o3d.utility.Vector3dVector(self.msg_to_array(pc_msg)[:, :3])
        map_path = self.get_parameter("pcd_map_path").value
        if not map_path:
            self.get_logger().warn("No map file path provided. Global map is not loaded yet. Please launch with map:=/path/to/map.pcd")
            self.global_map = None
            return

        self.global_map = o3d.io.read_point_cloud(map_path)
        if self.global_map is None or len(self.global_map.points) == 0:
            self.get_logger().error(f"Failed to load global map from: {map_path}")
            self.global_map = None
            return
        self.global_map = self.voxel_down_sample(self.global_map, self.get_parameter("map_voxel_size").value)
        # o3d.io.write_point_cloud("/home/wheelchair2/laksh_ws/pcds/lab_map_with_outside_corridor (with ground pcd)_downsampled.pcd", self.global_map)
        self.get_logger().info("Global map received.")

    def _handle_initial_pose(self, pose_msg, frame_id):
        if hasattr(self, '_has_converged'):
            del self._has_converged
        self.scan_buffer.clear()
        self.pending_initial_pose = (pose_msg, frame_id)
        initial_map_to_base = self.pose_to_mat(pose_msg)
        if self.cur_odom is None:
            self.get_logger().info("Initial pose received, applying initial estimate (waiting for odometry)...")
            initial_pose = initial_map_to_base
        else:
            initial_pose = np.matmul(initial_map_to_base, self.inverse_se3(self.pose_to_mat(self.cur_odom.pose.pose)))

        initial_pose = self.flatten_transform(initial_pose)
        self.T_map_to_odom = initial_pose
        self.initialized = True
        self.get_logger().info(f"Initial pose set successfully (frame: {frame_id}).")
        self.publish_odom(initial_pose)

        if self.cur_scan is not None and self.cur_odom is not None:
            self.global_localization(initial_pose)

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
            self.get_logger().info("Waiting for initial pose...")
            return
        
        if self.cur_scan is not None and self.cur_odom is not None:
            self.global_localization(self.T_map_to_odom)


def main(args=None):
    rclpy.init(args=args)
    node = FastLIOLocalization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()