#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import copy
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
import rclpy.timer
import transforms3d.quaternions as tq
import transforms3d.euler as te
import tf2_ros
from geometry_msgs.msg import Transform
from std_msgs.msg import Header


class TransformFusion(Node):
    OFFICIAL_OBJECTS = [
        ("固定バケツ① B1", -0.870, 0.000, 0.10),
        ("固定バケツ② B2(H600)", -1.480, -1.820, 0.10),
        ("固定バケツ③ B3(H300)", -1.480, 1.820, 0.10),
        ("椅子 CHAIR", -5.155, 0.000, 0.10),
        ("机 DESK1", -3.860, -2.850, 0.10),
        ("机 DESK2", -3.860, 2.840, 0.10),
        ("机 DESK3", -5.560, 5.330, 0.10),
        ("机 DESK4", -1.080, 5.290, 0.10),
        ("旗 FLAG", -3.025, -0.120, 0.10),
        ("教壇 PODIUM", 0.000, 0.550, 0.15),
    ]

    def __init__(self):
        super().__init__("transform_fusion")

        self.cur_odom_to_baselink = None
        self.cur_map_to_odom = None
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.pub_localization = self.create_publisher(Odometry, "/localization", 1)
        self.pub_localization_path = self.create_publisher(Path, "/localization_path", 10)
        self.pub_robot_marker = self.create_publisher(Marker, "/robot_marker", 1)

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("enable_safety_kill", False)
        self.declare_parameter("court_boundary_margin_m", 0.0)
        self.enable_safety_kill = self.get_parameter("enable_safety_kill").value
        self.court_margin = self.get_parameter("court_boundary_margin_m").value

        odom_topic = self.get_parameter("odom_topic").value
        self.create_subscription(Odometry, odom_topic, self.cb_save_cur_odom, 1)
        self.create_subscription(Odometry, "/map_to_odom", self.cb_save_map_to_odom, 1)
        self.last_published_stamp = None

    @staticmethod
    def quat_xyzw_to_wxyz(quat_xyzw):
        quat = np.asarray(quat_xyzw, dtype=np.float64)
        return np.array([quat[3], quat[0], quat[1], quat[2]], dtype=np.float64)

    @staticmethod
    def quat_wxyz_to_xyzw(quat_wxyz):
        quat = np.asarray(quat_wxyz, dtype=np.float64)
        return np.array([quat[1], quat[2], quat[3], quat[0]], dtype=np.float64)

    def pose_to_mat(self, pose_msg):
        trans = np.eye(4)
        trans[:3, 3] = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        quat = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
        q_wxyz = self.quat_xyzw_to_wxyz(quat)
        trans[:3, :3] = tq.quat2mat(q_wxyz)
        return trans

    def transform_fusion(self):
        if self.cur_map_to_odom is not None:
            T_map_to_odom = self.pose_to_mat(self.cur_map_to_odom.pose.pose)
        else:
            # global_localization からの初回到着を待つ（未到着時の不要なワープ表示を防止）
            return

        transform_msg = Transform()
        transform_msg.translation.x = float(T_map_to_odom[0, 3])
        transform_msg.translation.y = float(T_map_to_odom[1, 3])
        transform_msg.translation.z = float(T_map_to_odom[2, 3])
        
        quat_wxyz = tq.mat2quat(T_map_to_odom[:3, :3])
        quat_xyzw = self.quat_wxyz_to_xyzw(quat_wxyz)

        transform_msg.rotation.x = float(quat_xyzw[0])
        transform_msg.rotation.y = float(quat_xyzw[1])
        transform_msg.rotation.z = float(quat_xyzw[2])
        transform_msg.rotation.w = float(quat_xyzw[3])
        
        now_stamp = self.cur_odom_to_baselink.header.stamp if self.cur_odom_to_baselink is not None else self.get_clock().now().to_msg()
        self.last_published_stamp = (now_stamp.sec, now_stamp.nanosec)

        transform_stamped_msg = tf2_ros.TransformStamped()
        transform_stamped_msg.header.stamp = now_stamp
        transform_stamped_msg.header.frame_id = "map"
        transform_stamped_msg.child_frame_id = "camera_init"
        transform_stamped_msg.transform = transform_msg
        self.tf_broadcaster.sendTransform(transform_stamped_msg)

        # また odom フレーム名にも念のため同等に TF ブロードキャスト
        tf_odom = tf2_ros.TransformStamped()
        tf_odom.header.stamp = now_stamp
        tf_odom.header.frame_id = "map"
        tf_odom.child_frame_id = "odom"
        tf_odom.transform = transform_msg
        self.tf_broadcaster.sendTransform(tf_odom)

        if self.cur_odom_to_baselink is None:
            return

        cur_odom = copy.copy(self.cur_odom_to_baselink)
        if cur_odom is not None:
            T_odom_to_base_link = self.pose_to_mat(cur_odom.pose.pose)
            T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)

            r, p, yaw = te.mat2euler(T_map_to_base_link[:3, :3], axes="sxyz")
            R_horizontal = te.euler2mat(0.0, 0.0, yaw, axes="sxyz")
            quat_wxyz = tq.mat2quat(R_horizontal)
            quat_xyzw = self.quat_wxyz_to_xyzw(quat_wxyz)

            xyz = np.copy(T_map_to_base_link[:3, 3])
            xyz[2] = 0.0  # 平面フィールド上のため上下の沈み込み・浮きを防止

            # 🚨 リアルタイム安全監視
            rx, ry = float(xyz[0]), float(xyz[1])
            if self.enable_safety_kill:
                if rx > self.court_margin:
                    self.get_logger().fatal(
                        f"🚨 [SAFETY KILLED] 相手コート侵入検知！ (X={rx:.3f}m > {self.court_margin:.2f}m)"
                    )
                    os._exit(99)
                for obj_name, ox, oy, limit_d in self.OFFICIAL_OBJECTS:
                    dist_to_obj = ((rx - ox) ** 2 + (ry - oy) ** 2) ** 0.5
                    if dist_to_obj < limit_d:
                        self.get_logger().fatal(
                            f"🚨 [SAFETY KILLED] [{obj_name}] 接触検知！ (距離={dist_to_obj:.3f}m < {limit_d:.3f}m)"
                        )
                        os._exit(98)
            else:
                # 警告のみ出力（ノード停止せず走行継続）
                if rx > self.court_margin:
                    self.get_logger().warn(f"⚠️ [COURT WARN] 相手コート接近中 (X={rx:.2f}m)")
                for obj_name, ox, oy, limit_d in self.OFFICIAL_OBJECTS:
                    dist_to_obj = ((rx - ox) ** 2 + (ry - oy) ** 2) ** 0.5
                    if dist_to_obj < limit_d:
                        self.get_logger().warn(f"⚠️ [PROXIMITY WARN] [{obj_name}] 接近 (距離={dist_to_obj:.2f}m)")

            localization = Odometry()
            localization.pose.pose = Pose(
                position = Point(x = float(xyz[0]), y = float(xyz[1]), z = float(xyz[2])), 
                orientation = Quaternion(x = float(quat_xyzw[0]), y = float(quat_xyzw[1]), z = float(quat_xyzw[2]), w = float(quat_xyzw[3]))
            )
            localization.twist = cur_odom.twist

            localization.header.stamp = cur_odom.header.stamp
            localization.header.frame_id = "map"
            localization.child_frame_id = "body"
            self.pub_localization.publish(localization)
            self.publish_robot_marker(localization)

            # Map-frame trajectory path publisher
            pose_stamped = PoseStamped()
            pose_stamped.header = localization.header
            pose_stamped.pose = localization.pose.pose
            if not hasattr(self, '_path_skip'):
                self._path_skip = 0
            self._path_skip += 1
            if self._path_skip % 3 == 0:
                self.path_msg.header.stamp = localization.header.stamp
                self.path_msg.poses.append(pose_stamped)
                self.pub_localization_path.publish(self.path_msg)

            if not hasattr(self, '_pub_count'):
                self._pub_count = 0
            self._pub_count += 1
            if self._pub_count % 50 == 1:
                self.get_logger().info(f"Robot localized pose: x={xyz[0]:.2f}, y={xyz[1]:.2f}, z={xyz[2]:.2f}")

    def publish_robot_marker(self, localization):
        marker = Marker()
        marker.header = localization.header
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = copy.deepcopy(localization.pose.pose)
        # 車体高0.4mの中心なので、地面(Z=0)から+0.2mに底面を接地させる
        marker.pose.position.z = max(marker.pose.position.z, 0.0) + 0.2
        marker.scale.x = 0.8
        marker.scale.y = 0.5
        marker.scale.z = 0.4
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 0.9
        self.pub_robot_marker.publish(marker)


    def cb_save_cur_odom(self, msg):
        self.cur_odom_to_baselink = msg
        self.transform_fusion()

    def cb_save_map_to_odom(self, msg):
        self.cur_map_to_odom = msg
        self.transform_fusion()


def main(args=None):
    rclpy.init(args=args)
    node = TransformFusion()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
