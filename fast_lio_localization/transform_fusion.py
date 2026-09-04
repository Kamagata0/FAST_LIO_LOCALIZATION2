#!/usr/bin/env python3

import copy
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import rclpy.timer
import transforms3d.quaternions as tq
import transforms3d.euler as te
import tf2_ros
from geometry_msgs.msg import Transform
from std_msgs.msg import Header


class TransformFusion(Node):
    def __init__(self):
        super().__init__("transform_fusion")

        self.cur_odom_to_baselink = None
        self.cur_map_to_odom = None

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.pub_localization = self.create_publisher(Odometry, "/localization", 1)
        self.pub_robot_marker = self.create_publisher(Marker, "/robot_marker", 1)

        self.declare_parameter("odom_topic", "/odom")
        odom_topic = self.get_parameter("odom_topic").value
        self.create_subscription(Odometry, odom_topic, self.cb_save_cur_odom, 1)
        self.create_subscription(Odometry, "/map_to_odom", self.cb_save_map_to_odom, 1)

        self.freq_pub_localization = 50
        self.timer = self.create_timer(1/self.freq_pub_localization, self.transform_fusion)
        # threading.Thread(target=self.transform_fusion, daemon=True).start()

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
        if self.cur_odom_to_baselink is None:
            return

        if self.cur_map_to_odom is not None:
            T_map_to_odom = self.pose_to_mat(self.cur_map_to_odom.pose.pose)
        else:
            T_map_to_odom = np.eye(4)

        transform_msg = Transform()
        transform_msg.translation.x = T_map_to_odom[0, 3]
        transform_msg.translation.y = T_map_to_odom[1, 3]
        transform_msg.translation.z = T_map_to_odom[2, 3]
        
        quat_wxyz = tq.mat2quat(T_map_to_odom[:3, :3])
        quat_xyzw = self.quat_wxyz_to_xyzw(quat_wxyz)

        transform_msg.rotation.x = quat_xyzw[0]
        transform_msg.rotation.y = quat_xyzw[1]
        transform_msg.rotation.z = quat_xyzw[2]
        transform_msg.rotation.w = quat_xyzw[3]
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.cur_odom_to_baselink.header.frame_id
        
        # print(self.cur_odom_to_baselink.header)
        transform_stamped_msg = tf2_ros.TransformStamped(
                header = self.cur_odom_to_baselink.header,
            child_frame_id = "odom",
                transform = transform_msg
            )
        transform_stamped_msg.header.frame_id = "map"
        self.tf_broadcaster.sendTransform(transform_stamped_msg)

        cur_odom = copy.copy(self.cur_odom_to_baselink)
        if cur_odom is not None:
            T_odom_to_base_link = self.pose_to_mat(cur_odom.pose.pose)
            T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)

            quat_wxyz = tq.mat2quat(T_map_to_base_link[:3, :3])
            quat_xyzw = self.quat_wxyz_to_xyzw(quat_wxyz)

            xyz = T_map_to_base_link[:3, 3]

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

    def cb_save_map_to_odom(self, msg):
        self.cur_map_to_odom = msg


def main(args=None):
    rclpy.init(args=args)
    node = TransformFusion()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
