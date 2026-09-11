#!/usr/bin/env python3

import argparse
import numpy as np
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
import transforms3d.euler as te


class PublishInitialPose(Node):
    def __init__(self):
        super().__init__("publish_initial_pose")
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    def publish_pose(self, x, y, z, roll, pitch, yaw, repeat=10):
        # 購読者（global_localization）が見つかるまで最大5秒待機
        start_wait = time.time()
        while self.pub_pose.get_subscription_count() == 0 and (time.time() - start_wait) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        quat_wxyz = te.euler2quat(roll, pitch, yaw, axes='sxyz')
        quat_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]], dtype=float)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position = Point(x=float(x), y=float(y), z=float(z))
        msg.pose.pose.orientation = Quaternion(
            x=float(quat_xyzw[0]),
            y=float(quat_xyzw[1]),
            z=float(quat_xyzw[2]),
            w=float(quat_xyzw[3]),
        )

        for _ in range(repeat):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_pose.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        self.get_logger().info(f"Published Initial Pose to {self.pub_pose.get_subscription_count()} subscriber(s): x={x}, y={y}, z={z}, yaw={yaw}")


def main(args=None):
    rclpy.init(args=args)
    node = PublishInitialPose()

    parser = argparse.ArgumentParser(description="Publish initial pose to /initialpose")
    parser.add_argument("x", type=float, default=-5.0, nargs="?")
    parser.add_argument("y", type=float, default=4.5, nargs="?")
    parser.add_argument("z", type=float, default=0.0, nargs="?")
    parser.add_argument("yaw", type=float, default=0.0, nargs="?")
    parser.add_argument("pitch", type=float, default=0.0, nargs="?")
    parser.add_argument("roll", type=float, default=0.0, nargs="?")
    parser.add_argument("--repeat", type=int, default=5)
    parsed = parser.parse_args()

    node.publish_pose(parsed.x, parsed.y, parsed.z, parsed.roll, parsed.pitch, parsed.yaw, repeat=parsed.repeat)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
