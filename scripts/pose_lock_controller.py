#!/usr/bin/env python3

import os
import sys
import time
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, String


class PoseLockController(Node):
    """
    起動後（またはrosbag再生開始後）の最初の 1.0 秒間のみ初期姿勢の確定を許可し、
    1.0 秒経過後は T_map_to_odom の変更や外部からの強制ワープをハードコードレベルで遮断・完全固定する。
    """
    def __init__(self):
        super().__init__("pose_lock_controller")

        self.declare_parameter("lock_time_sec", 1.0)
        self.lock_time_sec = self.get_parameter("lock_time_sec").value

        self.start_sim_time = None
        self.start_wall_time = time.time()
        self.is_locked = False
        self.has_first_snap = False

        self.pub_lock_status = self.create_publisher(Bool, "/pose_lock_status", 10)
        self.create_subscription(Clock, "/clock", self.cb_clock, 10)
        self.create_subscription(Odometry, "/map_to_odom", self.cb_map_to_odom, 10)

        # 10Hzでロック状態を監視・配信
        self.create_timer(0.1, self.check_lock_timer)
        self.get_logger().info(f"PoseLockController initialized: 1-second snap window (limit: {self.lock_time_sec:.1f}s)")

    def cb_clock(self, msg):
        cur_sec = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.start_sim_time is None:
            self.start_sim_time = cur_sec
            self.get_logger().info(f"Rosbag clock detected. Simulation start: {self.start_sim_time:.3f}s")
            return

        elapsed = cur_sec - self.start_sim_time
        if elapsed >= self.lock_time_sec and not self.is_locked:
            self.is_locked = True
            self.get_logger().info(f"【1秒経過・ワープ完全禁止】経過時間 {elapsed:.2f}s >= {self.lock_time_sec:.1f}s: 座標系を永久ロックしました。")
            self._publish_lock_msg(True)

    def cb_map_to_odom(self, msg):
        self.has_first_snap = True

    def check_lock_timer(self):
        # シミュレーション時間が来ていない場合も実時間でバックアップ監視
        if not self.is_locked:
            elapsed_wall = time.time() - self.start_wall_time
            # rosbagが止まっていないのに実時間で4秒以上経っていれば安全のためロック
            if elapsed_wall >= (self.lock_time_sec + 3.0) and self.has_first_snap:
                self.is_locked = True
                self.get_logger().info(f"【タイムアウト・ワープ完全禁止】実時間 {elapsed_wall:.2f}s: 座標系を永久ロックしました。")
                self._publish_lock_msg(True)
        else:
            self._publish_lock_msg(True)

    def _publish_lock_msg(self, locked: bool):
        msg = Bool()
        msg.data = locked
        self.pub_lock_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseLockController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
