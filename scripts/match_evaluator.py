#!/usr/bin/env python3

import os
import sys
import time
import json
import shutil
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Bool
import ros2_numpy
import transforms3d.euler as te
import transforms3d.quaternions as tq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load_pcd_binary(pcd_path):
    if not os.path.exists(pcd_path):
        return None
    try:
        with open(pcd_path, "rb") as f:
            while True:
                line = f.readline().decode("ascii", errors="ignore")
                if line.startswith("DATA"):
                    break
            data = f.read()
        pts = np.frombuffer(data, dtype=np.float32).reshape(-1, 3)
        return pts
    except Exception:
        return None


class MatchEvaluator(Node):
    def __init__(self, output_dir, trial=1, target_fitness=0.99, map_pcd_path=None, params_desc=""):
        super().__init__("match_evaluator")
        self.output_dir = output_dir
        self.trial = trial
        self.target_fitness = target_fitness
        self.map_pcd_path = map_pcd_path
        self.params_desc = params_desc

        self.first_dir = os.path.join(self.output_dir, "first")
        self.best_dir = os.path.join(self.output_dir, "best")
        self.temp_dir = os.path.join(self.output_dir, f"trial_{self.trial}")
        os.makedirs(self.first_dir, exist_ok=True)
        os.makedirs(self.best_dir, exist_ok=True)
        os.makedirs(self.temp_dir, exist_ok=True)

        self.trajectory = []
        self.map_to_odom_record = None
        self.metric_record = None
        self.accumulated_scan_points = []
        self.start_time = time.time()
        self.last_msg_time = time.time()
        self.is_pose_locked = False

        self.create_subscription(Odometry, "/localization", self.cb_localization, 10)
        self.create_subscription(Odometry, "/map_to_odom", self.cb_map_to_odom, 10)
        self.create_subscription(PointCloud2, "/cur_scan_in_map", self.cb_scan, 10)
        self.create_subscription(String, "/localization_metric", self.cb_metric, 10)
        self.create_subscription(Bool, "/pose_lock_status", self.cb_lock_status, 10)

        self.get_logger().info(
            f"[Trial #{self.trial}] MatchEvaluator running. Target: {self.target_fitness:.2f} (Params: {self.params_desc})"
        )

    def cb_lock_status(self, msg):
        self.is_pose_locked = msg.data

    def cb_metric(self, msg):
        try:
            self.metric_record = json.loads(msg.data)
            self.get_logger().info(
                f"[Trial #{self.trial}] Match metric: Fitness={self.metric_record.get('fitness', 0.0):.4f}, "
                f"Status={self.metric_record.get('status')}"
            )
        except Exception:
            pass
        self.last_msg_time = time.time()

    def cb_map_to_odom(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        quat_wxyz = [ori.w, ori.x, ori.y, ori.z]
        _, _, yaw = te.quat2euler(quat_wxyz, axes='sxyz')
        self.map_to_odom_record = {
            "x": pos.x,
            "y": pos.y,
            "z": pos.z,
            "yaw_deg": np.degrees(yaw),
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }
        self.last_msg_time = time.time()

    def cb_localization(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.trajectory.append([stamp, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w])
        self.last_msg_time = time.time()

    def cb_scan(self, msg):
        try:
            pc_array = ros2_numpy.numpify(msg)
            pts = pc_array["xyz"]
            if len(pts) > 0:
                sub_pts = pts[::4, :3]
                self.accumulated_scan_points.append(sub_pts)
                if len(self.accumulated_scan_points) > 100:
                    self.accumulated_scan_points.pop(0)
            self.last_msg_time = time.time()
        except Exception:
            pass

    def evaluate_and_save(self):
        duration = time.time() - self.start_time
        traj_arr = np.array(self.trajectory)
        fitness = float(self.metric_record.get("fitness", 0.0)) if self.metric_record else 0.0

        # ベストスコア読み込み
        best_score_file = os.path.join(self.output_dir, ".best_fitness")
        prev_best = -1.0
        if os.path.exists(best_score_file):
            try:
                with open(best_score_file, "r") as f:
                    prev_best = float(f.read().strip())
            except Exception:
                prev_best = -1.0

        is_new_best = (fitness > prev_best)
        if is_new_best:
            with open(best_score_file, "w") as f:
                f.write(f"{fitness:.6f}\n")

        # 1. 軌跡長計算
        total_distance = 0.0
        if len(traj_arr) > 1:
            diffs = np.diff(traj_arr[:, 1:4], axis=0)
            total_distance = np.sum(np.linalg.norm(diffs, axis=1))

        # 2. 画像描画
        screenshot_tmp = os.path.join(self.temp_dir, "screenshot.png")
        all_scans = None
        try:
            plt.figure(figsize=(11, 9), dpi=150)
            plt.title(f"Trial #{self.trial} Alignment [Fitness: {fitness:.4f}]", fontsize=13, fontweight='bold')
            plt.xlabel("X (meters)", fontsize=12)
            plt.ylabel("Y (meters)", fontsize=12)
            plt.grid(True, linestyle="--", alpha=0.5)

            # Static CAD map
            if self.map_pcd_path:
                map_pts = load_pcd_binary(self.map_pcd_path)
                if map_pts is not None and len(map_pts) > 0:
                    plt.scatter(map_pts[::12, 0], map_pts[::12, 1], s=1, c="black", alpha=0.45, label="Static CAD Map Walls")

            # Point clouds in map frame
            if len(self.accumulated_scan_points) > 0:
                all_scans = np.vstack(self.accumulated_scan_points)
                if self.map_to_odom_record is not None:
                    T = np.eye(4)
                    T[0, 3] = self.map_to_odom_record["x"]
                    T[1, 3] = self.map_to_odom_record["y"]
                    yaw = np.radians(self.map_to_odom_record["yaw_deg"])
                    T[:2, :2] = [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
                    pts_h = np.column_stack([all_scans, np.ones(len(all_scans))])
                    all_scans = (T @ pts_h.T).T[:, :3]
                plt.scatter(all_scans[::3, 0], all_scans[::3, 1], s=2, c="cyan", alpha=0.4, label="Mid-360 LiDAR Cloud")

            # Trajectory
            if len(traj_arr) > 0:
                plt.plot(traj_arr[:, 1], traj_arr[:, 2], color="crimson", linewidth=2.5, label="Robot Path")
                plt.scatter([traj_arr[0, 1]], [traj_arr[0, 2]], color="lime", edgecolors="black", s=110, marker="o", label="Start Pose", zorder=5)
                plt.scatter([traj_arr[-1, 1]], [traj_arr[-1, 2]], color="red", edgecolors="black", s=110, marker="X", label="End Pose", zorder=5)

            plt.legend(loc="upper right", fontsize=10)
            plt.axis("equal")
            plt.xlim(-7.0, 7.0)
            plt.ylim(-6.5, 7.0)
            plt.tight_layout()
            plt.savefig(screenshot_tmp)
            plt.close()
        except Exception as e:
            self.get_logger().warn(f"Plotting failed: {e}")

        # 3. 成果物振り分け (初回 = first/, ベスト = best/)
        # A) 初回試行 (Trial 1) の保存
        if self.trial == 1:
            first_img = os.path.join(self.first_dir, "trial_1_screenshot.png")
            first_txt = os.path.join(self.first_dir, "trial_1_metrics.txt")
            if os.path.exists(screenshot_tmp):
                shutil.copy(screenshot_tmp, first_img)
            with open(first_txt, "w", encoding="utf-8") as f:
                f.write("=====================================================\n")
                f.write(" FIRST TRIAL METRICS (Trial #1 Baseline)\n")
                f.write("=====================================================\n")
                f.write(f"Date & Time        : {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Trial Number       : 1\n")
                f.write(f"Fitness Score      : {fitness:.4f}\n")
                f.write(f"Parameters         : {self.params_desc}\n")
                f.write(f"Trajectory Length  : {total_distance:.2f} m ({len(traj_arr)} poses)\n")
                f.write(f"Duration           : {duration:.2f} s\n")
                if self.map_to_odom_record:
                    f.write(f"Map->Odom Snap     : X={self.map_to_odom_record['x']:.3f}, "
                            f"Y={self.map_to_odom_record['y']:.3f}, Yaw={self.map_to_odom_record['yaw_deg']:.2f} deg\n")
                f.write("=====================================================\n")
            self.get_logger().info(f"[First Trial] Recorded to {self.first_dir}")

        # B) ベスト更新時 (Best) の保存
        if is_new_best:
            best_img = os.path.join(self.best_dir, "best_trial_screenshot.png")
            best_csv = os.path.join(self.best_dir, "best_trajectory.csv")
            best_pcd = os.path.join(self.best_dir, "best_aligned_result.pcd")
            best_txt = os.path.join(self.best_dir, "best_summary.txt")

            if os.path.exists(screenshot_tmp):
                shutil.copy(screenshot_tmp, best_img)
            if len(traj_arr) > 0:
                np.savetxt(best_csv, traj_arr, delimiter=",", header="stamp,x,y,z,qx,qy,qz,qw", comments="")
            
            # PCD 保存
            try:
                import open3d as o3d
                combined = o3d.geometry.PointCloud()
                if self.map_pcd_path and os.path.exists(self.map_pcd_path):
                    combined += o3d.io.read_point_cloud(self.map_pcd_path)
                if all_scans is not None and len(all_scans) > 0:
                    scan_pcd = o3d.geometry.PointCloud()
                    scan_pcd.points = o3d.utility.Vector3dVector(all_scans)
                    combined += scan_pcd
                if len(combined.points) > 0:
                    o3d.io.write_point_cloud(best_pcd, combined)
            except Exception:
                pass

            # Summary 保存
            with open(best_txt, "w", encoding="utf-8") as f:
                f.write("=====================================================\n")
                f.write(" ALL-TIME BEST LOCALIZATION RECORD (Near 100% Match)\n")
                f.write("=====================================================\n")
                f.write(f"Updated At         : {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Best Trial Index   : {self.trial}\n")
                f.write(f"Best Fitness Score : {fitness:.4f}\n")
                f.write(f"Best Parameters    : {self.params_desc}\n")
                f.write(f"Trajectory Length  : {total_distance:.2f} m ({len(traj_arr)} poses)\n")
                f.write(f"Duration           : {duration:.2f} s\n")
                if self.map_to_odom_record:
                    f.write(f"Optimal Map->Odom  : X={self.map_to_odom_record['x']:.3f}, "
                            f"Y={self.map_to_odom_record['y']:.3f}, Yaw={self.map_to_odom_record['yaw_deg']:.2f} deg\n")
                f.write("=====================================================\n")

            self.get_logger().info(f"★ NEW ALL-TIME BEST! Fitness: {fitness:.4f} recorded in {self.best_dir}")

        # 一時ディレクトリのクリーンアップ（firstとbest以外の不要ファイルを削除）
        try:
            shutil.rmtree(self.temp_dir)
        except Exception:
            pass

        # 目標達成判定
        is_target_met = (fitness >= self.target_fitness)
        return 0 if is_target_met else 1


def main():
    parser = argparse.ArgumentParser(description="Match Evaluator for Extreme Overnight Localization")
    parser.add_argument("--trial", type=int, default=1, help="Trial index")
    parser.add_argument("--target-fitness", type=float, default=0.99, help="Target fitness score")
    parser.add_argument("--output-dir", default="experiment_results", help="Directory to store results")
    parser.add_argument("--map-pcd", default="maps/robocon2026_field.pcd", help="Static PCD map path")
    parser.add_argument("--params-desc", default="", help="Description of trial parameters")
    parser.add_argument("--idle-timeout", type=float, default=5.0, help="Idle timeout after last message")
    parser.add_argument("--max-duration", type=float, default=180.0, help="Max duration in seconds")
    args, unknown = parser.parse_known_args()

    rclpy.init()
    evaluator = MatchEvaluator(
        output_dir=args.output_dir,
        trial=args.trial,
        target_fitness=args.target_fitness,
        map_pcd_path=args.map_pcd,
        params_desc=args.params_desc
    )

    exit_code = 1
    start = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(evaluator, timeout_sec=0.2)
            elapsed = time.time() - start
            since_last = time.time() - evaluator.last_msg_time

            if elapsed > args.max_duration:
                evaluator.get_logger().info(f"Max duration ({args.max_duration}s) reached.")
                break
            if len(evaluator.trajectory) > 50 and since_last > args.idle_timeout:
                evaluator.get_logger().info(f"Rosbag playback finished (idle for {since_last:.1f}s). Finalizing...")
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            exit_code = evaluator.evaluate_and_save()
        except Exception as e:
            print(f"Evaluation error: {e}")
            exit_code = 1
        evaluator.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
