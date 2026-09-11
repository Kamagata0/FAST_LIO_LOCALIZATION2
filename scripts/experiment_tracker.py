#!/usr/bin/env python3

import os
import sys
import time
import json
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
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


class ExperimentTracker(Node):
    def __init__(self, output_dir, trial=1, fitness_thresh=0.70, map_pcd_path=None):
        super().__init__("experiment_tracker")
        self.output_dir = output_dir
        self.trial = trial
        self.fitness_thresh = fitness_thresh
        self.map_pcd_path = map_pcd_path
        os.makedirs(self.output_dir, exist_ok=True)

        self.trajectory = []
        self.map_to_odom_record = None
        self.metric_record = None
        self.accumulated_scan_points = []
        self.start_time = time.time()
        self.last_msg_time = time.time()

        self.create_subscription(Odometry, "/localization", self.cb_localization, 10)
        self.create_subscription(Odometry, "/map_to_odom", self.cb_map_to_odom, 10)
        self.create_subscription(PointCloud2, "/cur_scan_in_map", self.cb_scan, 10)
        self.create_subscription(String, "/localization_metric", self.cb_metric, 10)

        self.get_logger().info(f"ExperimentTracker (Trial #{self.trial}) started. Threshold: {self.fitness_thresh:.2f}")

    def cb_metric(self, msg):
        try:
            self.metric_record = json.loads(msg.data)
            self.get_logger().info(f"[Trial #{self.trial}] Metric received: Fitness={self.metric_record.get('fitness', 0.0):.4f}, Status={self.metric_record.get('status')}")
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
                if len(self.accumulated_scan_points) > 80:
                    self.accumulated_scan_points.pop(0)
            self.last_msg_time = time.time()
        except Exception:
            pass

    def save_results(self):
        duration = time.time() - self.start_time
        traj_arr = np.array(self.trajectory)

        fitness = self.metric_record.get("fitness", 0.0) if self.metric_record else 0.0
        is_passed = (fitness >= self.fitness_thresh) or (self.map_to_odom_record is not None and fitness >= 0.20)

        # 1. Save trajectory.csv
        csv_path = os.path.join(self.output_dir, "trajectory.csv")
        if len(traj_arr) > 0:
            np.savetxt(csv_path, traj_arr, delimiter=",", header="stamp,x,y,z,qx,qy,qz,qw", comments="")
            self.get_logger().info(f"Saved trajectory: {csv_path} ({len(traj_arr)} poses)")

        # 2. Compute path metrics
        total_distance = 0.0
        if len(traj_arr) > 1:
            diffs = np.diff(traj_arr[:, 1:4], axis=0)
            total_distance = np.sum(np.linalg.norm(diffs, axis=1))

        # 3. Append to trial_summary.txt
        summary_path = os.path.join(self.output_dir, "trial_summary.txt")
        status_str = "SUCCESS" if is_passed else "FAIL (RETRY REQUIRED)"
        with open(summary_path, "a", encoding="utf-8") as f:
            f.write(f"[Trial {self.trial}] {time.strftime('%Y-%m-%d %H:%M:%S')} | Status: {status_str} | Fitness: {fitness:.4f} | Distance: {total_distance:.2f}m | Duration: {duration:.1f}s\n")

        # 4. Save metrics.txt (latest summary)
        metrics_path = os.path.join(self.output_dir, "metrics.txt")
        with open(metrics_path, "w", encoding="utf-8") as f:
            f.write("=====================================================\n")
            f.write(f" OVERNIGHT LOCALIZATION EXPERIMENT (Trial #{self.trial}) \n")
            f.write("=====================================================\n")
            f.write(f"Date & Time        : {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Trial Number       : {self.trial}\n")
            f.write(f"Evaluation Status  : {status_str}\n")
            f.write(f"Fitness Score      : {fitness:.4f} (Threshold: {self.fitness_thresh:.2f})\n")
            f.write(f"Experiment Duration: {duration:.2f} seconds\n")
            f.write(f"Trajectory Length  : {total_distance:.2f} meters ({len(traj_arr)} poses)\n")
            if self.map_to_odom_record is not None:
                f.write(f"Snapped Map->Odom  : X={self.map_to_odom_record['x']:.3f} m, "
                        f"Y={self.map_to_odom_record['y']:.3f} m, "
                        f"Yaw={self.map_to_odom_record['yaw_deg']:.2f} deg\n")
            f.write("=====================================================\n")
        self.get_logger().info(f"Saved metrics summary: {metrics_path}")

        # 5. Generate 2D Alignment Plot (Trial screenshot and latest)
        plot_trial_path = os.path.join(self.output_dir, f"trial_{self.trial}_screenshot.png")
        plot_latest_path = os.path.join(self.output_dir, "alignment_plot.png")
        try:
            plt.figure(figsize=(11, 9), dpi=150)
            plt.title(f"Trial #{self.trial} Alignment Screenshot [Fitness: {fitness:.3f} - {status_str}]", fontsize=13, fontweight='bold')
            plt.xlabel("X (meters)", fontsize=12)
            plt.ylabel("Y (meters)", fontsize=12)
            plt.grid(True, linestyle="--", alpha=0.5)

            # Draw static CAD map walls
            if self.map_pcd_path:
                map_pts = load_pcd_binary(self.map_pcd_path)
                if map_pts is not None and len(map_pts) > 0:
                    plt.scatter(map_pts[::15, 0], map_pts[::15, 1], s=1, c="black", alpha=0.45, label="Static CAD Map Walls")

            all_scans = None
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
                plt.scatter(all_scans[::3, 0], all_scans[::3, 1], s=2, c="cyan", alpha=0.4, label="LiDAR Point Cloud")

            # Draw trajectory
            if len(traj_arr) > 0:
                plt.plot(traj_arr[:, 1], traj_arr[:, 2], color="crimson", linewidth=2.5, label="Robot Path")
                plt.scatter([traj_arr[0, 1]], [traj_arr[0, 2]], color="lime", edgecolors="black", s=110, marker="o", label="Start Pose", zorder=5)
                plt.scatter([traj_arr[-1, 1]], [traj_arr[-1, 2]], color="red", edgecolors="black", s=110, marker="X", label="End Pose", zorder=5)

            plt.legend(loc="upper right", fontsize=10)
            plt.axis("equal")
            plt.xlim(-7.0, 7.0)
            plt.ylim(-6.5, 7.0)
            plt.tight_layout()
            plt.savefig(plot_trial_path)
            plt.savefig(plot_latest_path)
            plt.close()
            self.get_logger().info(f"Saved alignment screenshot with CAD map: {plot_trial_path}")
        except Exception as e:
            self.get_logger().warn(f"Failed to generate plot: {e}")

        # 6. Save aligned PCD
        pcd_out = os.path.join(self.output_dir, "aligned_result.pcd")
        try:
            import open3d as o3d
            combined = o3d.geometry.PointCloud()
            if self.map_pcd_path and os.path.exists(self.map_pcd_path):
                map_pcd = o3d.io.read_point_cloud(self.map_pcd_path)
                combined += map_pcd
            if all_scans is not None and len(all_scans) > 0:
                scan_pcd = o3d.geometry.PointCloud()
                scan_pcd.points = o3d.utility.Vector3dVector(all_scans)
                combined += scan_pcd
            if len(combined.points) > 0:
                o3d.io.write_point_cloud(pcd_out, combined)
                self.get_logger().info(f"Saved merged PCD: {pcd_out}")
        except Exception:
            pass

        return 0 if is_passed else 1


def main():
    parser = argparse.ArgumentParser(description="Experiment Tracker & Metric Exporter")
    parser.add_argument("--trial", type=int, default=1, help="Trial index")
    parser.add_argument("--fitness-thresh", type=float, default=0.70, help="Fitness threshold for convergence")
    parser.add_argument("--output-dir", default="experiment_results", help="Directory to store results")
    parser.add_argument("--map-pcd", default="maps/robocon2026_field.pcd", help="Static PCD map path")
    parser.add_argument("--idle-timeout", type=float, default=5.0, help="Idle timeout in seconds after last message")
    parser.add_argument("--max-duration", type=float, default=180.0, help="Max recording duration in seconds")
    args, unknown = parser.parse_known_args()

    rclpy.init()
    tracker = ExperimentTracker(
        output_dir=args.output_dir,
        trial=args.trial,
        fitness_thresh=args.fitness_thresh,
        map_pcd_path=args.map_pcd
    )

    exit_code = 1
    start = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(tracker, timeout_sec=0.2)
            elapsed = time.time() - start
            since_last = time.time() - tracker.last_msg_time

            if elapsed > args.max_duration:
                tracker.get_logger().info(f"Max duration ({args.max_duration}s) reached.")
                break
            if len(tracker.trajectory) > 50 and since_last > args.idle_timeout:
                tracker.get_logger().info(f"Rosbag playback finished (idle for {since_last:.1f}s). Finalizing...")
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            exit_code = tracker.save_results()
        except Exception:
            exit_code = 1
        tracker.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
