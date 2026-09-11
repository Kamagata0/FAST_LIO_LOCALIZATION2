#!/usr/bin/env python3

import os
import sys
import time
import argparse
import subprocess


def main():
    parser = argparse.ArgumentParser(description="Permission & Trigger Controller for Night Localization Pipeline")
    parser.add_argument("--run", action="store_true", help="Start immediately without interactive prompt")
    parser.add_argument("--extreme", action="store_true", default=True, help="Use Extreme Multi-Trial pipeline (default: True)")
    parser.add_argument("--delay", type=float, default=0.0, help="Delay in seconds before starting")
    parser.add_argument("--watch-file", type=str, default="", help="Watch for existence of trigger file to start")
    parser.add_argument("--rosbag", type=str, default="", help="Optional override for rosbag path")
    parser.add_argument("--init-x", type=float, default=1.6, help="Initial X estimate")
    parser.add_argument("--init-y", type=float, default=0.0, help="Initial Y estimate")
    parser.add_argument("--init-yaw", type=float, default=0.305, help="Initial Yaw estimate")
    parser.add_argument("--max-retries", type=int, default=300, help="Max retry attempts (default: 300)")
    parser.add_argument("--target-fitness", type=float, default=0.99, help="Target fitness score (default: 0.99)")
    parser.add_argument("--rviz", action="store_true", help="Launch RViz2 GUI")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    pipeline_sh = os.path.join(script_dir, "align_pipeline_extreme.sh" if args.extreme else "align_pipeline.sh")

    print("======================================================================")
    print("  MID-360 NIGHT AUTOMATIC LOCALIZATION PIPELINE CONTROLLER            ")
    print("======================================================================")
    print(f"Pipeline Script    : {os.path.basename(pipeline_sh)}")
    print(f"Max Retries        : {args.max_retries}")
    print(f"Target Fitness     : {args.target_fitness}")
    print(f"Target Initial Pose: x={args.init_x}, y={args.init_y}, yaw={args.init_yaw}")
    print(f"Output Mode        : first/ and best/ only")

    if args.watch_file:
        print(f"Waiting for trigger file: {args.watch_file} ...")
        while not os.path.exists(args.watch_file):
            time.sleep(1.0)
        print("Trigger file detected! Proceeding...")
    elif args.delay > 0:
        print(f"Scheduled start in {args.delay:.0f} seconds ({args.delay/60:.1f} minutes)...")
        time.sleep(args.delay)
    elif not args.run:
        print("\nReady for overnight experiment.")
        try:
            input("Press [Enter] to grant permission and start pipeline (or Ctrl+C to cancel): ")
        except (KeyboardInterrupt, EOFError):
            print("\nAborted.")
            sys.exit(0)

    print("Permission confirmed. Launching pipeline ...")

    env = os.environ.copy()
    if args.rosbag:
        env["ROSBAG_PATH"] = args.rosbag
    env["INIT_X"] = str(args.init_x)
    env["INIT_Y"] = str(args.init_y)
    env["INIT_YAW"] = str(args.init_yaw)
    env["MAX_RETRIES"] = str(args.max_retries)
    env["TARGET_FITNESS"] = str(args.target_fitness)
    env["RVIZ"] = "true" if args.rviz else "false"

    try:
        subprocess.run(["bash", pipeline_sh], env=env)
    except KeyboardInterrupt:
        print("\nPipeline interrupted by user.")


if __name__ == "__main__":
    main()
