#!/usr/bin/env bash
# ==============================================================================
# FAST-LIO Localization - Smooth & Perfectly Aligned RViz2 Demo
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

source /opt/ros/humble/setup.bash 2>/dev/null || true
source "${WORKSPACE_DIR}/../../install/setup.bash" 2>/dev/null || true

export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"

ROSBAG_PATH="/mnt/c/Users/akeer.AKERU/Downloads/09-02_not-game-20260902T141825Z-1-001/09-02_not-game"
MAP_PATH="${WORKSPACE_DIR}/maps/robocon2026_field.pcd"

echo "============================================================"
echo " Starting FAST-LIO Localization with RViz2 GUI & Rosbag..."
echo " Target (100% Inside Field): x=-1.90, y=3.65, yaw=-5.00°"
echo "============================================================"

# Kill old processes
pkill -9 -f "fastlio_mapping" 2>/dev/null || true
pkill -9 -f "global_localization.py" 2>/dev/null || true
pkill -9 -f "transform_fusion.py" 2>/dev/null || true
pkill -9 -f "field_localization_node.py" 2>/dev/null || true
pkill -9 -f "robot_dashboard_node.py" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 1

# Launch FAST-LIO localization nodes + RViz2 + 2D Cyber HUD Dashboard
echo "1. Launching FAST-LIO Localization (3D RViz2 + 2D Cyber HUD Dashboard)..."
ros2 launch fast_lio_localization localization.launch.py \
    use_sim_time:=true \
    lidar_mode:=livox \
    rviz:=true \
    map:="${MAP_PATH}" &
LAUNCH_PID=$!

python3 "${WORKSPACE_DIR}/fast_lio_localization/robot_dashboard_node.py" \
    --ros-args -p use_sim_time:=true -p show_window:=true &
DASHBOARD_PID=$!

sleep 4

# Publish confirmed initial pose (x=-1.90, y=3.65, yaw=-5.00 deg)
echo "2. Applying Initial Pose: x=-1.90, y=3.65, yaw=-5.00° (-0.0873 rad)..."
python3 "${WORKSPACE_DIR}/fast_lio_localization/publish_initial_pose.py" \
    -1.90 3.65 0.0 -0.0873 0.0 0.0 --repeat 3 2>/dev/null || true

# Play rosbag
echo "3. Playing Rosbag with clock..."
ros2 bag play "${ROSBAG_PATH}" --clock

echo "Rosbag playback finished."
