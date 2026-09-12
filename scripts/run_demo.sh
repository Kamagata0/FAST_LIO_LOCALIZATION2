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

DEFAULT_BAG="/mnt/c/Users/akeer.AKERU/Downloads/09-02_not-game-20260902T141825Z-1-001/09-02_not-game"
ROSBAG_PATH="${1:-${ROSBAG_PATH:-${DEFAULT_BAG}}}"

if [ ! -e "${ROSBAG_PATH}" ]; then
    echo "============================================================"
    echo " エラー: rosbag のパスが見つかりません: '${ROSBAG_PATH}'"
    echo " 使用方法: $0 [/path/to/rosbag] [lidar_mode (livox/isaac)]"
    echo "============================================================"
    exit 1
fi

LIDAR_MODE="${2:-livox}"
MAP_PATH="${MAP_PATH:-${WORKSPACE_DIR}/maps/robocon2026_field.pcd}"

echo "============================================================"
echo " Starting FAST-LIO Localization with RViz2 & 2D Cyber HUD..."
echo " Opponent Robot & Target Tracking: ENABLED"
echo " Initial Pose: x=-2.46, y=-3.85, yaw=178.00° (Zero Kill, Clearance > 23cm, Max X <= -0.15m)"
echo " Rosbag Path : ${ROSBAG_PATH}"
echo "============================================================"

# Kill old processes
pkill -9 -f "fastlio_mapping" 2>/dev/null || true
pkill -9 -f "global_localization.py" 2>/dev/null || true
pkill -9 -f "transform_fusion.py" 2>/dev/null || true
pkill -9 -f "robot_dashboard_node.py" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 1

USE_RVIZ="${USE_RVIZ:-false}"

# Launch FAST-LIO localization nodes + 2D Cyber HUD Dashboard (RViz: Disabled for Ultra Low Latency)
echo "1. Launching Localization (2D Cyber HUD | RViz: ${USE_RVIZ})..."
ros2 launch "${WORKSPACE_DIR}/launch/localization.launch.py" \
    use_sim_time:=true \
    lidar_mode:="${LIDAR_MODE}" \
    rviz:="${USE_RVIZ}" \
    initial_pose_x:="-2.46" \
    initial_pose_y:="-3.85" \
    initial_pose_yaw:="3.106686" \
    enable_auto_snap:="true" \
    enable_safety_kill:="false" \
    map:="${MAP_PATH}" &
LAUNCH_PID=$!

python3 "${WORKSPACE_DIR}/fast_lio_localization/robot_dashboard_node.py" \
    --ros-args -p use_sim_time:=true -p show_window:=true &
DASHBOARD_PID=$!

sleep 3

# Play rosbag
echo "2. Playing Rosbag with clock..."
ros2 bag play "${ROSBAG_PATH}" --clock

echo "Rosbag playback finished."
