#!/usr/bin/env bash
# ==============================================================================
# Robocon 2026 - Ultra-Fast Zero-Odometry Localization & 2D Dashboard Demo
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

source /opt/ros/humble/setup.bash 2>/dev/null || true
source "${WORKSPACE_DIR}/../../install/setup.bash" 2>/dev/null || true

export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"

ROSBAG_PATH="${1:-${ROSBAG_PATH}}"

if [ -z "${ROSBAG_PATH}" ] || [ ! -e "${ROSBAG_PATH}" ]; then
    echo "=================================================================="
    echo " エラー: rosbag のパスが指定されていないか、存在しません！"
    echo " 指定されたパス: '${ROSBAG_PATH}'"
    echo "------------------------------------------------------------------"
    echo " 使用方法: $0 /path/to/rosbag"
    echo " 例:       $0 ~/rosbag2_2026_09_08-16_17_47"
    echo "=================================================================="
    exit 1
fi

echo "=================================================================="
echo " 🚀 ROBOCON 2026 ULTRA-FAST FIELD LOCALIZATION & 2D HUD DEMO"
echo " Zero-Odometry | Zero-Initial-Pose | 30fps Real-time Telemetry"
echo "=================================================================="

# Kill any existing processes
pkill -9 -f "fastlio_mapping" 2>/dev/null || true
pkill -9 -f "transform_fusion.py" 2>/dev/null || true
pkill -9 -f "field_localization_node.py" 2>/dev/null || true
pkill -9 -f "robot_dashboard_node.py" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 1

# Detect LiDAR mode from rosbag metadata
LIDAR_MODE="${2}"
if [ -z "${LIDAR_MODE}" ]; then
    if [ -f "${ROSBAG_PATH}/metadata.yaml" ] && grep -q "sensor_msgs/msg/PointCloud2" "${ROSBAG_PATH}/metadata.yaml" 2>/dev/null; then
        LIDAR_MODE="isaac"
        echo "🔍 rosbag の LiDAR 型を検出: PointCloud2 -> lidar_mode:=isaac を設定"
    else
        LIDAR_MODE="livox"
        echo "🔍 rosbag の LiDAR 型を検出: CustomMsg -> lidar_mode:=livox を設定"
    fi
fi

# Launch specialized nodes & 2D HUD + RViz2
echo "1. Launching Robocon System (Field Localization + 2D HUD Dashboard + RViz2 3D)..."
ros2 launch fast_lio_localization robocon_system.launch.py \
    use_sim_time:=true \
    lidar_mode:="${LIDAR_MODE}" \
    dashboard:=true \
    rviz:=true &
LAUNCH_PID=$!

sleep 3

# Play bag
echo "2. Playing Rosbag..."
ros2 bag play "${ROSBAG_PATH}" --clock

echo "Playback finished."

