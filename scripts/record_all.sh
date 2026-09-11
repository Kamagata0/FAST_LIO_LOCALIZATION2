#!/usr/bin/env bash
# ==============================================================================
# Robocon 2026 - One-Command Full Telemetry & Sensor ROS2 Bag Recorder
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Output folder
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
TAG="${1:-run}"
BAG_DIR="${WORKSPACE_DIR}/bags/bag_${TIMESTAMP}_${TAG}"
mkdir -p "${WORKSPACE_DIR}/bags"

echo "=================================================================="
echo " 🔴 ROBOCON 2026 ROS2 DATA RECORDER (ONE-CLICK LOGGER)"
echo "=================================================================="
echo " Output Destination: ${BAG_DIR}"
echo " Timestamp          : ${TIMESTAMP}"
echo " Session Tag        : ${TAG}"
echo "------------------------------------------------------------------"
echo " Recording Topics:"
echo "   - /livox/lidar            (Raw PointCloud2 / CustomMsg)"
echo "   - /robot_pose             (Global Pose: X, Y, Yaw)"
echo "   - /target_relative        (Target Relative Distance/Angle)"
echo "   - /robot_status           (Belt Speeds, Cylinder Status)"
echo "   - /robot_dashboard/image  (2D Realtime HUD Rendered Image)"
echo "   - /tf, /tf_static         (Coordinate Transforms)"
echo "=================================================================="
echo " Press [Ctrl+C] to stop recording safely."
echo ""

# Source ROS 2 environment
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "${WORKSPACE_DIR}/../../install/setup.bash" 2>/dev/null || true

# Execute ros2 bag record with sqlite3 / mcap storage
ros2 bag record \
    -o "${BAG_DIR}" \
    /livox/lidar \
    /robot_pose \
    /target_relative \
    /robot_status \
    /robot_dashboard/image \
    /tf \
    /tf_static

echo ""
echo "=================================================================="
echo " ✅ Recording completed successfully!"
echo " Saved to: ${BAG_DIR}"
echo "=================================================================="

