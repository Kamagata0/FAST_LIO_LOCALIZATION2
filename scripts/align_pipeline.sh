#!/usr/bin/env bash

# ==============================================================================
# Night Automatic Localization & Alignment Pipeline (Multi-Trial Retry Edition)
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Configuration defaults
ROSBAG_PATH="${ROSBAG_PATH:-/mnt/c/Users/akeer.AKERU/Downloads/09-02_not-game-20260902T141825Z-1-001/09-02_not-game}"
MAP_PATH="${MAP_PATH:-${WORKSPACE_DIR}/maps/robocon2026_field.pcd}"
BASE_X="${INIT_X:--5.0}"
BASE_Y="${INIT_Y:-4.5}"
BASE_Z="${INIT_Z:-0.0}"
BASE_YAW="${INIT_YAW:-0.0}"
OUTPUT_DIR="${OUTPUT_DIR:-${WORKSPACE_DIR}/experiment_results}"
RVIZ="${RVIZ:-false}"
MAX_DURATION="${MAX_DURATION:-180}"
MAX_RETRIES="${MAX_RETRIES:-5}"
FITNESS_THRESH="${FITNESS_THRESH:-0.60}"

echo "====================================================="
echo " NIGHT AUTOMATIC LOCALIZATION & ALIGNMENT PIPELINE  "
echo " (Max Retries: ${MAX_RETRIES}, Threshold: ${FITNESS_THRESH}) "
echo "====================================================="
echo "Rosbag Path  : ${ROSBAG_PATH}"
echo "Map Path     : ${MAP_PATH}"
echo "Base Pose    : x=${BASE_X}, y=${BASE_Y}, z=${BASE_Z}, yaw=${BASE_YAW}"
echo "Output Dir   : ${OUTPUT_DIR}"
echo "====================================================="

mkdir -p "${OUTPUT_DIR}"

# Source ROS2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
fi
if [ -f "${WORKSPACE_DIR}/../../install/setup.bash" ]; then
    source "${WORKSPACE_DIR}/../../install/setup.bash"
fi

kill_subprocesses() {
    pkill -9 -f "fastlio_mapping" 2>/dev/null || true
    pkill -9 -f "global_localization.py" 2>/dev/null || true
    pkill -9 -f "transform_fusion.py" 2>/dev/null || true
    pkill -9 -f "experiment_tracker.py" 2>/dev/null || true
    pkill -9 -f "ros2 bag play" 2>/dev/null || true
    pkill -9 -f "publish_initial_pose.py" 2>/dev/null || true
    sleep 1
}

trap kill_subprocesses EXIT INT TERM

# Offset candidates for retry attempts: (dx, dy, dyaw)
OFFSETS=(
    "0.0 0.0 0.0"
    "0.2 -0.2 0.05"
    "-0.2 0.2 -0.05"
    "0.3 0.3 0.10"
    "-0.3 -0.3 -0.10"
)

EXPERIMENT_PASSED=false

for trial in $(seq 1 $MAX_RETRIES); do
    echo ""
    echo "====================================================="
    echo " >>> STARTING TRIAL ${trial} / ${MAX_RETRIES} <<<"
    echo "====================================================="

    # Select offset for this trial
    offset_idx=$((trial - 1))
    offset="${OFFSETS[$offset_idx]}"
    read -r dx dy dyaw <<< "$offset"

    CUR_X=$(awk "BEGIN {print $BASE_X + $dx}")
    CUR_Y=$(awk "BEGIN {print $BASE_Y + $dy}")
    CUR_YAW=$(awk "BEGIN {print $BASE_YAW + $dyaw}")

    echo "[Trial ${trial}] Target Initial Pose: x=${CUR_X}, y=${CUR_Y}, yaw=${CUR_YAW}"

    # 1. Launch FAST-LIO & Localization Nodes
    echo "[Trial ${trial}] 1/4 Launching Localization Nodes..."
    ros2 launch fast_lio_localization localization.launch.py \
        use_sim_time:=true \
        lidar_mode:=livox \
        rviz:="${RVIZ}" \
        map:="${MAP_PATH}" &
    LAUNCH_PID=$!

    sleep 3

    # 2. Start Experiment Tracker
    echo "[Trial ${trial}] 2/4 Starting Experiment Tracker..."
    python3 "${SCRIPT_DIR}/experiment_tracker.py" \
        --trial "${trial}" \
        --fitness-thresh "${FITNESS_THRESH}" \
        --output-dir "${OUTPUT_DIR}" \
        --map-pcd "${MAP_PATH}" \
        --max-duration "${MAX_DURATION}" \
        --ros-args -p use_sim_time:=true &
    TRACKER_PID=$!

    # 3. Start Rosbag Playback
    echo "[Trial ${trial}] 3/4 Starting Rosbag Playback..."
    ros2 bag play "${ROSBAG_PATH}" --clock &
    BAG_PID=$!

    sleep 2

    # 4. Automatically publish initial pose for 1-shot snapping
    echo "[Trial ${trial}] 4/4 Publishing Initial Pose..."
    python3 "${WORKSPACE_DIR}/fast_lio_localization/publish_initial_pose.py" \
        "${CUR_X}" "${CUR_Y}" "${BASE_Z}" "${CUR_YAW}" 0.0 0.0 --repeat 5 \
        --ros-args -p use_sim_time:=true 2>/dev/null || true

    # Wait for bag playback to finish
    wait "$BAG_PID" 2>/dev/null || true
    echo "[Trial ${trial}] Rosbag playback finished."

    sleep 2

    # Check tracker status and exit code safely
    wait "$TRACKER_PID" 2>/dev/null
    TRACKER_EXIT=$?

    # Kill trial processes cleanly before evaluating
    kill_subprocesses

    if [ "$TRACKER_EXIT" -eq 0 ]; then
        echo ""
        echo "====================================================="
        echo " [SUCCESS] TRIAL ${trial} PASSED CONVERGENCE CRITERIA!"
        echo "====================================================="
        EXPERIMENT_PASSED=true
        break
    else
        echo ""
        echo "====================================================="
        echo " [WARN] Trial ${trial} did not achieve required fitness."
        if [ "$trial" -lt "$MAX_RETRIES" ]; then
            echo " Retrying with adjusted initial pose candidate..."
            sleep 2
        fi
        echo "====================================================="
    fi
done

echo ""
echo "====================================================="
if [ "$EXPERIMENT_PASSED" = true ]; then
    echo " OVERNIGHT EXPERIMENT COMPLETED SUCCESSFULLY!        "
else
    echo " COMPLETED ALL ${MAX_RETRIES} TRIALS (SAFETY SHUTDOWN) "
fi
echo " Results and screenshots saved in: ${OUTPUT_DIR}     "
echo "====================================================="
