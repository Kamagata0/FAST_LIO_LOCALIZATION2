#!/usr/bin/env bash

# ==============================================================================
# MID-360 夜間自動マップマッチング・完全一致パイプライン (Extreme Edition)
# 1秒限定吸着＆永久固定 + 最大数百回自動試行 + 初回(first) & ベスト(best) 厳選保存
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
MAX_RETRIES="${MAX_RETRIES:-300}"
TARGET_FITNESS="${TARGET_FITNESS:-0.99}"
LOCK_TIME_SEC="${LOCK_TIME_SEC:-1.0}"

echo "======================================================================"
echo " MID-360 夜間自動マップマッチング・完全一致パイプライン (Extreme)     "
echo " (Max Retries: ${MAX_RETRIES}, Target Fitness: ${TARGET_FITNESS}, Lock: ${LOCK_TIME_SEC}s) "
echo "======================================================================"
echo "Rosbag Path  : ${ROSBAG_PATH}"
echo "Map Path     : ${MAP_PATH}"
echo "Base Pose    : x=${BASE_X}, y=${BASE_Y}, z=${BASE_Z}, yaw=${BASE_YAW}"
echo "Output Dir   : ${OUTPUT_DIR} (first/ & best/ only)"
echo "======================================================================"

mkdir -p "${OUTPUT_DIR}/first"
mkdir -p "${OUTPUT_DIR}/best"
rm -f "${OUTPUT_DIR}/.best_fitness"

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
    pkill -9 -f "pose_lock_controller.py" 2>/dev/null || true
    pkill -9 -f "match_evaluator.py" 2>/dev/null || true
    pkill -9 -f "ros2 bag play" 2>/dev/null || true
    pkill -9 -f "publish_initial_pose.py" 2>/dev/null || true
    sleep 1
}

trap kill_subprocesses EXIT INT TERM

EXPERIMENT_PASSED=false

# 試行ループ（最大 MAX_RETRIES 回）
for trial in $(seq 1 $MAX_RETRIES); do
    echo ""
    echo "======================================================================"
    echo " >>> STARTING TRIAL ${trial} / ${MAX_RETRIES} <<<"
    echo "======================================================================"

    # パラメータ生成（Trial 1はベースライン、以降は微小オフセットを格子状・スパイラル状に探索）
    if [ "$trial" -eq 1 ]; then
        dx=0.0
        dy=0.0
        dyaw=0.0
        map_voxel=0.4
        scan_voxel=0.1
    else
        # 螺旋探索によるパラメータ生成
        idx=$((trial - 1))
        layer=$(( (idx - 1) / 8 + 1 ))
        pos_in_layer=$(( (idx - 1) % 8 ))
        radius=$(awk "BEGIN {print $layer * 0.10}")
        angle_rad=$(awk "BEGIN {print $pos_in_layer * 0.785398}") # 45度刻み
        
        dx=$(awk "BEGIN {print $radius * cos($angle_rad)}")
        dy=$(awk "BEGIN {print $radius * sin($angle_rad)}")
        dyaw=$(awk "BEGIN {print (($idx % 12) - 6) * 0.05}") # -0.30 ~ +0.30 rad
        map_voxel=0.3
        scan_voxel=0.08
    fi

    CUR_X=$(awk "BEGIN {print $BASE_X + $dx}")
    CUR_Y=$(awk "BEGIN {print $BASE_Y + $dy}")
    CUR_YAW=$(awk "BEGIN {print $BASE_YAW + $dyaw}")
    PARAMS_DESC="dx=${dx}, dy=${dy}, dyaw=${dyaw}, map_vox=${map_voxel}, scan_vox=${scan_voxel}"

    echo "[Trial ${trial}] Target Pose: x=${CUR_X}, y=${CUR_Y}, yaw=${CUR_YAW}"
    echo "[Trial ${trial}] Parameters : ${PARAMS_DESC}"

    # 1. 1秒ワープ防止コントローラー起動
    python3 "${SCRIPT_DIR}/pose_lock_controller.py" \
        --ros-args -p lock_time_sec:="${LOCK_TIME_SEC}" -p use_sim_time:=true &
    LOCK_PID=$!

    # 2. FAST-LIO & Localization ノード起動
    echo "[Trial ${trial}] 1/4 Launching Localization Nodes..."
    ros2 launch fast_lio_localization localization.launch.py \
        use_sim_time:=true \
        lidar_mode:=livox \
        rviz:="${RVIZ}" \
        map:="${MAP_PATH}" &
    LAUNCH_PID=$!

    sleep 3

    # 3. Match Evaluator 起動
    echo "[Trial ${trial}] 2/4 Starting Match Evaluator..."
    python3 "${SCRIPT_DIR}/match_evaluator.py" \
        --trial "${trial}" \
        --target-fitness "${TARGET_FITNESS}" \
        --output-dir "${OUTPUT_DIR}" \
        --map-pcd "${MAP_PATH}" \
        --params-desc "${PARAMS_DESC}" \
        --max-duration "${MAX_DURATION}" \
        --ros-args -p use_sim_time:=true &
    EVALUATOR_PID=$!

    # 4. rosbag 再生開始
    echo "[Trial ${trial}] 3/4 Starting Rosbag Playback..."
    ros2 bag play "${ROSBAG_PATH}" --clock &
    BAG_PID=$!

    sleep 1

    # 5. 初期1秒の窓内で初期姿勢を送信
    python3 "${WORKSPACE_DIR}/fast_lio_localization/publish_initial_pose.py" \
        "${CUR_X}" "${CUR_Y}" "${BASE_Z}" "${CUR_YAW}" 0.0 0.0 --repeat 3 \
        --ros-args -p use_sim_time:=true 2>/dev/null || true

    # bag 再生完了を待機
    wait "$BAG_PID" 2>/dev/null || true
    echo "[Trial ${trial}] Rosbag playback finished."

    sleep 2

    # Evaluator の結果判定
    EVAL_EXIT=0
    wait "$EVALUATOR_PID" 2>/dev/null || EVAL_EXIT=$?

    # プロセス終了とクリーンアップ
    kill_subprocesses

    if [ "$EVAL_EXIT" -eq 0 ]; then
        echo ""
        echo "======================================================================"
        echo " ★★★ [TARGET ACHIEVED] TRIAL ${trial} HIT TARGET FITNESS (${TARGET_FITNESS}+) ★★★"
        echo "======================================================================"
        EXPERIMENT_PASSED=true
        break
    else
        echo "[Trial ${trial}] Evaluation completed. Continuing search for optimal match..."
    fi
done

echo ""
echo "======================================================================"
if [ "$EXPERIMENT_PASSED" = true ]; then
    echo " EXTREME LOCALIZATION PIPELINE COMPLETED SUCCESSFULLY!                "
else
    echo " COMPLETED ALL ${MAX_RETRIES} SEARCH TRIALS (COMPLETED EXHAUSTIVE SWEEP) "
fi
echo " Artifacts saved strictly to:                                         "
echo "   - First Trial : ${OUTPUT_DIR}/first/                               "
echo "   - Best Trial  : ${OUTPUT_DIR}/best/                                "
echo "======================================================================"
