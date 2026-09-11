std_msgs/Header header

# ベルト直動
float64 belt_target_speed
float64 belt_actual_speed

# エアシリンダ
bool cylinder_deployed

# ターゲット相対情報
bool target_detected
float64 target_distance
float64 target_angle_rad
float64 target_angle_deg

# 相手ロボット（敵機）追跡情報
bool opponent_detected
float64 opponent_distance
float64 opponent_x
float64 opponent_y
float64 opponent_angle_deg
