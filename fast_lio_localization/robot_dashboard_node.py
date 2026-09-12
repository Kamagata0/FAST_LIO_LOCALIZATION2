#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robocon 2026 2D Real-time HUD Dashboard & Mechanism Data Server Node
- Renders top-down 2D field map with robot pose, belt/cylinder orientation, and target info.
- Overlays real-time numeric telemetry (Belt target/actual speed, cylinder state, distance, angles).
- Publishes rendered frame as sensor_msgs/Image and displays OpenCV GUI window.
"""

import math
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import transforms3d.euler as te

try:
    from fast_lio_localization.msg import RobotStatus
    HAVE_STATUS_MSG = True
except ImportError:
    HAVE_STATUS_MSG = False


class RobotDashboardNode(Node):
    def __init__(self):
        super().__init__("robot_dashboard_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("show_window", True),
                ("publish_image", True),
                ("field_width_m", 12.0),
                ("field_height_m", 11.0),
                ("image_width", 960),
                ("image_height", 720),
            ]
        )

        self.show_window = self.get_parameter("show_window").value
        self.publish_image = self.get_parameter("publish_image").value
        self.img_w = self.get_parameter("image_width").value
        self.img_h = self.get_parameter("image_height").value

        # Robot & Mechanism State
        self.robot_x = -1.90
        self.robot_y = 3.65
        self.robot_yaw = -0.0873  # rad
        self.has_robot_pose = False

        # Target State
        self.target_rel_x = 0.0
        self.target_rel_y = 0.0
        self.target_dist = 0.0
        self.target_angle_deg = 0.0
        self.has_target = False

        # Opponent Robot State
        self.opp_map_x = 0.0
        self.opp_map_y = 0.0
        self.opp_rel_x = 0.0
        self.opp_rel_y = 0.0
        self.opp_dist = 0.0
        self.opp_angle_deg = 0.0
        self.has_opponent = False
        self.last_opp_time = 0.0

        # Mechanism Telemetry
        self.belt_target_speed = 1.50  # m/s
        self.belt_actual_speed = 1.48  # m/s
        self.cylinder_deployed = True

        self.path_history = []
        self.max_path_len = 500

        # Publishers & Subscribers
        if self.publish_image:
            self.pub_image = self.create_publisher(Image, "/robot_dashboard/image", 10)

        # Primary robot pose source: /localization (published by transform_fusion)
        self.sub_loc_odom = self.create_subscription(Odometry, "/localization", self.cb_loc_odom, 10)
        self.sub_target = self.create_subscription(PointStamped, "/target_relative", self.cb_target_rel, 10)
        self.sub_opp_pose = self.create_subscription(PoseStamped, "/opponent_pose", self.cb_opp_pose, 10)
        self.sub_opp_rel = self.create_subscription(PointStamped, "/opponent_relative", self.cb_opp_rel, 10)
        if HAVE_STATUS_MSG:
            self.sub_status = self.create_subscription(RobotStatus, "/robot_status", self.cb_robot_status, 10)

        # Timer for 30 FPS rendering
        self.timer = self.create_timer(1.0 / 30.0, self.render_dashboard)

        self.get_logger().info("🎨 [Robot Dashboard] 2D Real-time HUD Visualizer Started!")

    def cb_opp_pose(self, msg: PoseStamped):
        self.opp_map_x = msg.pose.position.x
        self.opp_map_y = msg.pose.position.y
        self.has_opponent = True
        self.last_opp_time = time.time()

    def cb_opp_rel(self, msg: PointStamped):
        self.opp_rel_x = msg.point.x
        self.opp_rel_y = msg.point.y
        self.opp_dist = math.sqrt(self.opp_rel_x ** 2 + self.opp_rel_y ** 2)
        self.opp_angle_deg = math.degrees(math.atan2(self.opp_rel_y, self.opp_rel_x))
        self.has_opponent = True
        self.last_opp_time = time.time()

    def _record_path_pt(self, px, py):
        if not self.path_history or math.hypot(px - self.path_history[-1][0], py - self.path_history[-1][1]) > 0.05:
            self.path_history.append((px, py))
            if len(self.path_history) > self.max_path_len:
                self.path_history.pop(0)

    def cb_loc_odom(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self._record_path_pt(self.robot_x, self.robot_y)
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = te.quat2euler([q.w, q.x, q.y, q.z], axes="sxyz")
        self.has_robot_pose = True

    def cb_target_rel(self, msg: PointStamped):
        self.target_rel_x = msg.point.x
        self.target_rel_y = msg.point.y
        self.target_dist = math.sqrt(self.target_rel_x ** 2 + self.target_rel_y ** 2)
        self.target_angle_deg = math.degrees(math.atan2(self.target_rel_y, self.target_rel_x))
        self.has_target = True

    def cb_robot_status(self, msg):
        self.belt_target_speed = msg.belt_target_speed
        self.belt_actual_speed = msg.belt_actual_speed
        self.cylinder_deployed = msg.cylinder_deployed
        if msg.target_detected:
            self.target_dist = msg.target_distance
            self.target_angle_deg = msg.target_angle_deg
            self.has_target = True

    def map_to_pixel(self, mx: float, my: float):
        """Converts metric field coordinates (X: -6 to 6, Y: -5 to 6) to image pixels."""
        field_min_x, field_max_x = -6.0, 6.0
        field_min_y, field_max_y = -5.0, 6.2
        
        # Left margin for field map, right side reserved for HUD cards
        map_area_w = int(self.img_w * 0.65)
        map_area_h = self.img_h - 40

        px = int(20 + (mx - field_min_x) / (field_max_x - field_min_x) * (map_area_w - 40))
        py = int(self.img_h - 20 - (my - field_min_y) / (field_max_y - field_min_y) * (map_area_h - 40))
        return px, py

    def render_dashboard(self):
        # 1. Dark Cyber Theme Background
        canvas = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
        canvas[:] = (18, 22, 28)  # Deep dark navy-gray

        # 2. Draw 2D Field Arena
        # Outer boundary
        tl = self.map_to_pixel(-5.85, 5.95)
        br = self.map_to_pixel(5.85, -4.85)
        cv2.rectangle(canvas, tl, br, (180, 190, 200), 2)

        # Center Yellow Line Divider (黄色の境界線: X = 0.0)
        c_top = self.map_to_pixel(0.0, 5.95)
        c_bot = self.map_to_pixel(0.0, -4.85)
        cv2.line(canvas, c_top, c_bot, (0, 230, 255), 3, cv2.LINE_AA)
        
        # Court Area Labels
        own_lbl_p = self.map_to_pixel(-3.0, 5.60)
        opp_lbl_p = self.map_to_pixel(2.8, 5.60)
        cv2.putText(canvas, "[ OWN COURT (X <= 0) ]", (own_lbl_p[0] - 60, own_lbl_p[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 180), 1, cv2.LINE_AA)
        cv2.putText(canvas, "[ OPPONENT AREA (FORBIDDEN) ]", (opp_lbl_p[0] - 80, opp_lbl_p[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 100, 255), 1, cv2.LINE_AA)

        c_left = self.map_to_pixel(-5.85, 0.55)
        c_right = self.map_to_pixel(5.85, 0.55)
        cv2.line(canvas, c_left, c_right, (60, 70, 85), 1, cv2.LINE_AA)

        # Center Divider / Slope partitions (中央分離帯・スロープ)
        div_bot_tl = self.map_to_pixel(-0.15, -0.15)
        div_bot_br = self.map_to_pixel(0.15, -4.60)
        cv2.rectangle(canvas, div_bot_tl, div_bot_br, (50, 60, 75), -1)
        div_top_tl = self.map_to_pixel(-0.15, 5.65)
        div_top_br = self.map_to_pixel(0.15, 1.25)
        cv2.rectangle(canvas, div_top_tl, div_top_br, (50, 60, 75), -1)

        # Center Teaching Podium (教壇)
        pod_tl = self.map_to_pixel(-0.6, 1.15)
        pod_br = self.map_to_pixel(0.6, -0.05)
        cv2.rectangle(canvas, pod_tl, pod_br, (80, 100, 130), -1)
        cv2.rectangle(canvas, pod_tl, pod_br, (120, 150, 190), 1)
        cv2.putText(canvas, "PODIUM", (pod_tl[0] + 5, (pod_tl[1] + pod_br[1]) // 2 + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (220, 230, 240), 1, cv2.LINE_AA)

        # Flags (旗・支柱: 高3000mm, 土台 W390xD390xH180, 中心 X=550mm, Y=3025mm)
        flag_positions = [(-3.025, -0.12), (3.025, -0.12)]
        for fx, fy in flag_positions:
            f_px, f_py = self.map_to_pixel(fx, fy)
            cv2.circle(canvas, (f_px, f_py), 14, (0, 140, 200), 1, cv2.LINE_AA)
            cv2.circle(canvas, (f_px, f_py), 5, (255, 220, 0), -1, cv2.LINE_AA)
            tri_pts = np.array([[f_px, f_py - 2], [f_px + 14, f_py - 7], [f_px, f_py - 12]], np.int32)
            cv2.fillPoly(canvas, [tri_pts], (0, 100, 255), cv2.LINE_AA)
            cv2.putText(canvas, "FLAG", (f_px - 14, f_py + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, (200, 240, 255), 1, cv2.LINE_AA)

        # 固定バケツ①②③ (Robocon 2026 Official Spec)
        bucket_specs = [
            ("B1", -0.87, 0.00, "B1"), ("B1", 0.87, 0.00, "B1"),
            ("B2", -1.48, -1.82, "B2(H600)"), ("B2", 1.48, -1.82, "B2(H600)"),
            ("B3", -1.48, 1.82, "B3(H300)"), ("B3", 1.48, 1.82, "B3(H300)"),
        ]
        for _, bx, by, lbl in bucket_specs:
            b_px, b_py = self.map_to_pixel(bx, by)
            cv2.circle(canvas, (b_px, b_py), 15, (0, 90, 140), 1, cv2.LINE_AA)
            cv2.circle(canvas, (b_px, b_py), 7, (40, 70, 95), -1, cv2.LINE_AA)
            cv2.circle(canvas, (b_px, b_py), 8, (0, 190, 255), 2, cv2.LINE_AA)
            cv2.putText(canvas, lbl, (b_px - 10, b_py + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.28, (0, 220, 255), 1, cv2.LINE_AA)

        # 椅子 (Chair - PCD中心: X=-5.155, Y=0.0)
        chair_positions = [(-5.155, 0.00), (5.155, 0.00)]
        for cx_m, cy_m in chair_positions:
            c_px, c_py = self.map_to_pixel(cx_m, cy_m)
            cv2.rectangle(canvas, (c_px - 8, c_py - 8), (c_px + 8, c_py + 8), (140, 100, 200), 2)
            cv2.putText(canvas, "CHAIR", (c_px - 14, c_py + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.28, (200, 180, 255), 1, cv2.LINE_AA)

        # 机 (Desks - PCD中心: Desk1: -3.86,-2.85; Desk2: -3.86,2.84; Desk3: -5.56,5.33; Desk4: -1.08,5.29)
        desk_positions = [
            (-3.860, -2.850), (-3.860, 2.840), (3.860, -2.850), (3.860, 2.840),
            (-5.560, 5.330), (5.560, 5.330), (-1.080, 5.290), (1.080, 5.290),
        ]
        for dx_m, dy_m in desk_positions:
            d_px, d_py = self.map_to_pixel(dx_m, dy_m)
            cv2.rectangle(canvas, (d_px - 10, d_py - 7), (d_px + 10, d_py + 7), (60, 180, 180), 2)
            cv2.putText(canvas, "DESK", (d_px - 12, d_py + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.26, (100, 230, 230), 1, cv2.LINE_AA)

        # 2b. Draw Trajectory Trail History (自陣内の無衝突走行履歴)
        if len(self.path_history) >= 2:
            pixel_pts = [self.map_to_pixel(hx, hy) for hx, hy in self.path_history]
            for i in range(len(pixel_pts) - 1):
                cv2.line(canvas, pixel_pts[i], pixel_pts[i+1], (0, 255, 200), 2, cv2.LINE_AA)

        # 3. Draw Robot Icon & Heading
        r_px, r_py = self.map_to_pixel(self.robot_x, self.robot_y)
        robot_radius_px = 16

        # Heading vector
        head_len = 32
        head_x = int(r_px + head_len * math.cos(self.robot_yaw))
        head_y = int(r_py - head_len * math.sin(self.robot_yaw))

        # Body circle & orientation arrow
        cv2.circle(canvas, (r_px, r_py), robot_radius_px, (0, 230, 255), -1, cv2.LINE_AA)
        cv2.circle(canvas, (r_px, r_py), robot_radius_px + 2, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.arrowedLine(canvas, (r_px, r_py), (head_x, head_y), (0, 255, 128), 3, cv2.LINE_AA, tipLength=0.35)

        # Linear Belt / Cylinder Direction Pointer
        cyl_angle = self.robot_yaw
        cyl_len = 24
        cyl_end_x = int(r_px + cyl_len * math.cos(cyl_angle))
        cyl_end_y = int(r_py - cyl_len * math.sin(cyl_angle))
        cyl_color = (0, 128, 255) if self.cylinder_deployed else (120, 120, 120)
        cv2.line(canvas, (r_px, r_py), (cyl_end_x, cyl_end_y), cyl_color, 4, cv2.LINE_AA)

        # 4. Draw Target Object & Connecting Vector
        if self.has_target:
            # Map-frame target position
            c, s = math.cos(self.robot_yaw), math.sin(self.robot_yaw)
            tgt_map_x = self.robot_x + c * self.target_rel_x - s * self.target_rel_y
            tgt_map_y = self.robot_y + s * self.target_rel_x + c * self.target_rel_y
            t_px, t_py = self.map_to_pixel(tgt_map_x, tgt_map_y)

            # Target glow & marker
            cv2.circle(canvas, (t_px, t_py), 12, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.circle(canvas, (t_px, t_py), 16, (0, 165, 255), 2, cv2.LINE_AA)
            cv2.circle(canvas, (t_px, t_py), 10, (0, 255, 255), -1, cv2.LINE_AA)
            cv2.circle(canvas, (t_px, t_py), 14, (0, 200, 255), 2, cv2.LINE_AA)
            cv2.circle(canvas, (t_px, t_py), 16, (0, 200, 255), 2, cv2.LINE_AA)
            cv2.line(canvas, (r_px, r_py), (t_px, t_py), (0, 255, 255), 1, cv2.LINE_AA)

            # Target distance text
            mid_x = (r_px + t_px) // 2
            mid_y = (r_py + t_py) // 2 - 8
            cv2.putText(canvas, f"{self.target_dist:.2f}m ({self.target_angle_deg:+.1f}deg)",
                        (mid_x - 30, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(canvas, f"TGT {self.target_dist:.2f}m",
                        (mid_x - 25, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(canvas, f"TGT {self.target_dist:.2f}m ({self.target_angle_deg:+.1f}deg)",
                        (mid_x - 45, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)

        # 4b. Draw Opponent Robot (Enemy) Icon & Proximity Line
        if self.has_opponent and (time.time() - self.last_opp_time < 2.0):
            opp_px, opp_py = self.map_to_pixel(self.opp_map_x, self.opp_map_y)
            
            # Enemy square & pulsing warning marker
            half_s = 14
            cv2.rectangle(canvas, (opp_px - half_s, opp_py - half_s), (opp_px + half_s, opp_py + half_s), (0, 0, 255), -1)
            cv2.rectangle(canvas, (opp_px - half_s - 3, opp_py - half_s - 3), (opp_px + half_s + 3, opp_py + half_s + 3), (50, 50, 255), 2)
            cv2.putText(canvas, "ENEMY", (opp_px - 22, opp_py - half_s - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (50, 50, 255), 1, cv2.LINE_AA)

            # Proximity warning vector
            line_col = (0, 0, 255) if self.opp_dist < 2.5 else (100, 100, 200)
            cv2.line(canvas, (r_px, r_py), (opp_px, opp_py), line_col, 2, cv2.LINE_AA)
            mid_ox = (r_px + opp_px) // 2
            mid_oy = (r_py + opp_py) // 2 - 8
            cv2.putText(canvas, f"ENEMY {self.opp_dist:.2f}m",
                        (mid_ox - 35, mid_oy), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1, cv2.LINE_AA)

        # 5. Right-Hand HUD Telemetry Cards Panel
        panel_x = int(self.img_w * 0.65) + 10
        cv2.rectangle(canvas, (panel_x, 20), (self.img_w - 20, self.img_h - 20), (28, 34, 44), -1)
        cv2.rectangle(canvas, (panel_x, 20), (self.img_w - 20, self.img_h - 20), (70, 85, 110), 2)

        # Title Header
        cv2.putText(canvas, "ROBOCON 2026", (panel_x + 15, 55),
                    cv2.FONT_HERSHEY_DUPLEX, 0.72, (0, 230, 255), 2, cv2.LINE_AA)
        cv2.putText(canvas, "JETSON DATA SERVER", (panel_x + 15, 78),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (160, 180, 200), 1, cv2.LINE_AA)
        cv2.line(canvas, (panel_x + 15, 92), (self.img_w - 35, 92), (60, 75, 95), 1)

        # Section 1: Robot Pose & Court Safety Status
        cv2.putText(canvas, "[ ROBOT POSE & COURT SAFETY ]", (panel_x + 15, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 220, 240), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"X   : {self.robot_x:+.3f} m", (panel_x + 25, 142),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"Y   : {self.robot_y:+.3f} m", (panel_x + 25, 164),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"Yaw : {math.degrees(self.robot_yaw):+.2f} deg", (panel_x + 25, 186),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
        
        # Court Boundary & Clearance Check
        in_own_court = (self.robot_x <= 0.0)
        court_str = "OWN COURT (SAFE)" if in_own_court else "⚠️ ENEMY AREA (ILLEGAL)"
        court_col = (0, 255, 180) if in_own_court else (0, 0, 255)
        cv2.putText(canvas, f"Zone: {court_str}", (panel_x + 25, 208),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, court_col, 1, cv2.LINE_AA)
        cv2.line(canvas, (panel_x + 15, 225), (self.img_w - 35, 225), (60, 75, 95), 1)

        # Section 2: Linear Belt Actuator Speeds
        cv2.putText(canvas, "[ BELT LINEAR ACTUATOR ]", (panel_x + 15, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (200, 220, 240), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"Target: {self.belt_target_speed:5.2f} m/s", (panel_x + 25, 265),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 200), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"Actual: {self.belt_actual_speed:5.2f} m/s", (panel_x + 25, 290),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 220, 255), 1, cv2.LINE_AA)
        speed_err = abs(self.belt_target_speed - self.belt_actual_speed)
        cv2.putText(canvas, f"Error : {speed_err:5.2f} m/s", (panel_x + 25, 315),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (180, 190, 200), 1, cv2.LINE_AA)
        cv2.line(canvas, (panel_x + 15, 335), (self.img_w - 35, 335), (60, 75, 95), 1)

        # Section 3: Air Cylinder
        cv2.putText(canvas, "[ AIR CYLINDER ]", (panel_x + 15, 360),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (200, 220, 240), 1, cv2.LINE_AA)
        cyl_str = "DEPLOYED (ACTIVE)" if self.cylinder_deployed else "RETRACTED"
        cyl_col = (0, 255, 128) if self.cylinder_deployed else (140, 140, 140)
        cv2.putText(canvas, f"State : {cyl_str}", (panel_x + 25, 390),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, cyl_col, 1, cv2.LINE_AA)
        cv2.line(canvas, (panel_x + 15, 415), (self.img_w - 35, 415), (60, 75, 95), 1)

        # Section 4: Target Tracking
        cv2.putText(canvas, "[ TARGET LOCK ]", (panel_x + 15, 440),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (200, 220, 240), 1, cv2.LINE_AA)
        if self.has_target:
            cv2.putText(canvas, f"Dist  : {self.target_dist:5.2f} m", (panel_x + 25, 468),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(canvas, f"Angle : {self.target_angle_deg:+5.1f} deg", (panel_x + 25, 493),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)
        else:
            cv2.putText(canvas, "Searching target...", (panel_x + 25, 475),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.46, (120, 140, 160), 1, cv2.LINE_AA)
        cv2.line(canvas, (panel_x + 15, 518), (self.img_w - 35, 518), (60, 75, 95), 1)

        # Section 5: Opponent Robot (Enemy) Tracking
        cv2.putText(canvas, "[ OPPONENT ROBOT (ENEMY) ]", (panel_x + 15, 542),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (100, 150, 255), 1, cv2.LINE_AA)
        if self.has_opponent and (time.time() - self.last_opp_time < 2.0):
            cv2.putText(canvas, f"Pos   : X={self.opp_map_x:+.2f}m, Y={self.opp_map_y:+.2f}m",
                        (panel_x + 25, 570), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 120, 120), 1, cv2.LINE_AA)
            cv2.putText(canvas, f"Dist  : {self.opp_dist:4.2f} m ({self.opp_angle_deg:+5.1f} deg)",
                        (panel_x + 25, 595), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 0, 255), 1, cv2.LINE_AA)
            if self.opp_dist < 2.5:
                cv2.putText(canvas, "⚠️ PROXIMITY ALERT!", (panel_x + 25, 622),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            cv2.putText(canvas, "Scanning opponent arena...", (panel_x + 25, 575),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44, (120, 140, 160), 1, cv2.LINE_AA)

        # Footer
        cv2.line(canvas, (panel_x + 15, 650), (self.img_w - 35, 650), (60, 75, 95), 1)
        cv2.putText(canvas, "ROS2 TELEMETRY SERVER | 30 FPS", (panel_x + 15, self.img_h - 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100, 120, 140), 1, cv2.LINE_AA)

        # 6. Publish sensor_msgs/Image
        if self.publish_image:
            img_msg = Image()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "dashboard"
            img_msg.height = self.img_h
            img_msg.width = self.img_w
            img_msg.encoding = "bgr8"
            img_msg.is_bigendian = False
            img_msg.step = self.img_w * 3
            img_msg.data = bytes(canvas.tobytes())
            self.pub_image.publish(img_msg)

        # 7. Local OpenCV Window (if display active)
        if self.show_window:
            try:
                cv2.imshow("Robocon 2026 Jetson Data Server", canvas)
                cv2.waitKey(1)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = RobotDashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

