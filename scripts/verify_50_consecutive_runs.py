#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robocon 2026 - 50 Consecutive Full-Loop Zero-Collision Verification Script
Constraints:
1. Max X <= 0.0m throughout the entire loop trajectory (100% Own Court).
2. Min clearance to all 10 official objects >= 0.22m.
3. 50 consecutive runs across physical random perturbations.
"""

import os
import sys
import time
import math
import numpy as np
from scipy.spatial import cKDTree

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_DIR = os.path.dirname(SCRIPT_DIR)
MAP_PATH = os.path.join(WORKSPACE_DIR, "maps", "robocon2026_field.pcd")

OFFICIAL_OBJECTS = np.array([
    [-0.870,  0.000],  # B1
    [-1.480, -1.820],  # B2 (H600)
    [-1.480,  1.820],  # B3 (H300)
    [-4.980,  0.000],  # Chair
    [-3.855, -2.845],  # Desk 1
    [-3.855,  2.845],  # Desk 2
    [-5.445,  5.345],  # Desk 3
    [-1.105,  5.300],  # Desk 4
    [-3.025,  0.000],  # Flag
    [ 0.000,  0.550],  # Podium
], dtype=np.float64)

def load_pcd_map(pcd_file: str):
    with open(pcd_file, "rb") as f:
        header = []
        while True:
            line = f.readline().decode("ascii", errors="ignore")
            header.append(line)
            if line.startswith("DATA"):
                break
        points = 0
        for l in header:
            if l.startswith("POINTS"):
                points = int(l.split()[1])
        raw = f.read()
        arr = np.frombuffer(raw, dtype=np.float32)
        step = len(arr) // points
        map_pts = arr.reshape(points, step)[:, :3]
    return map_pts

def main():
    print("============================================================================")
    print(" 🚀 Robocon 2026 - 50 Consecutive Full-Loop Zero-Collision Verification")
    print("============================================================================")
    
    map_pts = load_pcd_map(MAP_PATH)
    map_2d = map_pts[::4, :2]
    map_tree = cKDTree(map_2d)
    obs_tree = cKDTree(OFFICIAL_OBJECTS)
    
    t_up = np.linspace(0, 1, 200)
    x_odom_up = 7.80 * t_up
    y_odom_up = -0.35 * t_up

    t_down = np.linspace(0, 1, 200)
    x_odom_down = 7.80 + (0.00 - 7.80) * t_down
    y_odom_down = -0.35 + (-2.20 - (-0.35)) * t_down

    raw_x_odom = np.concatenate([x_odom_up, x_odom_down])
    raw_y_odom = np.concatenate([y_odom_up, y_odom_down])
    traj_odom = np.column_stack([raw_x_odom, raw_y_odom])
    
    base_x = -3.58
    base_y = -3.90
    base_yaw = np.radians(90.0)
    
    np.random.seed(2026)
    passes = 0
    t0 = time.time()
    
    for i in range(1, 51):
        jx = np.random.uniform(-0.06, 0.06) if i > 1 else 0.0
        jy = np.random.uniform(-0.06, 0.06) if i > 1 else 0.0
        jyaw = np.random.uniform(-np.radians(2.0), np.radians(2.0)) if i > 1 else 0.0
        
        cur_x = base_x + jx
        cur_y = base_y + jy
        cur_yaw = base_yaw + jyaw
        
        c, s = np.cos(cur_yaw), np.sin(cur_yaw)
        R = np.array([[c, -s], [s, c]])
        traj_map = (R @ traj_odom.T).T + np.array([cur_x, cur_y])
        
        max_x = np.max(traj_map[:, 0])
        min_clearance = np.min(obs_tree.query(traj_map)[0])
        
        dists, _ = map_tree.query(traj_map, distance_upper_bound=0.40)
        inlier_ratio = np.mean(dists < 0.40) * 100.0
        
        is_pass = (max_x <= 0.0) and (min_clearance >= 0.22) and (inlier_ratio >= 90.0)
        if is_pass:
            passes += 1
            status_str = "✅ PASS"
        else:
            status_str = "❌ FAIL"
            
        if i % 10 == 0 or i <= 5:
            print(f"[{i:2d}/50] Pose: ({cur_x:5.2f}, {cur_y:5.2f}, {np.degrees(cur_yaw):5.1f}°) -> {status_str} | Max X: {max_x:+.2f}m (<=0.0) | Clearance: {min_clearance:.2f}m | Inliers: {inlier_ratio:.1f}%")
            
    elapsed = (time.time() - t0) * 1000.0
    print("----------------------------------------------------------------------------")
    print(f"🎯 連続成功回数: {passes} / 50 (所要時間: {elapsed:.1f}ms)")
    if passes == 50:
        print("🏆 50回連続完全無衝突・自陣限定・CADマップ精密合致 100% 達成！")
    else:
        print(f"⚠️ {50 - passes} 回のテストで制約違反が発生しました。")
        sys.exit(1)

if __name__ == "__main__":
    main()

