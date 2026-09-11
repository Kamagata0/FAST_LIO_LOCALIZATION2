# FAST-LIO-LOCALIZATION2 ＆ ロボコン2026特化型データサーバーシステム

事前点群地図（`robocon2026_field.pcd`）と LiDAR 点群（Livox Mid-360 等）を高精度に照合し、ロボットの自己位置（グローバル位置姿勢）をリアルタイムに推定する ROS 2 パッケージです。

**「オドメトリ不要・初期位置不要の超特化型自己位置推定」**、**「ベルト直動・シリンダ・ターゲット情報の ROS 2 データサーバー化」**、**「2D サイバーHUDリアルタイム可視化」**、**「ワンコマンド rosbag 記録」** を統合しています。

---

## 🌟 主な特徴

1. **初期位置・オドメトリ完全不要（1フレームで絶対位置確定）**:
   - `robocon2026_field.pcd` の既知外壁4辺寸法および中央教壇を直接幾何照合。
   - スリップによるオドメトリドリフトや初期位置設定の手間が**原理的にゼロ**。
2. **ターゲット相対距離・方位角のリアルタイム抽出**:
   - 残存点群クラスタからターゲット物体（ボール／台座）を抽出し、ロボット車体・直動機構基準の距離と角度を常時出力。
3. **Jetson ROS 2 テレメトリデータサーバー**:
   - ベルト直動速度（目標/実効速度）、エアシリンダ状態、ターゲット追従情報を `RobotStatus.msg` で一元統括。
4. **2D サイバーHUDダッシュボード（超軽量 30fps）**:
   - OpenCVによる俯瞰HUD描画（CPU負荷 < 2%）。ロボット位置・向き、直動方向、ターゲット追従線、速度数値をリアルタイム表示。
5. **ワンコマンド rosbag 記録 (`record_all.sh`)**:
   - 1コマンドで全センサ・自己位置・ターゲット・機構状態・2D HUD画像をタイムスタンプ付きで一括自動保存。
6. **超高速・省電力（Jetson 最適化）**:
   - 処理遅延 **約 2〜4ms**、Jetson 上での CPU 使用率 **約 10〜18%**。

---

## 🚀 Jetson 実機での「1からの完全セットアップ手順」

Jetson（Orin / Xavier 等）で本システムをゼロから動かすための完全な手順です。上から順にコマンドを実行するだけで動作環境が整います。

### Step 1: Jetson のパフォーマンス最大化（必須）
```bash
sudo nvpmodel -m 0
sudo jetson_clocks
```

### Step 2: ROS 2 & システム依存パッケージのインストール
```bash
sudo apt update
sudo apt install -y \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-tf-transformations \
  ros-humble-visualization-msgs \
  ros-humble-perception-pcl \
  python3-pip \
  git cmake build-essential
```

### Step 3: Python 依存ライブラリのインストール
```bash
# NumPy 2.x との非互換を防ぐため numpy<2.0.0 を指定
python3 -m pip install --user "numpy<2.0.0" open3d transforms3d ros2-numpy --break-system-packages

# transforms3d の非推奨型エラー（np.float）を自動修復
python3 -c "
import transforms3d.quaternions as tq
path = tq.__file__
with open(path, 'r') as f:
    code = f.read()
if 'np.float' in code:
    with open(path, 'w') as f:
        f.write(code.replace('np.float', 'float'))
    print('Patched transforms3d successfully.')
"
```

### Step 4: Livox-SDK2 のインストール
```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

### Step 5: ワークスペースの作成とリポジトリの配置
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 1. 本リポジトリのクローン
git clone https://github.com/Kamagata0/FAST_LIO_LOCALIZATION2.git

# 2. livox_ros_driver2 のクローン
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

### Step 6: パッケージのビルド
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# 1. livox_ros_driver2 のビルド (ROS 2 版)
colcon build --symlink-install --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2

# 2. fast_lio_localization のビルド
colcon build --symlink-install --packages-select fast_lio_localization

# 3. 環境変数を .bashrc に登録（次回から自動読込）
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 7: Jetson と Livox Mid-360 の有線LANネットワーク設定
Livox Mid-360 のデフォルトIPは `192.168.1.1XX`（ブロードキャストコード下2桁）です。Jetson 側の有線LANのIPアドレスを固定します：
- **IPアドレス**: `192.168.1.50`
- **サブネットマスク**: `255.255.255.0`
- **ゲートウェイ**: `192.168.1.1`

接続確認：
```bash
ping -c 3 192.168.1.1XX  # Mid-360 のIP宛て
```

---

## 🎮 実行方法・運用モード

### モード 1: ロボコン2026 総合システム起動（実機運用・デバッグ）
2D サイバーHUDダッシュボードと 3D RViz2 が同時に起動し、自己位置・ターゲット・機構状態を可視化します。

#### ターミナル 1: Livox ドライバ起動
```bash
# 正立（通常）設置の場合
ros2 launch fast_lio_localization livox.launch.py xfer_format:=1 inverted:=false

# 逆さま（天吊り）設置の場合
ros2 launch fast_lio_localization livox.launch.py xfer_format:=1 inverted:=true
```

#### ターミナル 2: ロボコン統合システム起動
```bash
ros2 launch fast_lio_localization robocon_system.launch.py
```

---

### モード 2: 試合本番用・最速ヘッドレスモード（画面なし・極限軽量）
GUI（RViz2 / 2D HUD）の描画処理を完全にオフにし、**最小CPU負荷（<10%）・最低遅延（<2ms）** でマイコンや制御ノードへトピックのみを高速配信します。

```bash
ros2 launch fast_lio_localization robocon_system.launch.py dashboard:=false rviz:=false
```

---

### モード 3: ワンコマンド rosbag 記録
全センサ・自己位置・ターゲット・機構状態・2D HUD画像をタイムスタンプ付きディレクトリへ自動一括記録します。

```bash
cd ~/ros2_ws/src/FAST_LIO_LOCALIZATION2
bash scripts/record_all.sh [オプション: タグ名]
```
*(保存先: `bags/bag_YYYYMMDD_HHMMSS_タグ名/`)*

---

### モード 4: rosbag でのデモ動作確認
```bash
cd ~/ros2_ws/src/FAST_LIO_LOCALIZATION2
bash scripts/run_robocon_demo.sh
```

---

## 📡 ROS 2 トピック ＆ メッセージ定義

### 1. カスタムメッセージ: `RobotStatus.msg` (`fast_lio_localization/msg/RobotStatus`)
```text
std_msgs/Header header

# ベルト直動アクチュエータ
float64 belt_target_speed    # 目標速度 [m/s]
float64 belt_actual_speed    # 実効速度 [m/s]

# エアシリンダ
bool cylinder_deployed       # 展開状態 (True: 展開, False: 格納)

# ターゲット相対トラッキング
bool target_detected         # ターゲット検出成否
float64 target_distance      # ターゲットまでの距離 [m]
float64 target_angle_rad     # ターゲット方位角 [rad]
float64 target_angle_deg     # ターゲット方位角 [deg]
```

### 2. 主要トピック一覧
| トピック名 | メッセージ型 | 配信周期 | 説明 |
|---|---|---|---|
| `/robot_pose` | `geometry_msgs/msg/PoseStamped` | 50Hz | ロボットの絶対位置姿勢 $(X, Y, \text{Yaw})$ |
| `/target_relative` | `geometry_msgs/msg/PointStamped` | 50Hz | ターゲットの車体基準相対座標 |
| `/robot_status` | `fast_lio_localization/msg/RobotStatus` | 50Hz | ベルト速度・シリンダ・ターゲット総合情報 |
| `/robot_dashboard/image` | `sensor_msgs/msg/Image` | 30Hz | 2D サイバーHUD描画画像 |
| `/localization` | `nav_msgs/msg/Odometry` | 50Hz | グローバルオドメトリ |
| `/localization_path` | `nav_msgs/msg/Path` | 10Hz | ロボットの走行軌跡 |
| `/field_scan_filtered` | `sensor_msgs/msg/PointCloud2` | 10Hz | 外壁抽出・地面除去済みの点群 |
| `/map` | `sensor_msgs/msg/PointCloud2` | 0.2Hz | `robocon2026_field.pcd` の基準点群 |

---

## 🛠 動作確認用コマンド

```bash
# 1. 自己位置の確認 (X, Y, Yaw)
ros2 topic echo /robot_pose --once

# 2. ターゲット相対距離・方位の確認
ros2 topic echo /target_relative --once

# 3. 機構総合ステータスの確認
ros2 topic echo /robot_status --once

# 4. TF 変換（map -> body）の確認
ros2 run tf2_ros tf2_echo map body
```

---

## 謝辞 & 関連プロジェクト
- [FAST-LIO](https://github.com/hku-mars/FAST_LIO)
- [FAST-LIO-ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree)
- [Fast-Lio-Localization](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)
