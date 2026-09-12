# FAST-LIO-LOCALIZATION2 ＆ ロボコン2026特化型データサーバーシステム

事前点群地図（`robocon2026_field.pcd`）と LiDAR 点群（Livox Mid-360 等）を高精度に照合し、ロボットの自己位置（グローバル位置姿勢）をリアルタイムに推定する ROS 2 パッケージです。

**一般的な Ubuntu PC (x86_64)** および **NVIDIA Jetson シリーズ (ARM64: Orin Nano / Orin NX / AGX Orin / Xavier 等)** のどちらでも、ゼロから環境構築して動作させることができます。

---

## 1. 対応環境・動作要件

| 項目 | 一般 PC (デスクトップ / ノート) | NVIDIA Jetson シリーズ |
|---|---|---|
| **CPU アーキテクチャ** | `x86_64` (Intel / AMD) | `aarch64` (ARM64) |
| **OS** | Ubuntu 22.04 LTS (推奨) / 20.04 | JetPack 6.x (Ubuntu 22.04 推奨) / JetPack 5.x |
| **ROS 2** | ROS 2 Humble (推奨) | ROS 2 Humble |
| **LiDAR** | Livox Mid-360 (実機 / rosbag / Isaac Sim) | Livox Mid-360 (実機 / rosbag) |
| **Python** | Python 3.8+ (Open3D, NumPy < 2.0, transforms3d) | Python 3.8+ (同左) |

---

## 2. ゼロからの環境構築（全PC・Jetson共通手順）

どのパソコン（一般 PC または Jetson）でも、上から順にターミナルで実行するだけでセットアップが完了します。

### Step 1: (Jetsonのみ) パフォーマンス最大化とスワップメモリ作成

> **※ 一般 PC の場合はこの Step をスキップして Step 2 へ進んでください。**

Jetson では、ビルド時のメモリ不足（OOM Crash）を防ぎ、実行パフォーマンスを最大化するために以下を実行します。

```bash
# 1. 動作クロック・電力を最大化 (Max-N モード)
sudo nvpmodel -m 0
sudo jetson_clocks

# 2. スワップメモリ（8GB）の作成（ビルド時のメモリ不足フリーズ防止）
if [ $(free -m | awk '/Swap/ {print $2}') -lt 4000 ]; then
    sudo fallocate -l 8G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
    echo "8GB Swap created successfully!"
fi
```

---

### Step 2: ROS 2 & システム依存パッケージのインストール

ROS 2 Humble がインストールされている環境で、ビルドに必要なライブラリを導入します。

```bash
sudo apt update
sudo apt install -y \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-tf-transformations \
  ros-humble-visualization-msgs \
  ros-humble-perception-pcl \
  python3-pip \
  git cmake build-essential \
  libgl1-mesa-dev libglu1-mesa-dev
```

---

### Step 3: Python 依存ライブラリのインストール & パッチ適用

NumPy 2.x による非互換エラーを防ぐため `numpy<2.0.0` を明示し、`transforms3d` 内部の非推奨型エラー（`np.float`）を自動修正します。

```bash
# 1. Python パッケージのインストール
# (※ Ubuntu 22.04 等でエラーが出る場合は --break-system-packages を付与してください)
python3 -m pip install --user "numpy<2.0.0" open3d transforms3d ros2-numpy --break-system-packages 2>/dev/null || \
python3 -m pip install --user "numpy<2.0.0" open3d transforms3d ros2-numpy

# 2. transforms3d の np.float 自動パッチ (実行必須)
python3 -c "
import transforms3d.quaternions as tq
path = tq.__file__
with open(path, 'r') as f:
    code = f.read()
if 'np.float' in code:
    with open(path, 'w') as f:
        f.write(code.replace('np.float', 'float'))
    print('✅ Patched transforms3d successfully (np.float -> float).')
else:
    print('✅ transforms3d is already compatible.')
"
```

> **Jetson での Open3D に関する注意**:
> もし Jetson (ARM64) で `pip install open3d` が見つからない（No matching distribution）と出る場合でも、本リポジトリの特化型ノード（`field_localization_node.py`）は Open3D に依存しない高速アルゴリズムで動作するため問題ありません。

---

### Step 4: Livox-SDK2 のビルド・インストール（必須基盤）

FAST-LIO および Livox ROS ドライバの通信基盤となる **Livox-SDK2** をインストールします（PC / Jetson 共通）。

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

---

### Step 5: ROS 2 ワークスペースの作成とリポジトリ配置

ワークスペース `~/ros2_ws` を作成し、本リポジトリと `livox_ros_driver2` をクローンします。

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 1. 本リポジトリのクローン
git clone https://github.com/Kamagata0/FAST_LIO_LOCALIZATION2.git

# 2. livox_ros_driver2 のクローン（ビルドに必須）
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

---

### Step 6: パッケージのビルド

ワークスペースのルート（`~/ros2_ws`）でビルドします。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# 1. livox_ros_driver2 のビルド (ROS 2 版)
colcon build --symlink-install --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2

# 2. fast_lio_localization のビルド
# ※ Jetson (メモリ8GB以下) の場合はフリーズを防ぐため並列数を制限します
if [ -f /etc/nv_tegra_release ]; then
  echo "Detected Jetson: Building with limited parallel workers..."
  colcon build --symlink-install --packages-select fast_lio_localization --parallel-workers 2
else
  colcon build --symlink-install --packages-select fast_lio_localization
fi

# 3. ワークスペースのセットアップファイルを読み込み
source install/setup.bash
```

> **TIP**: 次回以降ターミナルを開いたときに自動で読み込まれるよう、`~/.bashrc` に登録しておくことを推奨します：
> ```bash
> echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
> echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
> source ~/.bashrc
> ```

---

### Step 7: (実機使用時) 有線LANネットワーク設定

PC / Jetson と Livox Mid-360 をイーサネットケーブルで接続する場合、PC側の有線LAN（イーサネット）を以下の静的IPに設定します：

- **IPv4 方式**: 手動 (Manual)
- **IP アドレス**: `192.168.1.50`
- **ネットマスク**: `255.255.255.0` (または `/24`)
- **ゲートウェイ**: `192.168.1.1`

接続テスト（LiDAR のデフォルトIP宛てに ping）：
```bash
# Mid-360 本体のブロードキャストコード末尾2桁が例えば "12" の場合:
ping -c 3 192.168.1.112
```

---

## 3. 起動方法・運用モード

### モード 1: ロボコン2026 統合システム（実機運用・推奨）
初期位置・オドメトリ一切不要！外壁幾何照合により **1フレームで絶対自己位置を確定** し、2D サイバーHUDと 3D RViz2 が起動します。

#### ターミナル 1: Livox ドライバの起動
```bash
# 正立設置（通常）の場合:
ros2 launch fast_lio_localization livox.launch.py xfer_format:=1 inverted:=false

# 逆さま天吊り設置の場合:
ros2 launch fast_lio_localization livox.launch.py xfer_format:=1 inverted:=true
```

#### ターミナル 2: ロボコン統合システムの起動
```bash
ros2 launch fast_lio_localization robocon_system.launch.py
```

---

### モード 2: 競技本番用・ヘッドレスモード（Jetson推奨・極限軽量）
GUI（RViz2 / 2D HUD）の描画をオフにし、**CPU負荷 <10%・遅延 <2ms** で自己位置・ターゲット相対座標・ロボット状態をマイコン等の制御ノードへ高速配信します。

```bash
ros2 launch fast_lio_localization robocon_system.launch.py dashboard:=false rviz:=false
```

---

### モード 3: rosbag による動作デモ・シミュレーション
手元にある rosbag ファイルを指定して、誰のパソコンでも即座に動作確認ができます。

```bash
cd ~/ros2_ws/src/FAST_LIO_LOCALIZATION2

# 使用方法: bash scripts/run_robocon_demo.sh <rosbagのパス>
bash scripts/run_robocon_demo.sh /path/to/your/rosbag

# 例: ホームディレクトリにある rosbag を再生する場合
bash scripts/run_robocon_demo.sh ~/rosbag2_2026_09_08-16_17_47
```

---

### モード 4: ワンコマンド rosbag 記録 (`record_all.sh`)
試合や実験中の全センサ・自己位置・ターゲット・機構状態・2D HUD画像をタイムスタンプ付きで一括自動保存します。

```bash
cd ~/ros2_ws/src/FAST_LIO_LOCALIZATION2
bash scripts/record_all.sh [オプション: タグ名]
```
*(保存先: `bags/bag_YYYYMMDD_HHMMSS_タグ名/`)*

---

### モード 5: 通常の FAST-LIO-LOCALIZATION（ICP 位置合わせ）
従来の汎用 FAST-LIO モードです。RViz の `2D Pose Estimate` で大まかな初期位置を与えて収束させます。

```bash
# 実機 Livox の場合
ros2 launch fast_lio_localization localization.launch.py lidar_mode:=livox

# Isaac Sim (または PointCloud2 形式の rosbag) の場合
ros2 launch fast_lio_localization localization.launch.py lidar_mode:=isaac use_sim_time:=true
```

---

## 4. ROS 2 トピック ＆ メッセージ仕様

### 主要トピック一覧
| トピック名 | メッセージ型 | 配信周期 | 説明 |
|---|---|---|---|
| `/robot_pose` | `geometry_msgs/msg/PoseStamped` | 50Hz | 自機ロボットの絶対位置姿勢 $(X, Y, \text{Yaw})$ |
| `/target_relative` | `geometry_msgs/msg/PointStamped` | 50Hz | ターゲットの車体基準相対座標（距離・角度） |
| `/opponent_pose` | `geometry_msgs/msg/PoseStamped` | 50Hz | 相手ロボット（敵機）の絶対位置座標 $(X, Y)$ |
| `/opponent_relative` | `geometry_msgs/msg/PointStamped` | 50Hz | 相手ロボットの相対距離・方位 |
| `/robot_status` | `fast_lio_localization/msg/RobotStatus` | 50Hz | ベルト速度・シリンダ・ターゲット・敵機総合情報 |
| `/robot_dashboard/image` | `sensor_msgs/msg/Image` | 30Hz | 2D サイバーHUDダッシュボード画像 |
| `/localization` | `nav_msgs/msg/Odometry` | 50Hz | グローバルオドメトリ |
| `/field_scan_filtered` | `sensor_msgs/msg/PointCloud2` | 10Hz | 外壁抽出・地面除去済みの点群 |
| `/map` | `sensor_msgs/msg/PointCloud2` | 0.2Hz | 事前地図（`robocon2026_field.pcd`） |

---

## 5. トラブルシューティング

### Q1. ビルド時にフリーズする / コンパイルが途中で強制終了される
- **原因**: RAM（メモリ）が不足しています（特に Jetson やメモリ 8GB 以下の PC）。
- **解決策**:
  1. Step 1 に記載の **8GB スワップファイル作成** を実行してください。
  2. ビルドコマンドに `--parallel-workers 2` を追加して、同時並列ビルド数を制限してください：
     ```bash
     colcon build --symlink-install --packages-select fast_lio_localization --parallel-workers 2
     ```

### Q2. rosbag 再生時に点群やオドメトリが出ない
- **原因 1**: `use_sim_time:=true` が指定されていない。
  - rosbag 再生時は必ず `use_sim_time:=true` を指定し、再生側でも `--clock` を付けてください。
- **原因 2**: LiDAR の型が合っていない。
  - `ros2 bag info <bag>` を確認し、`/livox/lidar` が `PointCloud2` の場合は `lidar_mode:=isaac` を指定してください。

### Q3. `transforms3d` で `AttributeError: module 'numpy' has no attribute 'float'` が出る
- **原因**: NumPy 1.24 以降で `np.float` が廃止されたためです。
- **解決策**: Step 3 のワンライナーコマンドを実行してパッチを適用してください。

### Q4. 以前ビルドした古いパス（他ユーザーのパス）が残っている
- **解決策**: ワークスペースのビルドキャッシュをクリーンにして再ビルドします：
  ```bash
  cd ~/ros2_ws
  rm -rf build/ install/ log/
  colcon build --symlink-install
  source install/setup.bash
  ```

---

## 謝辞 & 関連プロジェクト
- [FAST-LIO](https://github.com/hku-mars/FAST_LIO)
- [FAST-LIO-ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree)
- [Fast-Lio-Localization](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)
