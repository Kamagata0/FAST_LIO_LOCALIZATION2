# FAST-LIO-LOCALIZATION2

事前に作成した点群地図（PCD）と、LiDAR が取得した現在の点群を照合して、ロボットの自己位置（グローバル位置姿勢）をリアルタイムに推定する ROS 2 パッケージです。

**Isaac Sim（シミュレーション）**、**実機Livox（Mid-360 等）**、**rosbag 再生** の 3 つの実行形態に対応しており、起動引数で柔軟に切り替えられます。

---

## 構成概要

```text
[LiDAR] + [IMU]
      ↓
[FAST-LIO (fastlio_mapping)] → /cloud_registered (現在点群) + /Odometry
      ↓
事前点群地図 (.pcd) → [ICP 地図照合 (global_localization.py)]
      ↓
/map_to_odom → [TF統合 (transform_fusion.py)] → /localization
```

### TF ツリー
```text
map → odom → body → livox_frame
```
- `map → odom`: `global_localization.py` / `transform_fusion.py` が発行（地図照合による補正。初期位置入力後に配信開始）
- `odom → body`: FAST-LIO (`fastlio_mapping`) が発行（高周波オドメトリ）
- `body → livox_frame`: launch 内の `static_transform_publisher` が発行

---

## 実行形態の違い（Isaac Sim / 実機Livox / rosbag）

| 項目 | Isaac Sim | 実機 Livox (Mid-360) | rosbag 再生 |
|---|---|---|---|
| 起動モード | `lidar_mode:=isaac` | `lidar_mode:=livox` | bag内の型に合わせる (`livox` or `isaac`) |
| シミュレーション時刻 | `use_sim_time:=false` (または true) | `use_sim_time:=false` | **`use_sim_time:=true`** (必須) |
| LiDAR メッセージ型 | `sensor_msgs/msg/PointCloud2` | `livox_ros_driver2/msg/CustomMsg` | bag内に記録された型 |
| LiDAR トピック名 | `/livox/lidar`（引数で変更可） | `/livox/lidar` | bag内のトピック名（引数で指定） |
| IMU メッセージ型 | `sensor_msgs/msg/Imu` | `sensor_msgs/msg/Imu` | `sensor_msgs/msg/Imu` |
| IMU トピック名 | `/livox/imu`（引数で変更可） | `/livox/imu` | bag内のトピック名（引数で指定） |
| ドライバ起動 | 不要 | **必要** (`livox.launch.py`) | 不要 (`ros2 bag play` で再生) |

---

## 必要な環境

- **OS**: Ubuntu 20.04 / 22.04
- **ROS 2**: Humble（推奨）
- **C++**: C++17, OpenMP, PCL, Eigen3
- **Python**: Python 3.8+ (Open3D, NumPy < 2.0, transforms3d, ros2_numpy)

---

## 環境構築手順（どのPCでも再現可能）

### 1. ROS 2 依存パッケージのインストール

```bash
sudo apt update
sudo apt install -y \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-tf-transformations \
  ros-humble-visualization-msgs \
  ros-humble-perception-pcl \
  python3-pip
```

### 2. Python 依存ライブラリのインストール

```bash
# NumPy 2.x との非互換を防ぐため numpy<2.0.0 を指定します
python3 -m pip install --user "numpy<2.0.0" open3d transforms3d ros2-numpy
```
> **注意**: Ubuntu 22.04 等で `externally-managed-environment` エラーが出る場合は、`--break-system-packages` オプションを付けて実行してください。

#### transforms3d の `np.float` 非推奨エラー対応
`transforms3d` 内部で古い `np.float` が使われている場合があるため、以下のコマンドを実行して自動修正します。

```bash
python3 -c "
import transforms3d.quaternions as tq
path = tq.__file__
with open(path, 'r') as f:
    code = f.read()
if 'np.float' in code:
    with open(path, 'w') as f:
        f.write(code.replace('np.float', 'float'))
    print('Successfully patched transforms3d (np.float -> float)')
else:
    print('transforms3d is already compatible')
"
```

---

### 3. Livox-SDK2 のビルド・インストール（必須）

本パッケージの C++ ノードは `livox_ros_driver2` に依存しているため、事前に **Livox-SDK2** をインストールします（Isaac Sim のみを使用する場合でもビルドに必要です）。

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

---

### 4. ROS 2 ワークスペースの作成とリポジトリの配置

ワークスペース（例: `~/ros2_ws`）を作成し、ソースディレクトリにリポジトリを配置します。

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 1. 本リポジトリ（既にクローン済みの場合は配置）
# git clone <本リポジトリURL> FAST_LIO_LOCALIZATION2

# 2. livox_ros_driver2 のクローン（ビルドに必須）
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

---

## ビルド

ROS 2 ワークスペースのルートディレクトリに移動してビルドします。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# 1. livox_ros_driver2 のビルド
colcon build --symlink-install --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2

# 2. fast_lio_localization のビルド
colcon build --symlink-install --packages-select fast_lio_localization

# 3. ワークスペースの環境変数を読み込み
source install/setup.bash
```

> **TIP**: 毎回 `source` する手間を省く場合は、`~/.bashrc` に追記しておくと便利です：
> ```bash
> echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
> ```

---

## 起動方法

### パターン A: Isaac Sim（シミュレータ）で使用する場合

#### 1. Isaac Sim 側の設定
Action Graph などで以下のトピックを配信するように設定し、シミュレーションを **Play（再生）** します：
- **LiDAR 点群**: `/livox/lidar` (`sensor_msgs/msg/PointCloud2`, frame_id: `livox_frame`)
- **IMU**: `/livox/imu` (`sensor_msgs/msg/Imu`, frame_id: `imu_link`)

#### 2. ローカライゼーションノードの起動
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch fast_lio_localization localization.launch.py lidar_mode:=isaac
```
※トピック名を変更している場合：
```bash
ros2 launch fast_lio_localization localization.launch.py \
  lidar_mode:=isaac \
  lidar_topic:=/my_robot/lidar \
  imu_topic:=/my_robot/imu
```

---

### パターン B: 実機 Livox（Mid-360 / Jetson 等）で使用する場合

> **Jetson を使用する場合の事前準備**:
> 処理落ちによる EKF の発散（位置の吹き飛び）を防ぐため、最大パフォーマンスに設定してください。
> ```bash
> sudo nvpmodel -m 0 && sudo jetson_clocks
> ```

#### 1. Livox ドライバの起動
- **通常（正立）設置の場合**（※`inverted:=false` を必ず指定）:
  ```bash
  cd ~/ros2_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash

  ros2 launch fast_lio_localization livox.launch.py xfer_format:=1 inverted:=false
  ```
- **逆さま（倒立）設置の場合**:
  ```bash
  ros2 launch fast_lio_localization livox.launch.py xfer_format:=1 inverted:=true
  ```

#### 2. ローカライゼーションノードの起動
別ターミナルで起動します：
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch fast_lio_localization localization.launch.py lidar_mode:=livox
```

> **注意**: 起動直後の 2〜3 秒間は IMU 重力推定キャリブレーションのため、**ロボットを完全に静止** させてください。

---

### パターン C: rosbag 再生で使用する場合

#### 1. rosbag 内のトピックと形式を確認
```bash
ros2 bag info /path/to/your_bag_directory
```
*(※フォルダ内に `metadata.yaml` があるディレクトリパスを指定してください)*

- 点群型が `livox_ros_driver2/msg/CustomMsg` の場合 → `lidar_mode:=livox`
- 点群型が `sensor_msgs/msg/PointCloud2` の場合 → `lidar_mode:=isaac`

#### 2. ローカライゼーションノードの起動 (ターミナル 1)
rosbag 再生時は必ず **`use_sim_time:=true`** を付与します。
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 例: CustomMsg 形式の場合
ros2 launch fast_lio_localization localization.launch.py \
  use_sim_time:=true \
  lidar_mode:=livox \
  map:=/path/to/your_map.pcd

# 例: PointCloud2 形式でトピック名が異なる場合
ros2 launch fast_lio_localization localization.launch.py \
  use_sim_time:=true \
  lidar_mode:=isaac \
  lidar_topic:=/my_bag/lidar \
  imu_topic:=/my_bag/imu \
  map:=/path/to/your_map.pcd
```

#### 3. rosbag の再生 (ターミナル 2)
再生時は必ず **`--clock`** オプションを付与して時刻を配信します。
```bash
source ~/ros2_ws/install/setup.bash

ros2 bag play /path/to/your_bag_directory --clock
```

---

## 別の地図（.pcd）を使用する場合

デフォルトでは `maps/robocon2026_field.pcd` が読み込まれます。別の地図ファイルを使用したい場合は `map` 引数で絶対パスを指定します：

```bash
ros2 launch fast_lio_localization localization.launch.py \
  lidar_mode:=livox \
  map:=/path/to/your_map.pcd
```

---

## RViz2 での初期位置合わせ手順

起動直後は待機状態となり、**初期位置（`/initialpose`）を入力するまで `/map_to_odom` はパブリッシュされません**。

1. 自動起動した RViz2 の画面上部にある **`2D Pose Estimate`** ツールをクリックします。
2. 地図点群上の**ロボットがいるおおよその位置をクリック**し、**ロボットの進行方向（向き）へドラッグ**して離します。
3. 初期位置が入力されると `global_localization` が自動で ICP マッチングを行い、`/map_to_odom` の配信が始まります。
4. グローバル地図（白/グレー）とリアルタイム点群（`/cloud_registered`）が壁や障害物の位置にピタッと重なります。

---

## 動作確認用コマンド

```bash
# 入力トピックの確認（IMU は約 200Hz）
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu

# 出力トピックの確認
ros2 topic hz /cloud_registered      # FAST-LIO処理後の点群
ros2 topic hz /map                   # 事前地図の点群
ros2 topic echo /map_to_odom --once  # 地図照合オフセット (初期位置入力後に出力)
ros2 topic echo /localization --once # 推定自己位置

# TF ツリーの確認
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom body
ros2 run tf2_ros tf2_echo body livox_frame
```

---

## トラブルシューティング

### 1. ロボットが遥か彼方に飛んでいく / 発散する
- **LiDAR の正立/逆さま設定**: 実機起動時、正立設置なのに `inverted:=false` を忘れていませんか？（`livox.launch.py` はデフォルトが `inverted:=true` のため、Y/Z軸と重力加速度が反転して暴走します）
- **起動時の静止**: 起動直後（最初の数秒間）にロボットが動いていると重力推定に失敗して吹き飛びます。
- **Jetson の処理落ち**: `sudo nvpmodel -m 0 && sudo jetson_clocks` を実行してください。
- **rosbag 時刻設定**: `use_sim_time:=true` と `ros2 bag play ... --clock` の両方が設定されているか確認してください。

### 2. `/map_to_odom` を echo しても何も出力されない
- **初期位置（2D Pose Estimate）は設定しましたか？**: ノード起動直後は初期化待ち（`Waiting for initial pose...`）となり、初期位置が与えられるまで `/map_to_odom` は配信されません。RViz2 上で `2D Pose Estimate` を行ってください。

### 3. 点群が RViz2 に表示されない / 出ない
- **IMU トピックは届いていますか？**: `ros2 topic hz /livox/imu` を確認してください。FAST-LIO は IMU が無いと点群を一切処理・出力しません。
- **RViz2 の Fixed Frame**: 左上 `Global Options` -> `Fixed Frame` が `map`（または `odom`）になっているか確認してください。

### 4. ビルド時に `livox_ros_driver2Config.cmake` が見つからない
- `Livox-SDK2` が正しくインストールされているか確認してください。
- `~/ros2_ws/src` に `livox_ros_driver2` をクローンし、先にビルドしてから `fast_lio_localization` をビルドしてください。

### 3. ビルドキャッシュや古いユーザー名のパスが残っている
過去に別の環境やユーザー名でビルドしたキャッシュが残っている場合は、`build/` と `install/` を削除してクリーンビルドしてください：

```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

### 4. 点群が回転・傾いてずれる
- IMU と LiDAR の取り付け向き（回転行列）が合っているか確認してください。
- 起動直後のキャリブレーション中はロボットを静止させてください。

---

## トピック一覧

| トピック名 | 型 | 説明 |
|---|---|---|
| `/livox/lidar` | `PointCloud2` または `CustomMsg` | LiDAR 入力データ |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 6軸 IMU 入力データ |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | FAST-LIO が統合した現在のリアルタイム点群 |
| `/Odometry` | `nav_msgs/msg/Odometry` | FAST-LIO による高周波オドメトリ |
| `/map` | `sensor_msgs/msg/PointCloud2` | 事前地図（PCD）の点群 |
| `/cur_scan_in_map` | `sensor_msgs/msg/PointCloud2` | 地図座標系に変換されたスキャン点群 |
| `/map_to_odom` | `nav_msgs/msg/Odometry` | グローバル地図とオドメトリ間の補正量 |
| `/localization` | `nav_msgs/msg/Odometry` | 補正後の最終的な自己位置姿勢 |

---

## 関連プロジェクト & 謝辞

- [FAST-LIO](https://github.com/hku-mars/FAST_LIO)
- [FAST-LIO-ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree)
- [Fast-Lio-Localization](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)
