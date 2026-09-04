# FAST-LIO-LOCALIZATION2

事前に作成した点群地図と、LiDAR が取得した現在の点群を照合して、ロボットの自己位置を推定する ROS 2 パッケージです。

Isaac Sim と実機Livoxの両方に対応しており、起動時の `lidar_mode` で入力形式を切り替えます。どちらのモードでもFAST-LIOとIMUを使用し、地図照合で自己位置を補正します。

## 構成

```text
LiDAR + IMU → FAST-LIO → /cloud_registered + /Odometry
                                  ↓
事前点群地図 robocon2026_field.pcd → ICPによる地図照合
                                  ↓
                    /map_to_odom → /localization
```

## Isaac Simと実機の違い

| 項目 | Isaac Sim | 実機Livox |
|---|---|---|
| 起動モード | `lidar_mode:=isaac` | `lidar_mode:=livox` |
| LiDAR型 | `sensor_msgs/msg/PointCloud2` | `livox_ros_driver2/msg/CustomMsg` |
| LiDARトピック | `/livox/lidar`（変更時は`lidar_topic`で指定） | `/livox/lidar` |
| IMU型 | `sensor_msgs/msg/Imu` | `sensor_msgs/msg/Imu` |
| IMUトピック | `imu_topic`で指定 | 通常 `/livox/imu` |
| FAST-LIO | PointCloud2入力で起動 | CustomMsg入力で起動 |

Isaac SimではLiDARとIMUをROS 2へPublishしてください。実機では先に `livox_ros_driver2` を起動します。FAST-LIOの出力は、両モードとも `/cloud_registered` と `/Odometry` に統一されます。

TF は次の構成にします。

```text
map → odom → body → livox_frame
```

## 必要な環境

- Ubuntu 20.04 以降
- ROS 2 Humble
- Python 3.8以降
- Open3D
- ros2_numpy
- transforms3d
- tf_transformations
- pcl_ros
- livox_ros_driver2

依存パッケージをインストールします。

```bash
sudo apt install ros-humble-pcl-ros
sudo apt install ros-humble-tf-transformations
python3 -m pip install --user open3d ros2-numpy transforms3d
```

`transforms3d`で`np.float`エラーが出る場合は、`/usr/lib/python3/dist-packages/transforms3d/quaternions.py`の`np.float`を`float`に置き換えてください。

## ビルド

ROS 2 ワークスペースのルートで実行します。

```bash
cd /ros2_ws
colcon build --symlink-install --packages-select fast_lio_localization
source /opt/ros/humble/setup.bash
source /home/akeru/ros2_ws/install/setup.bash
```

## Isaac Sim 側の設定

### LiDARとIMU

ROS 2 Publish Point Cloud の設定:

```text
Topic: /livox/lidar
Type: sensor_msgs/msg/PointCloud2
Frame: livox_frame
```

IMUの例:

```text
Topic: /livox/imu
Type: sensor_msgs/msg/Imu
Frame: imu_link
```

Isaac Simのオドメトリ入力は不要です。FAST-LIOがIMUとPointCloud2から `/Odometry` を生成します。

## 実機Livox側の設定

先にLivoxドライバを起動し、通常は次のトピックをPublishします。

```text
/livox/lidar : livox_ros_driver2/msg/CustomMsg
/livox/imu   : sensor_msgs/msg/Imu
```

`xfer_format:=1` はCustomMsg出力です。

### TF

必要なTF:

```text
odom → body           動的
body → livox_frame    静的
```

FAST-LIOが`odom → body`を出力し、launchが`body → livox_frame`を発行します。

## 起動

デフォルトでは次の地図を使用します。

```text
robocon2026_field.pcd
```

Isaac SimとROS 2のLiDAR・IMU Publishを開始した後、別ターミナルで起動します。

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch fast_lio_localization localization.launch.py
```

このlaunchはデフォルトでRViz2も起動します。RViz2を起動しない場合は、次のように指定します。

```bash
ros2 launch fast_lio_localization localization.launch.py rviz:=false
```

Isaac Simのデフォルトトピックは実機と同じ `/livox/lidar` と `/livox/imu` です。トピック名を変更している場合だけ、起動時に上書きします。

```bash
ros2 launch fast_lio_localization localization.launch.py \
  lidar_mode:=isaac \
  lidar_topic:=/livox/lidar \
  imu_topic:=/livox/imu
```

例えばIsaac Sim側を `/isaac/lidar` と `/isaac/imu` に変更した場合:

```bash
ros2 launch fast_lio_localization localization.launch.py \
  lidar_mode:=isaac \
  lidar_topic:=/isaac/lidar \
  imu_topic:=/isaac/imu
```

実機Livoxの場合:

```bash
ros2 launch fast_lio_localization livox.launch.py xfer_format:=1
ros2 launch fast_lio_localization localization.launch.py \
  lidar_mode:=livox \
  lidar_topic:=/livox/lidar \
  imu_topic:=/livox/imu
```

別の地図を使う場合:

```bash
ros2 launch fast_lio_localization localization.launch.py \
  map:=/absolute/path/to/map.pcd
```

## RViz で初期位置を指定

RViz2では、起動後に次を確認します。

1. RViz の Fixed Frame が `map` であることを確認します。
2. `2D Pose Estimate` を選びます。
3. 地図上のロボットのおおよその位置をクリックします。
4. ロボットの向きに合わせてドラッグします。
5. ロボットを停止した状態で ICP が実行されるのを待ちます。

初期位置は完全な自己位置ではなく、ICP が探索を始めるための大まかな位置です。地図と現在の点群の座標系も一致している必要があります。

## 動作確認

Isaac Sim の入力:

```bash
ros2 topic type /livox/lidar
ros2 topic type /livox/imu
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

期待される型:

```text
sensor_msgs/msg/PointCloud2
sensor_msgs/msg/Imu
```

TF の確認:

```bash
ros2 run tf2_ros tf2_echo odom body
ros2 run tf2_ros tf2_echo body livox_frame
```

自己位置推定結果の確認:

```bash
ros2 topic echo /map_to_odom --once
ros2 topic echo /localization --once
```

成功すると、白い事前地図と LiDAR 点群が同じ壁・柱・障害物の位置に重なります。

## トラブルシューティング

### `/map_to_odom` が出ない

次を確認します。

```bash
ros2 topic info /livox/lidar
ros2 topic info /Odometry
ros2 node list
```

`/livox/lidar` と `/livox/imu` にpublisherが必要です。初期姿勢を送った後、`/cloud_registered` と `/Odometry` が出力され、launchログに `Fitness score` が表示されます。

### LiDAR 点群が回転して見える

LiDAR点群が回転して見える場合は、IMUの軸、LiDARとIMUの取り付け方向、初期化時に機体が動いていなかったかを確認します。

```text
正しい: odom → body → livox_frame
誤り:   odom → livox_frame
```

ロボット停止中に`odom → body`の姿勢が変化する場合は、IMUの軸・取り付け方向・初期化状態を確認します。

### `extrapolation` が出る

LiDAR、オドメトリ、TF の時刻を Isaac Sim のシミュレーション時刻に統一します。起動直後の軽微な警告は、TF が蓄積するまで発生することがあります。

### 古い設定が起動する

次の環境を使っていることを確認します。

```bash
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

RViz2は[rviz/fastlio_localization.rviz](rviz/fastlio_localization.rviz)を使用します。

## トピック一覧

| トピック | 型 | 役割 |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` または `livox_ros_driver2/msg/CustomMsg` | Isaac Simまたは実機Livoxの入力 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Isaac Simまたは実機LivoxのIMU入力 |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | FAST-LIO後の現在点群 |
| `/Odometry` | `nav_msgs/msg/Odometry` | FAST-LIOのオドメトリ |
| `/map` | `sensor_msgs/msg/PointCloud2` | 事前地図 |
| `/cur_scan_in_map` | `sensor_msgs/msg/PointCloud2` | 現在スキャンの表示用点群 |
| `/submap` | `sensor_msgs/msg/PointCloud2` | ICP 用に切り出した地図 |
| `/map_to_odom` | `nav_msgs/msg/Odometry` | 地図から odom への補正 |
| `/localization` | `nav_msgs/msg/Odometry` | 補正後のロボット位置 |

## 関連プロジェクト

- [FAST-LIO](https://github.com/hku-mars/FAST_LIO)
- [FAST-LIO-ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree)

## 謝辞

本パッケージは FAST-LIO および Fast-Lio-Localization の成果をもとにしています。
