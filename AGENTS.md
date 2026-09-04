# Agent Customization for FAST-LIO-LOCALIZATION2

This is a ROS2-based 3D lidar localization framework. It provides real-time global localization in pre-built point cloud maps by fusing low-frequency global localization with high-frequency odometry from FAST-LIO.

## Architecture Overview

### Core Components

1. **C++ Laser Mapping Node** (`src/laserMapping.cpp`)
   - High-performance lidar odometry and mapping based on the LOAM algorithm
   - Uses iKD-Tree for efficient spatial indexing of point clouds
   - Compiles to executable: `fastlio_mapping`
   - Key classes: IMU processing, point cloud preprocessing, incremental mapping
   - Thread-safe with OpenMP parallelization

2. **Python Global Localization Node** (`fast_lio_localization/global_localization.py`)
   - Performs global localization by matching current scans to a pre-built global map
   - Uses Open3D for voxel-based point cloud alignment (ICP-like matching)
   - Publishes transform between map frame and odometry frame
   - Configurable voxel sizes and localization frequency

3. **Transform Fusion Node** (`fast_lio_localization/transform_fusion.py`)
   - Fuses global localization pose with odometry estimates
   - Publishes corrected transform to eliminate accumulative odometry drift

4. **Launch System** (`launch/`)
   - `localization.launch.py`: Main launch file orchestrating all nodes
   - Supports both Livox and standard LiDAR (Velodyne, Ouster)
   - Configurable via launch arguments (map path, lidar config, RVIZ, etc.)

### Data Flow

```
ROS2 Topic: /livox_lidar or /scan
    ↓
fastlio_mapping (C++) → /odometry/imu, /cloud_registered
    ↓
global_localization.py → /map_to_odom (global pose)
    ↓
transform_fusion.py → /tf (map→base_link via odometry fusion)
    ↓
RVIZ visualization
```

## Build & Development

### Build Commands

```bash
# From ROS2 workspace root
colcon build --symlink-install                    # Full build
colcon build --packages-select fast_lio_localization --symlink-install  # Single package
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug   # Debug build
```

### Prerequisites

- **System**: Ubuntu 20.04+ with ROS2 (Foxy through Jazzy supported)
- **C++ Standard**: C++17 with OpenMP support
- **Python Version**: 3.8+ (check numpy < 1.24)
- **Key Libraries**: PCL, Eigen3, Open3D, livox_ros_driver2

### Important Setup Notes

1. **NumPy Compatibility Issue**: Replace `np.float` → `np.float64` in `/usr/lib/python3/dist-packages/transforms3d/quaternions.py` (documented in README)
2. **Submodules**: Run `git submodule update --init` before first build (for iKD-Tree)
3. **Custom PCL**: Set `export PCL_ROOT={PATH}` in `.bashrc` if using custom PCL build
4. **Livox Driver**: Must source livox_ros_driver before building

## Common Development Tasks

### Adding Support for a New LiDAR Type

1. Create new YAML config in `config/{lidar_name}.yaml` (copy from similar existing config)
2. Update point cloud preprocessing logic in `src/preprocess.cpp` if needed
3. Update launch file parameter defaults or documentation

### Modifying Localization Parameters

- Edit `launch/localization.launch.py`: Adjust node parameters (voxel sizes, frequencies, thresholds)
- Key parameters:
  - `map_voxel_size`: Voxel size for global map downsampling (default 0.4m)
  - `scan_voxel_size`: Voxel size for current scan (default 0.1m)
  - `freq_localization`: Global localization update frequency (Hz, default 0.5)
  - `localization_threshold`: ICP matching quality threshold (0-1, default 0.8)

### Performance Optimization

- **C++ side**: Adjust `MP_PROC_NUM` in CMakeLists.txt (multiprocessing cores)
- **Python side**: Voxel sizes directly affect performance; larger voxel sizes = faster but less accurate
- **Memory**: Use downsampled PCD maps for RVIZ visualization (noted in README)

## Code Conventions

### File Organization

```
src/                          # C++ source files
├── laserMapping.cpp         # Main mapping algorithm
├── preprocess.cpp           # LiDAR preprocessing
├── preprocess.h
└── IMU_Processing.hpp       # IMU preprocessing

fast_lio_localization/        # Python package
├── __init__.py
├── global_localization.py   # Global localization node
├── transform_fusion.py      # Transform fusion
├── publish_initial_pose.py
└── invert_livox_scan.py

include/                      # Header files and libraries
├── ikd-Tree/               # Third-party spatial indexing (submodule)
├── IKFoM_toolkit/          # Iterated Kalman Filter on Manifold
└── *.h                     # Utility headers (math, logging)

config/                       # Configuration files
├── mid360.yaml            # Livox Mid360 config (default)
├── velodyne.yaml
└── ...

msg/                         # ROS2 message definitions
└── Pose6D.msg            # 6D pose message (x,y,z, roll,pitch,yaw)

launch/                       # Launch scripts
└── localization.launch.py   # Main entry point
```

### Python Node Structure

- Inherit from `rclpy.node.Node`
- Declare ROS2 parameters in `__init__` with defaults
- Use `self.get_logger()` for logging
- Use `tf2_ros` for transform management (publish to `/tf`)
- Handle threading for multi-rate callbacks

### C++ Compilation Flags

- C++17 with `-O3` optimization enabled by default
- OpenMP enabled for parallelization
- Architecture-aware compilation (MP_EN flag for multi-core systems)

## Testing & Debugging

### Typical Workflow

1. **Build**: `colcon build --symlink-install`
2. **Source**: `. install/setup.bash`
3. **Launch**: `ros2 launch fast_lio_localization localization.launch.py pcd_map_topic:=/map_cloud map:=/path/to/map.pcd`
4. **Visualize**: RVIZ2 opens automatically with pre-configured layout
5. **Monitor**: Check node logs for errors; topics published to `/map_to_odom`, `/odometry/imu`

### Debugging Tips

- **Transform issues**: Check `/tf` tree with `ros2 run tf2_tools view_frames`
- **Localization failures**: Monitor `/cur_scan_in_map` topic; should show reasonable alignment
- **Performance**: Check CPU/memory with standard ROS2 tools; watch for ICP convergence time
- **Python errors**: Often related to numpy version or Open3D installation (check README)

## Key Dependencies & Versions

| Dependency | Version | Purpose |
|------------|---------|---------|
| ROS2 | Humble, Jazzy (Foxy+ supported) | Messaging framework |
| PCL | Latest | Point cloud processing |
| Eigen3 | Latest | Linear algebra |
| Open3D | Latest | ICP-based localization |
| livox_ros_driver2 | Latest | Livox LiDAR interface |
| numpy | < 1.24 | Data processing (constraint due to transforms3d) |

## Common Pitfalls & Solutions

1. **"Map not loaded" errors**: Ensure PCD file path is correct and map topic matches launch parameter `pcd_map_topic`
2. **NumPy float issues**: Always read the README section on NumPy compatibility before first Python execution
3. **Transform publishing conflicts**: Only one node should publish `map→odom` transform; check `transform_fusion.py` and global_localization.py don't conflict
4. **Memory spikes on large maps**: Downsampling with voxel filters is mandatory for dense PCDs
5. **Poor localization accuracy**: Increase `map_voxel_size` and `scan_voxel_size` equally to keep registration quality but reduce computation

## File Relationships & Imports

- `laserMapping.cpp` uses: `preprocess.h`, `IMU_Processing.hpp`, `ikd-Tree/`, `IKFoM_toolkit/`
- Python nodes use: `rclpy`, `tf2_ros`, `open3d`, `sensor_msgs`, `nav_msgs`, `transforms3d`
- All C++ code expects `ROOT_DIR` macro pointing to project root (set in CMakeLists.txt)

## References & Related Projects

- Original LOAM paper: J. Zhang and S. Singh, "LOAM: Lidar Odometry and Mapping in Real-time" (RSS 2014)
- FAST-LIO-ROS2: [https://github.com/Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)
- iKD-Tree: Efficient incremental 3D KD-tree for dynamic point clouds (included as submodule)
- Open3D Documentation: [https://www.open3d.org/](https://www.open3d.org/)
