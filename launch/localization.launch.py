from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_path = get_package_share_directory("fast_lio_localization")
    default_config_path = os.path.join(package_path, "config")
    default_rviz_config_path = os.path.join(package_path, "rviz", "fastlio_localization.rviz")
    default_map_path = os.path.join(package_path, "maps", "robocon2026_field.pcd")

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_path = LaunchConfiguration("config_path")
    config_file = LaunchConfiguration("config_file")
    rviz_use = LaunchConfiguration("rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    pcd_map_topic = LaunchConfiguration("pcd_map_topic")
    pcd_map_path = LaunchConfiguration("map")
    lidar_mode = LaunchConfiguration("lidar_mode")
    lidar_topic = LaunchConfiguration("lidar_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    map_is_set = PythonExpression(["'", pcd_map_path, "' != ''"])
    lidar_type = PythonExpression(["1 if '", lidar_mode, "' == 'livox' else 4"])

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        "config_path", default_value=default_config_path, description="Yaml config file path"
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        "config_file", default_value="mid360.yaml", description="Config file"
    )
    declare_rviz_cmd = DeclareLaunchArgument("rviz", default_value="true", description="Use RViz to monitor results")

    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        "rviz_cfg", default_value=default_rviz_config_path, description="RViz config file path"
    )

    declare_map_path = DeclareLaunchArgument(
        "map",
        default_value=default_map_path,
        description="Path to PCD map file; defaults to the map bundled with this package",
    )
    declare_pcd_map_topic = DeclareLaunchArgument(
        "pcd_map_topic", default_value="/map", description="Topic to publish PCD map"
    )
    declare_lidar_mode = DeclareLaunchArgument(
        "lidar_mode", default_value="isaac", description="isaac=PointCloud2, livox=CustomMsg"
    )
    declare_lidar_topic = DeclareLaunchArgument(
        "lidar_topic", default_value="/livox/lidar", description="PointCloud2 input topic in Isaac mode"
    )
    declare_imu_topic = DeclareLaunchArgument(
        "imu_topic", default_value="/livox/imu", description="IMU input topic"
    )
    declare_odom_topic = DeclareLaunchArgument(
        "odom_topic", default_value="/Odometry", description="Odometry topic used by localization and fusion"
    )
    # Load parameters from yaml file

    # FAST-LIO uses Livox CustomMsg in livox mode and PointCloud2 in Isaac mode.
    fast_lio_node = Node(
        package="fast_lio_localization",
        executable="fastlio_mapping",
        parameters=[PathJoinSubstitution([config_path, config_file]), {
            "use_sim_time": use_sim_time,
            "preprocess.lidar_type": lidar_type,
            "common.lid_topic": lidar_topic,
            "common.imu_topic": imu_topic,
        }],
        output="screen",
    )

    # Global localization node
    global_localization_node = Node(
        package="fast_lio_localization",
        executable="global_localization.py",
        name="global_localization",
        output="screen",
        parameters=[{"map_voxel_size": 0.4,
                     "scan_voxel_size": 0.1,
                     "freq_localization": 0.5,
                     "freq_global_map": 0.25,
                     "localization_threshold": 0.5,
                     "max_height": 2.2,
                     "fov": 6.28319,
                     "fov_far": 300,
                     "pcd_map_path": pcd_map_path,
                     "pcd_map_topic": pcd_map_topic,
                     "lidar_topic": "/cloud_registered",
                     "odom_topic": odom_topic,
                     "use_sim_time": use_sim_time}],
    )

    # Transform fusion node
    transform_fusion_node = Node(
        package="fast_lio_localization",
        executable="transform_fusion.py",
        name="transform_fusion",
        output="screen",
        parameters=[{"odom_topic": odom_topic, "use_sim_time": use_sim_time}],
    )

    lidar_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_livox_frame",
        arguments=["-0.011", "-0.02329", "0.04412", "0", "0", "0", "body", "livox_frame"],
        output="screen",
    )
    
    # PCD to PointCloud2 publisher
    pcd_publisher_node = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="map_publisher",
        output="screen",
        parameters=[{"file_name": pcd_map_path,
                     "tf_frame": "map",
                    "cloud_topic": pcd_map_topic, "use_sim_time": use_sim_time,
                    "period_ms_": 500}],
        remappings=[
            ("cloud_pcd", pcd_map_topic),
        ],
        condition=IfCondition(map_is_set),
    )

    rviz_node = Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg], condition=IfCondition(rviz_use), parameters=[{"use_sim_time": use_sim_time}])

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_map_path)
    ld.add_action(declare_pcd_map_topic)
    ld.add_action(declare_lidar_mode)
    ld.add_action(declare_lidar_topic)
    ld.add_action(declare_imu_topic)
    ld.add_action(declare_odom_topic)

    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)
    ld.add_action(global_localization_node)
    ld.add_action(transform_fusion_node)
    ld.add_action(lidar_tf_node)
    ld.add_action(pcd_publisher_node)

    return ld
