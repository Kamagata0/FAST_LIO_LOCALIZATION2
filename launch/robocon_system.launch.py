import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("fast_lio_localization")
    default_config_path = os.path.join(pkg_dir, "config")
    default_map_path = os.path.join(pkg_dir, "maps", "robocon2026_field.pcd")

    use_sim_time = LaunchConfiguration("use_sim_time")
    lidar_topic = LaunchConfiguration("lidar_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    show_dashboard = LaunchConfiguration("dashboard")
    rviz = LaunchConfiguration("rviz")
    config_file = LaunchConfiguration("config_file")
    config_path = LaunchConfiguration("config_path")
    lidar_mode = LaunchConfiguration("lidar_mode")
    lidar_type = PythonExpression(["1 if '", lidar_mode, "' == 'livox' else 4"])

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation/bag clock if true"
    )
    declare_lidar_mode = DeclareLaunchArgument(
        "lidar_mode", default_value="livox", description="livox=CustomMsg, isaac=PointCloud2"
    )
    declare_lidar_topic = DeclareLaunchArgument(
        "lidar_topic", default_value="/livox/lidar", description="LiDAR PointCloud2 or CustomMsg topic"
    )
    declare_imu_topic = DeclareLaunchArgument(
        "imu_topic", default_value="/livox/imu", description="IMU topic"
    )
    declare_config_path = DeclareLaunchArgument(
        "config_path", default_value=default_config_path, description="Config directory"
    )
    declare_config_file = DeclareLaunchArgument(
        "config_file", default_value="mid360.yaml", description="Config file"
    )
    declare_show_dashboard = DeclareLaunchArgument(
        "dashboard", default_value="true", description="Show 2D OpenCV HUD Dashboard window"
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch RViz2 if true"
    )

    # 1. FAST-LIO Laser Mapping Node (High-Frequency Odometry & Point Cloud Fusion)
    fast_lio_node = Node(
        package="fast_lio_localization",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        parameters=[PathJoinSubstitution([config_path, config_file]), {
            "use_sim_time": use_sim_time,
            "preprocess.lidar_type": lidar_type,
            "common.lid_topic": lidar_topic,
            "common.imu_topic": imu_topic,
        }],
        output="screen",
    )

    # 2. Transform Fusion Node (Map -> Body Global Pose & Trajectory)
    transform_fusion_node = Node(
        package="fast_lio_localization",
        executable="transform_fusion.py",
        name="transform_fusion",
        output="screen",
        parameters=[{
            "odom_topic": "/Odometry",
            "use_sim_time": use_sim_time,
        }],
    )

    # 3. Specialized Robocon Field Localization & Target Node
    field_loc_node = Node(
        package="fast_lio_localization",
        executable="field_localization_node.py",
        name="field_localization_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "lidar_topic": "/cloud_registered",
            "map_path": default_map_path,
            "max_range": 12.0,
            "min_height": 0.05,
            "max_height": 1.80,
            "publish_tf": False,  # TF handled cleanly by transform_fusion
        }]
    )

    # 4. 2D HUD Dashboard & Mechanism Data Server
    dashboard_node = Node(
        package="fast_lio_localization",
        executable="robot_dashboard_node.py",
        name="robot_dashboard_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "show_window": show_dashboard,
            "publish_image": True,
        }]
    )

    # 5. Static Transforms
    static_tf_base_to_livox = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_livox_frame",
        arguments=["0", "0", "0", "0", "0", "0", "body", "livox_frame"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 6. PCD Map Publisher (Publishes robocon2026_field.pcd to /map)
    pcd_map_node = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="map_publisher",
        output="screen",
        parameters=[{
            "file_name": default_map_path,
            "tf_frame": "map",
            "cloud_topic": "/map",
            "use_sim_time": use_sim_time,
            "period_ms_": 5000,
        }],
        remappings=[("cloud_pcd", "/map")],
    )

    # 7. RViz2 Visualizer
    rviz_cfg = os.path.join(pkg_dir, "rviz", "fastlio_localization.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_lidar_mode,
        declare_lidar_topic,
        declare_imu_topic,
        declare_config_path,
        declare_config_file,
        declare_show_dashboard,
        declare_rviz,
        static_tf_base_to_livox,
        pcd_map_node,
        fast_lio_node,
        transform_fusion_node,
        field_loc_node,
        dashboard_node,
        rviz_node,
    ])
