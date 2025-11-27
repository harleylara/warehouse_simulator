#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share_path = get_package_share_directory('warehouse_simulator')
    pkg_install_path = get_package_prefix('warehouse_simulator')
    models_path = os.path.join(pkg_share_path, 'models')
    lib_path = os.path.join(pkg_install_path, 'lib')

    # === Launch argument: GUI sí/no ===
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='If true, run Gazebo with GUI. If false, run server-only (-s).'
    )
    gui = LaunchConfiguration('gui')

    # === Launch argument: nombre de mundo SIN extensión ===
    #   Ejemplos de uso:
    #     world_name:=warehouse_full   -> warehouse_full.sdf
    #     world_name:=warehouse_02     -> warehouse_02.sdf
    #     world_name:=empty_with_robot -> empty_with_robot.sdf
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='warehouse_full',
        description='World name (without .sdf) located in the worlds/ folder.'
    )
    world_name = LaunchConfiguration('world_name')

    # Genera una expresión tipo: 'warehouse_full.sdf'
    world_file = PythonExpression([
        "'",
        world_name,
        ".sdf'"
    ])

    # Ruta absoluta al mundo dentro del paquete
    world_path = PathJoinSubstitution([
        pkg_share_path,
        'worlds',
        world_file
    ])

    yaml_path = os.path.join(pkg_share_path, 'config', 'actors_waypoints.yaml')
    rviz_config_path = os.path.join(pkg_share_path, 'viz', 'viz_warehouse.rviz')

    # === Environment variables ===
    set_yaml = SetEnvironmentVariable(
        name='WAREHOUSE_SIMULATOR_YAML',
        value=yaml_path
    )

    set_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value=os.pathsep.join(filter(None, [
            os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''),
            lib_path
        ]))
    )

    set_ign = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.pathsep.join(filter(None, [
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
            models_path
        ]))
    )

    set_gz = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(filter(None, [
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            models_path
        ]))
    )

    # Launch source for ros_gz_sim
    gz_sim_launch_source = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py'
        )
    )

    # === GAZEBO SIM with GUI ===
    gz_launch_gui = IncludeLaunchDescription(
        gz_sim_launch_source,
        condition=IfCondition(gui),
        launch_arguments={
            'gz_args': ['-r ', world_path]
        }.items()
    )

    # === GAZEBO SIM without GUI (headless/server-only) ===
    gz_launch_headless = IncludeLaunchDescription(
        gz_sim_launch_source,
        condition=UnlessCondition(gui),
        launch_arguments={
            'gz_args': ['-r -s ', world_path]
        }.items()
    )

    # ======= BRIDGE ROS2 <-> GAZEBO =======
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/robot/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/robot/lidar2d/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/robot/lidar3d/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/robot/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # ======= For TF =========
    static_tf_lidar2d = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar2d',
        arguments=[
            # x y z  roll pitch yaw   parent      child
            '0.42', '0.0', '0.415',  '0', '0', '0', 'base_link', 'lidar_2d_link'
        ]
    )

    static_tf_lidar3d = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar3d',
        arguments=[
            '-0.278', '0', '1.15',  '0', '0', '0', 'base_link', 'lidar_3d_link'
        ]
    )

    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_rgbd',
        arguments=[
            '-0.2', '0.0', '0.86',  '0', '0', '0', 'base_link', 'camera_rgbd_link'
        ]
    )

    return LaunchDescription([
        gui_arg,
        world_name_arg,
        set_yaml,
        set_plugin_path,
        set_ign,
        set_gz,
        gz_launch_gui,
        gz_launch_headless,
        bridge,
        rviz_node,
        static_tf_lidar2d,
        static_tf_lidar3d,
        static_tf_camera,
    ])
