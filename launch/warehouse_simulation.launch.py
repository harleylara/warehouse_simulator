#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    pkg_share_path = get_package_share_directory('warehouse_simulator')
    pkg_install_path = get_package_prefix('warehouse_simulator')
    models_path = os.path.join(pkg_share_path, 'models')
    lib_path = os.path.join(pkg_install_path, 'lib')
    world_path = os.path.join(pkg_share_path, 'worlds', 'tugbot_warehouse.sdf')
    yaml_path = os.path.join(pkg_share_path, 'config', 'actors_waypoints.yaml')
    # os.environ['WAREHOUSE_SIMULATOR_YAML'] = yaml_path

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

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    return LaunchDescription([
        set_yaml,           # <- primero, para que exista cuando arranca gz_sim
        set_plugin_path,
        set_ign,
        set_gz,
        gz_launch
    ])
