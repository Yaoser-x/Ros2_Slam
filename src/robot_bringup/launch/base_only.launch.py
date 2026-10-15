"""Use the complete platform chain without lidar, SLAM or navigation."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("robot_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "effective_params_file", default_value=os.environ.get("ROBOT_EFFECTIVE_PARAMS", "")
            ),
            DeclareLaunchArgument(
                "base_mode", default_value=os.environ.get("ROBOT_BASE_MODE", "real")
            ),
            DeclareLaunchArgument("base_port", default_value=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "system.launch.py")),
                launch_arguments={
                    "mode": "base_only",
                    "use_lidar": "false",
                    "use_rviz": "false",
                    "fallback_static_odom": "false",
                    "use_base": "true",
                    "base_mode": LaunchConfiguration("base_mode"),
                    "base_port": LaunchConfiguration("base_port"),
                    "effective_params_file": LaunchConfiguration("effective_params_file"),
                }.items(),
            ),
        ]
    )
