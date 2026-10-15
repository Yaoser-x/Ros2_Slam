#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap
from nav2_common.launch import RewrittenYaml


class _BehaviorTreeParameters(RewrittenYaml):
    def substitute_params(self, data, param_rewrites):
        # Humble's RewrittenYaml only replaces existing YAML paths.
        data.setdefault("bt_navigator", {}).setdefault("ros__parameters", {}).setdefault(
            "default_nav_to_pose_bt_xml", ""
        )
        super().substitute_params(data, param_rewrites)


def generate_launch_description():
    nav2_share = get_package_share_directory("nav2_bringup")
    rb_share = get_package_share_directory("robot_bringup")
    default_bt_xml = os.path.join(rb_share, "behavior_trees", "navigate_to_pose_recovery.xml")
    configured_params = _BehaviorTreeParameters(
        source_file=LaunchConfiguration("params_file"),
        param_rewrites={
            "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml": LaunchConfiguration(
                "default_bt_xml_filename"
            ),
        },
        convert_types=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "params_file", default_value=os.path.join(nav2_share, "params", "nav2_params.yaml")
            ),
            DeclareLaunchArgument("default_bt_xml_filename", default_value=default_bt_xml),
            DeclareLaunchArgument("map_subscribe_transient_local", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="cmd_vel/nav"),
            GroupAction(
                actions=[
                    SetRemap(src="/cmd_vel", dst=LaunchConfiguration("cmd_vel_topic")),
                    SetRemap(src="navigate_to_pose", dst="nav2/navigate_to_pose"),
                    SetRemap(src="navigate_through_poses", dst="nav2/navigate_through_poses"),
                    SetRemap(src="goal_pose", dst="nav2/goal_pose"),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(nav2_share, "launch", "navigation_launch.py")
                        ),
                        launch_arguments={
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                            "params_file": configured_params,
                            "map_subscribe_transient_local": LaunchConfiguration(
                                "map_subscribe_transient_local"
                            ),
                            "autostart": LaunchConfiguration("autostart"),
                        }.items(),
                    ),
                ],
            ),
        ]
    )
