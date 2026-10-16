#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robot_config.launch_config import load_launch_config


def _compose(context):
    params_file = LaunchConfiguration("params_file").perform(context).strip()
    allow_uncompiled = (
        LaunchConfiguration("allow_uncompiled_config").perform(context).lower() == "true"
    )
    parameters = []
    publication_mode = "ASYNCHRONOUS"
    if params_file:
        if not os.path.isfile(params_file):
            raise RuntimeError(f"effective ROS params file does not exist: {params_file}")
        parameters.append(params_file)
        effective, _ = load_launch_config(params_file)
        publication_mode = effective["hardware"]["bridge_publication_mode"]
    elif not allow_uncompiled:
        raise RuntimeError("effective params are required; start through bin/robot")
    parameters.append({"port": LaunchConfiguration("port")})
    return [
        Node(
            package="stm32_robot_bridge",
            executable="bridge_node",
            name="stm32_bridge",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            parameters=parameters,
            additional_env={
                "RMW_FASTRTPS_PUBLICATION_MODE": publication_mode,
                "RMW_FASTRTPS_USE_QOS_FROM_XML": "0",
            },
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value=os.environ.get("ROBOT_EFFECTIVE_PARAMS", "")
            ),
            DeclareLaunchArgument("port", default_value="/dev/serial0"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("allow_uncompiled_config", default_value="false"),
            OpaqueFunction(function=_compose),
        ]
    )
