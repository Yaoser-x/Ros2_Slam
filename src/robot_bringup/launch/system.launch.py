#!/usr/bin/env python3

import os
import re
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from robot_config.launch_config import load_launch_config


def _package_exists(pkg_name: str) -> bool:
    try:
        get_package_share_directory(pkg_name)
        return True
    except Exception:
        return False


def _read_lidar_port_hint(params_file: str) -> str:
    if not params_file or not os.path.isfile(params_file):
        return ""
    try:
        with open(params_file, "r", encoding="utf-8") as handle:
            for line in handle:
                match = re.match(r'^\s*port\s*:\s*["\']?([^"\']+)["\']?\s*$', line)
                if match:
                    return match.group(1).strip()
    except OSError:
        return ""
    return ""


def _include(share: str, launch_name: str, arguments: dict):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", launch_name)),
        launch_arguments=arguments.items(),
    )


def _validate_and_compose(context):
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    if namespace:
        raise RuntimeError("nonempty namespace is not supported by the current platform")
    mode = LaunchConfiguration("mode").perform(context)
    mapping_source = LaunchConfiguration("mapping_source").perform(context)
    use_rviz = LaunchConfiguration("use_rviz")
    use_base_legacy = LaunchConfiguration("use_base").perform(context).lower() == "true"
    base_mode = LaunchConfiguration("base_mode").perform(context).strip().lower()
    base_fusion_mode = LaunchConfiguration("base_fusion_mode").perform(context).strip().lower()
    fallback_static_odom = (
        LaunchConfiguration("fallback_static_odom").perform(context).lower() == "true"
    )
    map_file = LaunchConfiguration("map_file").perform(context)
    lidar_enabled = LaunchConfiguration("use_lidar").perform(context).lower() == "true"
    lidar_params_file = LaunchConfiguration("lidar_params_file").perform(context)
    lidar_port_hint = _read_lidar_port_hint(lidar_params_file) if lidar_enabled else ""
    auto_mapping_drive = (
        LaunchConfiguration("auto_mapping_drive").perform(context).lower() == "true"
    )
    base_port_value = LaunchConfiguration("base_port").perform(context).strip()
    nav2_start = LaunchConfiguration("nav2_start").perform(context).lower() == "true"
    effective_params_file = LaunchConfiguration("effective_params_file").perform(context).strip()
    slam_params_file = LaunchConfiguration("slam_params_file").perform(context).strip()
    nav2_params_file = LaunchConfiguration("nav2_params_file").perform(context).strip()

    if mode not in ("mapping", "navigation", "base_only"):
        raise RuntimeError("Invalid 'mode'. Use 'mapping', 'navigation' or 'base_only'.")
    if mapping_source != "lidar":
        raise RuntimeError("Invalid 'mapping_source'. Only 'lidar' is supported.")
    if base_mode not in ("real", "fake", "none"):
        raise RuntimeError("Invalid 'base_mode'. Use 'real', 'fake' or 'none'.")
    if base_fusion_mode and base_fusion_mode not in ("none", "ekf", "wheel", "wheel_imu"):
        raise RuntimeError("Invalid 'base_fusion_mode'. Use 'wheel' or 'wheel_imu'.")
    if auto_mapping_drive:
        raise RuntimeError(
            "auto_mapping_drive is legacy. frontier_explorer lives in experiments/legacy and is not part of platform bringup."
        )

    if base_mode == "real" and not use_base_legacy:
        base_mode = "none"

    real_base_enabled = base_mode == "real"
    fake_base_enabled = base_mode == "fake"
    no_base = base_mode == "none"
    if mode == "base_only" and no_base:
        raise RuntimeError("base_only requires a real or fake base provider")

    effective = calibration = None
    if effective_params_file:
        effective, calibration = load_launch_config(effective_params_file)
        compiled_port = effective["hardware"]["base_port"]
        compiled_fusion = effective["runtime"]["fusion_mode"]
        fusion_aliases = {"none": "wheel", "ekf": "wheel_imu"}
        if base_port_value and base_port_value != compiled_port:
            raise RuntimeError("base_port differs from compiled config; override through bin/robot")
        if (
            base_fusion_mode
            and fusion_aliases.get(base_fusion_mode, base_fusion_mode) != compiled_fusion
        ):
            raise RuntimeError(
                "base_fusion_mode differs from compiled config; override through bin/robot"
            )
        base_port_value, base_fusion_mode = compiled_port, compiled_fusion
        if "base_mode" in effective["runtime"] and base_mode != effective["runtime"]["base_mode"]:
            raise RuntimeError("base_mode differs from compiled config; override through bin/robot")
        directory = os.path.dirname(effective_params_file)
        slam_artifact = {
            "quality": "slam-params.yaml",
            "precision": "slam-precision-params.yaml",
            "fast": "slam-fast-params.yaml",
        }[effective["runtime"].get("slam_profile", "quality")]
        for name, explicit, artifact in (
            ("slam_params_file", slam_params_file, slam_artifact),
            ("nav2_params_file", nav2_params_file, "nav2-params.yaml"),
        ):
            if explicit and os.path.realpath(explicit) != os.path.realpath(
                os.path.join(directory, artifact)
            ):
                raise RuntimeError(f"{name} must be the compiled artifact")
        slam_params_file = os.path.join(directory, slam_artifact)
        nav2_params_file = os.path.join(directory, "nav2-params.yaml")
        if lidar_enabled and os.path.realpath(lidar_params_file) != os.path.realpath(
            os.path.join(os.path.dirname(effective_params_file), "lidar-params.yaml")
        ):
            raise RuntimeError("lidar params must be the compiled artifact")
        for names, values in (
            (("x", "y", "z"), calibration["lidar"]["translation"]),
            (("roll", "pitch", "yaw"), calibration["lidar"]["rotation"]),
        ):
            for name, compiled_value in zip(names, values):
                explicit = LaunchConfiguration(f"lidar_tf_{name}").perform(context).strip()
                if explicit:
                    try:
                        matches = math.isclose(float(explicit), compiled_value, abs_tol=1e-12)
                    except ValueError:
                        matches = False
                    if not matches:
                        raise RuntimeError(
                            f"lidar_tf_{name} differs from compiled config; override through bin/robot"
                        )

    if real_base_enabled or fake_base_enabled:
        if not effective_params_file or not os.path.isfile(effective_params_file):
            raise RuntimeError(
                "base providers require compiled effective params; start through bin/robot"
            )
    if real_base_enabled:
        if base_port_value.lower() != "auto" and not os.path.exists(base_port_value):
            raise RuntimeError(f"base_port does not exist: {base_port_value}")
        if (
            base_port_value.lower() != "auto"
            and lidar_port_hint
            and os.path.realpath(base_port_value) == os.path.realpath(lidar_port_hint)
        ):
            raise RuntimeError(f"base and lidar cannot share one serial port: {base_port_value}")

    this_share = get_package_share_directory("robot_bringup")
    description_share = get_package_share_directory("robot_description")
    sensing_share = get_package_share_directory("robot_sensing")
    control_share = get_package_share_directory("robot_control")
    state_share = get_package_share_directory("robot_state_estimation")
    bridge_share = get_package_share_directory("stm32_robot_bridge")
    supervision_share = get_package_share_directory("robot_supervision")
    verification_share = get_package_share_directory("robot_verification")

    actions = [
        _include(
            description_share,
            "description.launch.py",
            {
                "use_lidar": "true" if lidar_enabled else "false",
                **(
                    {
                        "base_frame": calibration["frames"]["base"],
                        "imu_frame": calibration["frames"]["imu"],
                        "laser_frame": calibration["frames"]["laser"],
                        **{
                            f"lidar_tf_{name}": str(calibration["lidar"]["translation"][index])
                            for index, name in enumerate(("x", "y", "z"))
                        },
                        **{
                            f"lidar_tf_{name}": str(calibration["lidar"]["rotation"][index])
                            for index, name in enumerate(("roll", "pitch", "yaw"))
                        },
                    }
                    if calibration
                    else {
                        f"lidar_tf_{name}": (
                            LaunchConfiguration(f"lidar_tf_{name}").perform(context).strip()
                            or default
                        )
                        for name, default in zip(
                            ("x", "y", "z", "roll", "pitch", "yaw"),
                            ("0.07", "0.0", "0.13", "0.0", "0.0", "1.570796326795"),
                        )
                    }
                ),
            },
        ),
        _include(
            sensing_share,
            "lidar.launch.py",
            {
                "use_lidar": "true" if lidar_enabled else "false",
                "lidar_params_file": LaunchConfiguration("lidar_params_file"),
                "lidar_raw_scan_topic": LaunchConfiguration("lidar_raw_scan_topic"),
                "lidar_scan_topic": LaunchConfiguration("lidar_scan_topic"),
                "lidar_scan_output_size": LaunchConfiguration("lidar_scan_output_size"),
                "lidar_scan_angle_min": LaunchConfiguration("lidar_scan_angle_min"),
                "lidar_scan_angle_max": LaunchConfiguration("lidar_scan_angle_max"),
                "lidar_scan_normalizer_log_interval_sec": LaunchConfiguration(
                    "lidar_scan_normalizer_log_interval_sec"
                ),
            },
        ),
    ]

    if real_base_enabled or fake_base_enabled:
        actions.append(
            Node(
                package="robot_config",
                executable="compatibility_node",
                name="platform_compatibility",
                parameters=[LaunchConfiguration("effective_params_file")],
                output="screen",
            )
        )
        actions.append(
            Node(
                package="robot_chassis_ops",
                executable="chassis_ops_node",
                name="chassis_ops",
                parameters=[
                    effective_params_file,
                    {"wire_adapter_name": "fake_base" if fake_base_enabled else "stm32_bridge"},
                ],
                output="screen",
            )
        )
        actions.append(
            _include(
                control_share,
                "control.launch.py",
                {"params_file": LaunchConfiguration("effective_params_file")},
            )
        )

    if real_base_enabled:
        actions.append(
            _include(
                bridge_share,
                "stm32_bridge.launch.py",
                {
                    "params_file": LaunchConfiguration("effective_params_file"),
                    "port": base_port_value,
                },
            )
        )

    if fake_base_enabled:
        actions.append(
            _include(
                verification_share,
                "fake_base.launch.py",
                {
                    "params_file": LaunchConfiguration("effective_params_file"),
                    "scenario": LaunchConfiguration("fake_base_scenario"),
                },
            )
        )

    if real_base_enabled or fake_base_enabled:
        actions.append(
            _include(
                state_share,
                "state_estimation.launch.py",
                {
                    "base_fusion_mode": base_fusion_mode,
                    "effective_params_file": LaunchConfiguration("effective_params_file"),
                    "odom_topic": effective["platform"]["topics"]["odom"],
                    "ekf_wheel_imu_params_file": os.path.join(
                        os.path.dirname(effective_params_file), "ekf-wheel-imu.yaml"
                    ),
                },
            )
        )
        actions.append(
            _include(
                supervision_share,
                "supervision.launch.py",
                {"params_file": LaunchConfiguration("effective_params_file")},
            )
        )

    if mode == "mapping" and no_base and fallback_static_odom:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="odom_fallback_static_tf",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    effective["platform"]["frames"]["odom"] if effective else "odom",
                    effective["platform"]["frames"]["base"] if effective else "base_link",
                ],
                output="screen",
            )
        )

    if mode == "mapping":
        actions.append(LogInfo(msg=f"[robot_bringup] mode=mapping, base_mode={base_mode}"))
        if not _package_exists("slam_toolbox"):
            raise RuntimeError("Missing package: slam_toolbox. Install ros-humble-slam-toolbox.")
        actions.append(
            _include(
                this_share,
                "slam.launch.py",
                {
                    "slam_params_file": slam_params_file
                    or os.path.join(this_share, "config", "slam_toolbox_mapping.yaml"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "scan_topic": LaunchConfiguration("lidar_scan_topic"),
                },
            )
        )
    elif mode == "navigation":
        actions.append(LogInfo(msg=f"[robot_bringup] mode=navigation, base_mode={base_mode}"))
        if no_base:
            raise RuntimeError("Navigation mode requires base_mode:=real or base_mode:=fake.")
        if not _package_exists("nav2_bringup"):
            raise RuntimeError(
                "Missing package: nav2_bringup. Install ros-humble-navigation2 ros-humble-nav2-bringup."
            )
        if map_file == "" or not os.path.isfile(map_file):
            raise RuntimeError("Navigation mode requires a valid 'map_file' (*.yaml).")
        actions.append(
            Node(
                package="robot_navigation_guard",
                executable="navigation_guard_node",
                name="navigation_guard",
                output="screen",
                parameters=[effective_params_file],
            )
        )
        actions.append(
            _include(
                this_share,
                "localization.launch.py",
                {
                    "map_file": LaunchConfiguration("map_file"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "params_file": nav2_params_file,
                    "autostart": LaunchConfiguration("nav2_autostart"),
                },
            )
        )
        if nav2_start:
            actions.append(
                _include(
                    this_share,
                    "nav2.launch.py",
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "params_file": nav2_params_file,
                        "default_bt_xml_filename": LaunchConfiguration("nav2_bt_xml_file"),
                        "autostart": LaunchConfiguration("nav2_autostart"),
                        "cmd_vel_topic": "cmd_vel/nav",
                    },
                )
            )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(this_share, "launch", "viz.launch.py")),
            condition=IfCondition(use_rviz),
            launch_arguments={"rviz_config": LaunchConfiguration("rviz_config")}.items(),
        )
    )
    if namespace:
        return [GroupAction(actions=[PushRosNamespace(namespace), *actions])]
    return actions


def generate_launch_description():
    rb_share = get_package_share_directory("robot_bringup")
    sensing_share = get_package_share_directory("robot_sensing")
    state_share = get_package_share_directory("robot_state_estimation")

    default_nav2_params = os.environ.get(
        "ROBOT_NAV2_PARAMS", os.path.join(rb_share, "config", "nav2_params_robot.yaml")
    )
    if not os.path.isfile(default_nav2_params):
        raise RuntimeError(f"Canonical Nav2 params are missing: {default_nav2_params}")

    effective_default = os.environ.get("ROBOT_EFFECTIVE_PARAMS", "")
    return LaunchDescription(
        [
            DeclareLaunchArgument("effective_params_file", default_value=effective_default),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("mode", default_value="mapping"),
            DeclareLaunchArgument("mapping_source", default_value="lidar"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            DeclareLaunchArgument("base_mode", default_value="real"),
            DeclareLaunchArgument("use_base", default_value="true"),
            DeclareLaunchArgument("fallback_static_odom", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "lidar_params_file",
                default_value=os.environ.get(
                    "ROBOT_LIDAR_PARAMS",
                    os.path.join(sensing_share, "config", "ydlidar_x2.yaml"),
                ),
            ),
            DeclareLaunchArgument("lidar_raw_scan_topic", default_value="scan_raw"),
            DeclareLaunchArgument("lidar_scan_topic", default_value="scan"),
            DeclareLaunchArgument("lidar_scan_output_size", default_value="425"),
            DeclareLaunchArgument("lidar_scan_angle_min", default_value=str(-3.141592653589793)),
            DeclareLaunchArgument("lidar_scan_angle_max", default_value=str(3.141592653589793)),
            DeclareLaunchArgument("lidar_scan_normalizer_log_interval_sec", default_value="30.0"),
            DeclareLaunchArgument("lidar_tf_x", default_value=""),
            DeclareLaunchArgument("lidar_tf_y", default_value=""),
            DeclareLaunchArgument("lidar_tf_z", default_value=""),
            DeclareLaunchArgument("lidar_tf_roll", default_value=""),
            DeclareLaunchArgument("lidar_tf_pitch", default_value=""),
            DeclareLaunchArgument("lidar_tf_yaw", default_value=""),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value="",
            ),
            DeclareLaunchArgument("map_file", default_value=""),
            DeclareLaunchArgument("auto_mapping_drive", default_value="false"),
            DeclareLaunchArgument(
                "auto_mapping_nav2_params_file",
                default_value=os.path.join(rb_share, "config", "nav2_mapping_params.yaml"),
            ),
            DeclareLaunchArgument("auto_mapping_max_duration_sec", default_value="180.0"),
            DeclareLaunchArgument("auto_mapping_frontier_min_distance", default_value="0.70"),
            DeclareLaunchArgument("auto_mapping_frontier_max_distance", default_value="2.5"),
            DeclareLaunchArgument("auto_mapping_min_frontier_cluster_cells", default_value="10"),
            DeclareLaunchArgument("auto_mapping_goal_approach_offset", default_value="0.55"),
            DeclareLaunchArgument("auto_mapping_goal_approach_max_offset", default_value="0.90"),
            DeclareLaunchArgument("auto_mapping_goal_approach_step", default_value="0.05"),
            DeclareLaunchArgument("auto_mapping_goal_clearance_radius", default_value="0.38"),
            DeclareLaunchArgument(
                "auto_mapping_goal_unknown_clearance_radius", default_value="0.35"
            ),
            DeclareLaunchArgument("auto_mapping_linear_speed", default_value="0.10"),
            DeclareLaunchArgument("auto_mapping_turn_speed", default_value="1.65"),
            DeclareLaunchArgument("auto_mapping_emergency_stop_distance", default_value="0.25"),
            DeclareLaunchArgument("auto_mapping_front_stop_distance", default_value="0.40"),
            DeclareLaunchArgument("auto_mapping_front_resume_distance", default_value="0.48"),
            DeclareLaunchArgument("auto_mapping_front_slow_distance", default_value="0.80"),
            DeclareLaunchArgument(
                "auto_mapping_side_emergency_stop_distance", default_value="0.12"
            ),
            DeclareLaunchArgument("auto_mapping_side_stop_distance", default_value="0.10"),
            DeclareLaunchArgument("auto_mapping_side_resume_distance", default_value="0.18"),
            DeclareLaunchArgument("base_port", default_value=""),
            DeclareLaunchArgument("base_fusion_mode", default_value=""),
            DeclareLaunchArgument("fake_base_scenario", default_value="normal"),
            DeclareLaunchArgument(
                "base_ekf_params_file",
                default_value=os.path.join(state_share, "config", "ekf_base.yaml"),
            ),
            DeclareLaunchArgument("nav2_params_file", default_value=""),
            DeclareLaunchArgument(
                "nav2_bt_xml_file",
                default_value=os.path.join(
                    rb_share, "behavior_trees", "navigate_to_pose_recovery.xml"
                ),
            ),
            DeclareLaunchArgument("nav2_autostart", default_value="true"),
            DeclareLaunchArgument("nav2_start", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config", default_value=os.path.join(rb_share, "rviz", "system.rviz")
            ),
            OpaqueFunction(function=_validate_and_compose),
        ]
    )
