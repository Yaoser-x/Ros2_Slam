"""Humble configuration contracts and a real launch substitution regression."""

import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "src/robot_config/config/components/nav2/base.yaml"


def test_humble_motion_model_and_behavior_server_preserve_navigation_policy():
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    assert config["amcl"]["ros__parameters"]["robot_model_type"] == (
        "nav2_amcl::DifferentialMotionModel"
    )
    assert "recoveries_server" not in config
    behavior = config["behavior_server"]["ros__parameters"]
    assert behavior["behavior_plugins"] == ["spin", "backup", "wait"]
    assert [behavior[name]["plugin"] for name in behavior["behavior_plugins"]] == [
        "nav2_behaviors/Spin",
        "nav2_behaviors/BackUp",
        "nav2_behaviors/Wait",
    ]
    assert (behavior["global_frame"], behavior["robot_base_frame"]) == ("odom", "base_link")
    assert behavior["transform_tolerance"] == 0.3
    assert (behavior["max_rotational_vel"], behavior["min_rotational_vel"]) == (0.7, 0.2)
    assert behavior["rotational_acc_lim"] == 1.5
    assert config["lifecycle_manager_navigation"]["ros__parameters"]["node_names"] == [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]


def test_plugin_set_covers_custom_pose_tree_and_humble_default_through_poses_tree():
    params = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))["bt_navigator"]["ros__parameters"]
    libraries = set(params["plugin_lib_names"])
    custom = ET.parse(ROOT / "src/robot_bringup/behavior_trees/navigate_to_pose_recovery.xml")
    tags = {element.tag for element in custom.iter()}
    # Additional nodes in the installed Nav2 1.1.20 default ThroughPoses tree.
    tags.update({"RemovePassedGoals", "ComputePathThroughPoses", "ReactiveSequence"})
    builtins = {"root", "BehaviorTree", "Sequence", "ReactiveSequence", "ReactiveFallback"}
    plugins = {
        "RecoveryNode": "nav2_recovery_node_bt_node",
        "PipelineSequence": "nav2_pipeline_sequence_bt_node",
        "RateController": "nav2_rate_controller_bt_node",
        "ComputePathToPose": "nav2_compute_path_to_pose_action_bt_node",
        "ComputePathThroughPoses": "nav2_compute_path_through_poses_action_bt_node",
        "RemovePassedGoals": "nav2_remove_passed_goals_action_bt_node",
        "ClearEntireCostmap": "nav2_clear_costmap_service_bt_node",
        "FollowPath": "nav2_follow_path_action_bt_node",
        "GoalUpdated": "nav2_goal_updated_condition_bt_node",
        "RoundRobin": "nav2_round_robin_node_bt_node",
        "Spin": "nav2_spin_action_bt_node",
        "Wait": "nav2_wait_action_bt_node",
        "BackUp": "nav2_back_up_action_bt_node",
    }
    assert tags <= builtins | plugins.keys()
    assert {plugins[tag] for tag in tags - builtins} <= libraries
    assert "default_nav_to_pose_bt_xml" in params
    assert "default_bt_xml_filename" not in params


@pytest.mark.ros
@pytest.mark.parametrize("with_bt_placeholder", [False, True])
def test_real_humble_launch_rewrites_custom_bt_without_changing_other_parameters(
    tmp_path, with_bt_placeholder
):
    probe = subprocess.run(
        [sys.executable, "-c", "import launch; from nav2_common.launch import RewrittenYaml"],
        capture_output=True,
        text=True,
    )
    if probe.returncode and any(
        f"No module named '{name}'" in probe.stderr for name in ("launch", "nav2_common")
    ):
        pytest.skip("real ROS launch/Nav2 runtime unavailable; native launch check remains pending")
    assert probe.returncode == 0, probe.stderr
    params = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    navigator = params["bt_navigator"]["ros__parameters"]
    navigator.update(global_frame="test_map", robot_base_frame="test_base")
    if not with_bt_placeholder:
        navigator.pop("default_nav_to_pose_bt_xml")
    source = tmp_path / "params.yaml"
    source.write_text(yaml.safe_dump(params), encoding="utf-8")
    chosen_bt = str(ROOT / "src/robot_bringup/behavior_trees/navigate_to_pose_recovery.xml")
    code = """
import importlib.util
import json
from pathlib import Path
import sys
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import SetRemap
import yaml

spec = importlib.util.spec_from_file_location('tested_nav2_launch', sys.argv[1])
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
context = LaunchContext()
context.launch_configurations.update(params_file=sys.argv[2], default_bt_xml_filename=sys.argv[3])
description = module.generate_launch_description()
for action in description.entities:
    if isinstance(action, DeclareLaunchArgument):
        action.execute(context)
group = next(action for action in description.entities if isinstance(action, GroupAction))
for action in group.get_sub_entities():
    if isinstance(action, SetRemap):
        action.execute(context)
remappings = context.launch_configurations['ros_remaps']
assert ('navigate_to_pose', 'nav2/navigate_to_pose') in remappings
assert ('navigate_through_poses', 'nav2/navigate_through_poses') in remappings
assert ('goal_pose', 'nav2/goal_pose') in remappings
include = next(action for action in group.get_sub_entities() if isinstance(action, IncludeLaunchDescription))
arguments = dict(include.launch_arguments)
assert 'default_bt_xml_filename' not in arguments
rewritten = Path(perform_substitutions(
    context, normalize_to_list_of_substitutions(arguments['params_file'])
))
try:
    params = yaml.safe_load(rewritten.read_text())
    assert params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] == sys.argv[3]
    print(json.dumps(params))
finally:
    rewritten.unlink()
"""
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            code,
            str(ROOT / "src/robot_bringup/launch/nav2.launch.py"),
            str(source),
            chosen_bt,
        ],
        env=os.environ.copy(),
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    expected = params
    expected["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"] = chosen_bt
    assert json.loads(result.stdout) == expected
