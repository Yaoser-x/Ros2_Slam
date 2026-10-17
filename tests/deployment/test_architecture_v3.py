from pathlib import Path
import ast
import re

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]


def imports_historical_imu(code):
    for node in ast.walk(ast.parse(code)):
        if isinstance(node, ast.Import):
            if any(alias.name.split(".")[-1] == "imu_converter" for alias in node.names):
                return True
        elif isinstance(node, ast.ImportFrom):
            if (node.module or "").split(".")[-1] == "imu_converter":
                return True
            if node.module in (None, "stm32_robot_bridge") and any(
                alias.name == "imu_converter" for alias in node.names
            ):
                return True
    return False


@pytest.mark.parametrize("code", [
    "import stm32_robot_bridge.imu_converter as legacy",
    "from stm32_robot_bridge.imu_converter import classify_imu_quality",
    "from .imu_converter import ImuClockSynchronizer",
    "from . import imu_converter as legacy",
    "from stm32_robot_bridge import imu_converter",
])
def test_historical_estimator_import_guard_catches_aliases(code):
    assert imports_historical_imu(code)


def test_v3_runtime_does_not_import_historical_estimator():
    directory = ROOT / "src/stm32_robot_bridge/stm32_robot_bridge"
    for path in directory.glob("*.py"):
        if path.name != "imu_converter.py":
            assert not imports_historical_imu(path.read_text(encoding="utf-8")), path


def test_formal_odometry_is_only_dynamic_tf_owner():
    owners = []
    for path in (ROOT / "src").glob("*/**/*.py"):
        if "/test/" in path.as_posix() or path.name.startswith("test_"):
            continue
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            if isinstance(node, ast.Call) and (
                isinstance(node.func, ast.Name) and node.func.id == "TransformBroadcaster"
                or isinstance(node.func, ast.Attribute) and node.func.attr == "TransformBroadcaster"
            ):
                owners.append(path.relative_to(ROOT).as_posix())
    assert owners == ["src/robot_state_estimation/robot_state_estimation/formal_odometry_node.py"]


def test_robot_control_is_only_host_motion_command_publisher():
    publishers = []
    for path in (ROOT / "src").glob("*/**/*.py"):
        if "/test/" in path.as_posix() or path.name.startswith("test_"):
            continue
        if re.search(
            r"create_publisher\s*\(\s*HostMotionCommand\s*,",
            path.read_text(encoding="utf-8"),
        ):
            publishers.append(path.relative_to(ROOT).as_posix())
    assert publishers == ["src/robot_control/robot_control/cmd_vel_mux.py"]


def test_bridge_has_no_navigation_dependency_and_only_installs_v3_runtime():
    package = (ROOT / "src/stm32_robot_bridge/package.xml").read_text(encoding="utf-8")
    cmake = (ROOT / "src/stm32_robot_bridge/CMakeLists.txt").read_text(encoding="utf-8")
    assert "nav2" not in package and "slam_toolbox" not in package
    assert "ament_python_install_package" in cmake and "bridge_node_v3.py" in cmake
    module_directory = ROOT / "src/stm32_robot_bridge/stm32_robot_bridge"
    assert (module_directory / "protocol_v3.py").is_file()
    assert not (module_directory / "protocol_v2.py").exists()
    assert "protocol_v2.py" not in cmake and "bridge_node.py" not in cmake


def test_bridge_runtime_has_no_odometry_supervision_or_tf_ownership():
    node = (ROOT / "src/stm32_robot_bridge/stm32_robot_bridge/bridge_node_v3.py").read_text(
        encoding="utf-8"
    )
    for forbidden in (
        "EncoderOdometry",
        "MotionSupervisor",
        "nav_msgs",
        "TransformBroadcaster",
        "create_publisher(Odometry",
    ):
        assert forbidden not in node
    assert "WheelObservation" in node and "ImuObservation" in node


def test_fake_and_real_base_share_raw_platform_contract_and_publish_no_tf():
    real = (ROOT / "src/stm32_robot_bridge/stm32_robot_bridge/bridge_node_v3.py").read_text(
        encoding="utf-8"
    )
    fake = (ROOT / "src/robot_verification/robot_verification/fake_base_node.py").read_text(
        encoding="utf-8"
    )
    for message in (
        "HostMotionCommand",
        "WheelObservation",
        "ImuObservation",
        "ChassisLinkState",
        "FirmwareControlState",
        "FirmwareInfo",
        "DiagnosticArray",
    ):
        assert message in real and message in fake
    for provider in (real, fake):
        assert "Odometry" not in provider
        assert "TransformBroadcaster" not in provider
    assert '"~/wire_get_info"' in fake
    assert '"~/wire_line_ctrl"' in fake


def test_state_estimation_owns_wheel_odom_and_formal_gate_owns_dynamic_tf():
    wheel = (
        ROOT / "src/robot_state_estimation/robot_state_estimation/wheel_odometry_node.py"
    ).read_text(encoding="utf-8")
    assert "create_publisher(" in wheel and "Odometry" in wheel
    assert "TransformBroadcaster" not in wheel
    formal = (
        ROOT / "src/robot_state_estimation/robot_state_estimation/formal_odometry_node.py"
    ).read_text(encoding="utf-8")
    assert "TransformBroadcaster" in formal
    assert "permit_formal_odometry" in formal
    for config_name in ("ekf_wheel.yaml", "ekf_base.yaml"):
        config = yaml.safe_load(
            (ROOT / "src/robot_state_estimation/config" / config_name).read_text(encoding="utf-8")
        )
        assert config["/**"]["ros__parameters"]["publish_tf"] is False


def test_platform_topics_are_relative_and_namespace_is_launchable():
    platform = yaml.safe_load(
        (ROOT / "src/robot_config/config/platform.yaml").read_text(encoding="utf-8")
    )
    assert all(not value.startswith("/") for value in platform["platform"]["topics"].values())
    system = (ROOT / "src/robot_bringup/launch/system.launch.py").read_text(encoding="utf-8")
    assert "PushRosNamespace" in system
    assert 'DeclareLaunchArgument("namespace", default_value="")' in system
    nav2_launch = (ROOT / "src/robot_bringup/launch/nav2.launch.py").read_text(encoding="utf-8")
    assert 'SetRemap(src="navigate_to_pose", dst="nav2/navigate_to_pose")' in nav2_launch
    assert 'dst="/nav2/navigate_to_pose"' not in nav2_launch


def test_no_certified_nav2_fallback_or_duplicated_slam_profile():
    config = ROOT / "src/robot_bringup/config"
    assert not (config / "nav2_params_fallback.yaml").exists()
    assert (config / "nav2_minimal_example.yaml").is_file()
    assert not list(config.glob("slam_toolbox_mapping_*.yaml"))
    for profile in (config / "slam_profiles").glob("*.yaml"):
        data = yaml.safe_load(profile.read_text(encoding="utf-8"))
        assert set(data) == {"profile", "overlay"}
    nav_profile = yaml.safe_load(
        (config / "nav2_profiles/navigation.yaml").read_text(encoding="utf-8")
    )
    assert set(nav_profile) == {"profile", "overlay"}


def test_real_base_launch_requires_compiled_effective_config():
    system = (ROOT / "src/robot_bringup/launch/system.launch.py").read_text(encoding="utf-8")
    bridge = (ROOT / "src/stm32_robot_bridge/launch/stm32_bridge.launch.py").read_text(
        encoding="utf-8"
    )
    assert "base providers require compiled effective params" in system
    assert "effective params are required; start through bin/robot" in bridge
