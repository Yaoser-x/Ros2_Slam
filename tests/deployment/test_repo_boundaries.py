from pathlib import Path
import importlib.util
import subprocess


ROOT = Path(__file__).resolve().parents[2]


def git_ls_files() -> list[str]:
    result = subprocess.run(
        ["git", "ls-files"],
        cwd=ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    return result.stdout.splitlines()


def test_runtime_and_vendor_sources_are_not_tracked() -> None:
    forbidden = (
        "logs/",
        "log/",
        "colcon.meta",
        "src/YDLidar-SDK/",
        "src/ydlidar_ros2_driver/",
        "src/third_party/robot_localization/",
    )

    tracked = git_ls_files()

    assert not [path for path in tracked if path == "colcon.meta" or path.startswith(forbidden)]


def test_platform_packages_exist_and_bringup_defaults_to_headless() -> None:
    for package_name in (
        "robot_interfaces",
        "robot_config",
        "robot_description",
        "robot_sensing",
        "robot_state_estimation",
        "robot_control",
        "robot_bringup",
        "stm32_robot_bridge",
    ):
        assert (ROOT / "src" / package_name / "package.xml").is_file()

    system_launch = (ROOT / "src" / "robot_bringup" / "launch" / "system.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'DeclareLaunchArgument("use_rviz", default_value="false")' in system_launch
    assert 'get_package_share_directory("robot_sensing")' in system_launch
    assert 'get_package_share_directory("robot_description")' in system_launch
    assert 'get_package_share_directory("robot_control")' in system_launch
    assert 'get_package_share_directory("robot_state_estimation")' in system_launch


def test_primary_cli_invokes_read_only_legacy_dispatcher_via_bash() -> None:
    cli = (ROOT / "bin" / "robot").read_text(encoding="utf-8")

    assert 'exec bash "${ROOT}/launch_scripts/robot.sh" "$@"' in cli


def test_stm32_bridge_uses_explicit_command_and_raw_observation_defaults() -> None:
    bridge_node = (
        ROOT / "src" / "stm32_robot_bridge" / "stm32_robot_bridge" / "bridge_node_v3.py"
    ).read_text(encoding="utf-8")
    launch_file = (
        ROOT / "src" / "stm32_robot_bridge" / "launch" / "stm32_bridge.launch.py"
    ).read_text(encoding="utf-8")

    assert '"host_motion_command_topic": "chassis/host_motion_command"' in bridge_node
    assert '"wheel_observation_topic": "wheel/observation"' in bridge_node
    assert '"imu_observation_topic": "imu/observation"' in bridge_node
    assert "EncoderOdometry" not in bridge_node
    assert "create_publisher(Odometry" not in bridge_node
    assert "enable_legacy_cmd_vel" not in bridge_node
    assert "ROBOT_EFFECTIVE_PARAMS" in launch_file
    assert 'DeclareLaunchArgument("namespace", default_value="")' in launch_file


def test_external_ydlidar_sdk_is_hidden_from_colcon() -> None:
    fetch_vendor = (ROOT / "scripts" / "bootstrap" / "fetch_vendor.sh").read_text(encoding="utf-8")

    assert "touch vendor/ydlidar-sdk/COLCON_IGNORE" in fetch_vendor

    sdk_package = ROOT / "vendor" / "ydlidar-sdk" / "package.xml"
    if sdk_package.exists():
        assert (ROOT / "vendor" / "ydlidar-sdk" / "COLCON_IGNORE").is_file()


def test_owned_executable_scripts_use_lf_line_endings() -> None:
    script_paths = sorted((ROOT / "launch_scripts").rglob("*.sh"))
    script_paths.extend(sorted((ROOT / "scripts").rglob("*.sh")))
    script_paths.extend(sorted((ROOT / "launch_scripts").rglob("*.py")))
    script_paths.extend(sorted((ROOT / "src").glob("*/launch/*.py")))
    script_paths.append(
        ROOT / "src" / "stm32_robot_bridge" / "stm32_robot_bridge" / "bridge_node_v3.py"
    )

    offenders = [
        path.relative_to(ROOT).as_posix()
        for path in script_paths
        if path.is_file() and b"\r\n" in path.read_bytes()
    ]

    assert offenders == []


def test_build_script_sources_ros_with_nounset_disabled() -> None:
    build_script = (ROOT / "scripts" / "build" / "build_ros_ws.sh").read_text(encoding="utf-8")

    assert "set -euo pipefail" in build_script
    assert "set +u" in build_script
    assert "source /opt/ros/humble/setup.bash" in build_script
    assert "set -u" in build_script
    assert build_script.index("set +u") < build_script.index("source /opt/ros/humble/setup.bash")
    assert build_script.index("source /opt/ros/humble/setup.bash") < build_script.rindex("set -u")


def test_colcon_entrypoints_only_discover_packages_under_src() -> None:
    entrypoints = (
        ROOT / "scripts" / "build" / "build_ros_ws.sh",
        ROOT / "scripts" / "verify" / "verify_upper.sh",
    )

    for path in entrypoints:
        content = path.read_text(encoding="utf-8")
        assert "colcon build --base-paths src" in content, path
        assert "colcon test --base-paths src" in content, path

    workflow = (ROOT / ".github/workflows/ci.yml").read_text(encoding="utf-8")
    assert "python3 scripts/verify/run_software_evidence.py" in workflow
    spec = importlib.util.spec_from_file_location(
        "software_evidence_colcon_boundary", ROOT / "scripts/verify/run_software_evidence.py"
    )
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    commands, _ = runner.commands("ros_ci", ROOT / "build/boundary-report")
    for name in ("colcon_build", "colcon_test"):
        argv = commands[name]
        assert "colcon" in argv, name
        assert argv[argv.index("colcon") + 1] == name.removeprefix("colcon_"), name
        assert argv.count("--base-paths") == 1, name
        discovered_paths = []
        for argument in argv[argv.index("--base-paths") + 1 :]:
            if argument.startswith("-"):
                break
            discovered_paths.append(argument)
        assert discovered_paths == ["src"], name


def test_gitattributes_keeps_shell_scripts_lf() -> None:
    attributes = (ROOT / ".gitattributes").read_text(encoding="utf-8")

    assert "*.sh text eol=lf" in attributes


def test_save_map_uses_humble_compatible_map_saver_arguments() -> None:
    save_map = (ROOT / "launch_scripts" / "save_map.sh").read_text(encoding="utf-8")

    assert "ros2 topic echo /map --once" in save_map
    assert "| head" not in save_map
    assert "save_map_timeout:=15.0" in save_map
    assert "save_map_timeout:=15000" not in save_map


def test_build_script_is_not_hidden_by_build_artifact_ignore_rule() -> None:
    result = subprocess.run(
        ["git", "check-ignore", "scripts/build/build_ros_ws.sh"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 1


def test_vendor_source_cache_is_git_ignored() -> None:
    result = subprocess.run(
        ["git", "check-ignore", "vendor/ydlidar-sdk"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 0
