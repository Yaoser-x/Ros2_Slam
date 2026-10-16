"""Host-only compiled-config/launch composition checks; no ROS executor is emulated."""

from copy import deepcopy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
from types import ModuleType

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]

from robot_config.compiler import ConfigError, compile_effective_config, load_effective_config
from robot_config.launch_config import load_launch_config
from robot_config.runtime_options import runtime_overrides

CONFIG = ROOT / "src/robot_config/config"


@pytest.fixture
def compiled_workdir():
    cache = ROOT / "build/compiled-runtime-launch-tests"
    cache.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="case-", dir=cache) as directory:
        path = Path(directory).resolve()
        assert path.is_relative_to(cache.resolve())
        yield path


def compile_run(directory, arguments=None, *, overrides=None, profile="navigation"):
    def merge(target, source):
        for key, value in source.items():
            if isinstance(value, dict) and isinstance(target.get(key), dict):
                merge(target[key], value)
            else:
                target[key] = deepcopy(value)

    base = load_effective_config(CONFIG, profile=profile)
    if arguments is not None:
        cli = runtime_overrides(arguments, base, environment={})
        if overrides:
            merge(cli, overrides)
        overrides = cli
    paths = compile_effective_config(
        CONFIG, directory / "compiled", profile=profile, overrides=overrides
    )
    return paths, load_launch_config(paths["ros_params"])[0]


def yaml_file(directory, name):
    return yaml.safe_load((directory / name).read_text(encoding="utf-8"))


def test_suspended_cli_compiles_strict_schema_and_binds_measured_geometry(compiled_workdir):
    paths, params = compile_run(
        compiled_workdir, ["base-test", "--suspended-wheel"], profile="diagnostics"
    )
    effective = yaml_file(paths["manifest"].parent, "effective-config.yaml")
    assert effective["runtime"]["test_environment"] == "suspended_wheels"
    assert effective["runtime"]["fusion_mode"] == "wheel"
    assert effective["calibration"]["drive"]["wheel_track_width_m"] == 0.178
    assert effective["calibration_bundle"]["identity"]["firmware_parameter_crc32"] == 0x0D663E2A


def custom_platform():
    return {
        "platform": {
            "frames": {
                "map": "survey_map",
                "odom": "survey_odom",
                "base": "survey_base",
                "imu": "survey_imu",
                "laser": "survey_laser",
            },
            "topics": {
                "wheel_odom": "observed/wheel_odom",
                "imu": "observed/imu",
                "odom": "observed/odom",
                "firmware_control_state": "observed/control",
            },
        },
        "hardware": {"footprint_m": [[0.23, 0.12], [0.23, -0.12], [-0.19, -0.12], [-0.19, 0.12]]},
        "calibration": {"lidar": {"frame": "survey_laser"}},
        "calibration_bundle": {
            "lidar": {"frame": "survey_laser"},
            "imu": {"frame": "survey_imu"},
        },
    }


def test_cli_overrides_reach_artifacts_identity_and_provenance(compiled_workdir):
    arguments = [
        "navigation",
        "--fake-base",
        "--ekf-base",
        "--lidar-yaw-deg",
        "15",
        "--lidar-port",
        "/dev/test_lidar",
        "--base-port",
        "/dev/test_base",
        "--no-lidar-inverted",
        "--lidar-reversion",
    ]
    paths, effective = compile_run(compiled_workdir, arguments)
    directory = paths["ros_params"].parent
    lidar = yaml_file(directory, "lidar-params.yaml")["ydlidar_ros2_driver_node"]["ros__parameters"]
    calibration = yaml_file(directory, "launch-calibration.yaml")
    manifest = yaml_file(directory, "manifest.yaml")
    assert effective["runtime"]["base_mode"] == "fake"
    assert effective["runtime"]["fusion_mode"] == "wheel_imu"
    assert effective["runtime"]["cli_args"] == arguments
    assert lidar["port"] == "/dev/test_lidar"
    assert lidar["inverted"] is False and lidar["reversion"] is True
    assert calibration["lidar"]["rotation"][2] == pytest.approx(math.radians(15))
    assert (
        effective["config_sha256"]
        != load_effective_config(CONFIG, profile="navigation")["config_sha256"]
    )
    for name, digest in manifest["artifact_sha256"].items():
        assert hashlib.sha256((directory / name).read_bytes()).hexdigest() == digest
    for name in (
        "runtime.base_mode",
        "runtime.fusion_mode",
        "runtime.lidar_inverted",
        "hardware.lidar_port",
        "calibration_bundle.lidar.rotation",
    ):
        assert manifest["provenance"][name] == "<runtime-overrides>"
    assert "launch-env.sh" in manifest["artifact_sha256"]
    environment = (directory / "launch-env.sh").read_text(encoding="utf-8")
    assert "ROBOT_BASE_FUSION_MODE=wheel_imu" in environment


def test_custom_frames_geometry_and_topics_reach_each_component(compiled_workdir):
    paths, effective = compile_run(
        compiled_workdir,
        ["navigation", "--fake-base", "--ekf-base"],
        overrides=custom_platform(),
    )
    directory = paths["ros_params"].parent
    params = yaml_file(directory, "ros-params.yaml")
    wheel = params["wheel_odometry"]["ros__parameters"]
    assert (wheel["frame_id"], wheel["child_frame_id"], wheel["odom_topic"]) == (
        "survey_odom",
        "survey_base",
        "observed/wheel_odom",
    )
    assert params["imu_adapter"]["ros__parameters"]["frame_id"] == "survey_imu"
    assert (
        params["chassis_ops"]["ros__parameters"]["firmware_control_state_topic"]
        == "observed/control"
    )
    assert (
        params["navigation_guard"]["ros__parameters"]["firmware_control_state_topic"]
        == "observed/control"
    )
    freshness = effective["safety"]["timing"]["status_timeout_sec"]
    assert params["navigation_guard"]["ros__parameters"]["state_timeout_sec"] == freshness
    assert params["formal_odometry"]["ros__parameters"]["compatibility_timeout_sec"] == freshness
    assert params["cmd_vel_mux"]["ros__parameters"]["nav_guard_timeout_sec"] == freshness
    for artifact in ("ekf-wheel.yaml", "ekf-wheel-imu.yaml"):
        ekf = yaml_file(directory, artifact)["/**"]["ros__parameters"]
        assert ekf["publish_tf"] is False
        assert (ekf["map_frame"], ekf["odom_frame"], ekf["base_link_frame"]) == (
            "survey_map",
            "survey_odom",
            "survey_base",
        )
        assert ekf["odom0"] == "observed/wheel_odom"
        if "imu0" in ekf:
            assert ekf["imu0"] == "observed/imu"
    nav2 = yaml_file(directory, "nav2-params.yaml")
    assert nav2["amcl"]["ros__parameters"]["base_frame_id"] == "survey_base"
    assert nav2["amcl"]["ros__parameters"]["odom_frame_id"] == "survey_odom"
    assert nav2["bt_navigator"]["ros__parameters"]["global_frame"] == "survey_map"
    assert nav2["bt_navigator"]["ros__parameters"]["odom_topic"] == "observed/odom"
    behavior = nav2["behavior_server"]["ros__parameters"]
    assert (behavior["global_frame"], behavior["robot_base_frame"]) == (
        "survey_odom",
        "survey_base",
    )
    for name, frame in (("local_costmap", "survey_odom"), ("global_costmap", "survey_map")):
        costmap = nav2[name][name]["ros__parameters"]
        assert costmap["global_frame"] == frame
        assert costmap["robot_base_frame"] == "survey_base"
        assert json.loads(costmap["footprint"]) == effective["hardware"]["footprint_m"]
    for artifact in ("slam-params.yaml", "slam-precision-params.yaml", "slam-fast-params.yaml"):
        slam = yaml_file(directory, artifact)["slam_toolbox"]["ros__parameters"]
        assert (slam["map_frame"], slam["odom_frame"], slam["base_frame"]) == (
            "survey_map",
            "survey_odom",
            "survey_base",
        )


def test_artifact_tampering_is_rejected(compiled_workdir):
    paths, _ = compile_run(compiled_workdir)
    artifact = paths["ros_params"].parent / "ekf-wheel-imu.yaml"
    artifact.write_text(artifact.read_text(encoding="utf-8") + "\n# altered\n", encoding="utf-8")
    with pytest.raises(RuntimeError, match="artifact hash mismatch"):
        load_launch_config(paths["ros_params"])


@pytest.mark.parametrize(
    "name",
    [
        "slam-params.yaml",
        "slam-precision-params.yaml",
        "slam-fast-params.yaml",
        "nav2-params.yaml",
        "ekf-wheel.yaml",
        "launch-env.sh",
    ],
)
def test_consumed_artifact_missing_from_manifest_is_rejected(compiled_workdir, name):
    paths, _ = compile_run(compiled_workdir)
    manifest = yaml_file(paths["manifest"].parent, paths["manifest"].name)
    del manifest["artifact_sha256"][name]
    paths["manifest"].write_text(yaml.safe_dump(manifest), encoding="utf-8")
    with pytest.raises(RuntimeError, match="missing compiled artifact"):
        load_launch_config(paths["ros_params"])


def test_legacy_command_timing_enters_effective_config(compiled_workdir):
    base = load_effective_config(CONFIG, profile="diagnostics")
    overrides = runtime_overrides(
        ["base", "--fake-base"],
        base,
        environment={"BASE_CMD_TIMEOUT": "0.16", "BASE_DRIVE_KEEPALIVE_SEC": "0.06"},
    )
    paths, effective = compile_run(compiled_workdir, overrides=overrides, profile="diagnostics")
    bridge = yaml_file(paths["ros_params"].parent, "ros-params.yaml")["stm32_bridge"][
        "ros__parameters"
    ]
    assert (
        bridge["cmd_timeout"] == effective["safety"]["timing"]["bridge_command_timeout_sec"] == 0.16
    )
    assert bridge["drive_keepalive_sec"] == 0.06


@pytest.mark.parametrize("mode", ["SYNCHRONOUS", "ASYNCHRONOUS"])
def test_bridge_publication_environment_consumes_hashed_config(
    compiled_workdir, launch_composer, mode
):
    paths, effective = compile_run(
        compiled_workdir, overrides={"hardware": {"bridge_publication_mode": mode}}
    )
    nodes, _ = launch_composer(
        ROOT / "src/stm32_robot_bridge/launch/stm32_bridge.launch.py",
        {
            "params_file": str(paths["ros_params"]),
            "port": "/dev/serial0",
            "namespace": "",
            "allow_uncompiled_config": "false",
        },
    )
    assert effective["hardware"]["bridge_publication_mode"] == mode
    bridge = yaml_file(paths["ros_params"].parent, "ros-params.yaml")["stm32_bridge"]
    assert bridge["ros__parameters"]["dds_publication_mode"] == mode
    assert nodes[0]["additional_env"] == {
        "RMW_FASTRTPS_PUBLICATION_MODE": mode,
        "RMW_FASTRTPS_USE_QOS_FROM_XML": "0",
    }


def test_bridge_publication_mode_changes_config_identity(compiled_workdir):
    hashes = []
    for mode in ("SYNCHRONOUS", "ASYNCHRONOUS"):
        _, effective = compile_run(
            compiled_workdir / mode, overrides={"hardware": {"bridge_publication_mode": mode}}
        )
        hashes.append(effective["config_sha256"])
    assert hashes[0] != hashes[1]
    with pytest.raises(ConfigError, match="schema"):
        compile_run(
            compiled_workdir / "invalid",
            overrides={"hardware": {"bridge_publication_mode": "UNSPECIFIED"}},
        )


def test_rehashed_file_cannot_hide_changed_effective_logical_identity(compiled_workdir):
    paths, _ = compile_run(compiled_workdir)
    effective = yaml_file(paths["effective"].parent, paths["effective"].name)
    effective["motion"]["soft_max_linear_mps"] = 0.20
    paths["effective"].write_text(yaml.safe_dump(effective), encoding="utf-8")
    manifest = yaml_file(paths["manifest"].parent, paths["manifest"].name)
    manifest["artifact_sha256"]["effective-config.yaml"] = hashlib.sha256(
        paths["effective"].read_bytes()
    ).hexdigest()
    paths["manifest"].write_text(yaml.safe_dump(manifest), encoding="utf-8")
    with pytest.raises(RuntimeError, match="effective config hash does not describe"):
        load_launch_config(paths["ros_params"])


@pytest.mark.parametrize(
    "alias,base_mode,port_key",
    [
        ("nav", "real", "base_port"),
        ("bridge", "real", "base_port"),
        ("test-base", "real", "base_port"),
        ("sensors", "none", "lidar_port"),
        ("system", "real", "base_port"),
    ],
)
def test_cli_aliases_are_classified_before_port_and_default_provider(alias, base_mode, port_key):
    effective = load_effective_config(CONFIG)
    overrides = runtime_overrides([alias, "--port", "/dev/test_alias"], effective, environment={})
    assert overrides["runtime"]["base_mode"] == base_mode
    assert overrides["hardware"] == {port_key: "/dev/test_alias"}


@pytest.mark.parametrize(
    "arguments,environment,error",
    [
        (["navigation", "--namespace", "robot1"], {}, "nonempty namespace"),
        (["navigation", "--lidar-yaw-deg", "nan"], {}, "finite"),
        (["navigation", "--lidar-yaw-rad", "inf"], {}, "finite"),
        (["navigation", "--lidar-port"], {}, "requires a value"),
        (["navigation"], {"BASE_WHEEL_TRACK_WIDTH": "0.3"}, "deprecated"),
        (["navigation"], {"BASE_ODOM_ANGULAR_SCALE": "2"}, "deprecated"),
        (["navigation"], {"BASE_ODOM_ANGULAR_SIGN": "-1"}, "deprecated"),
        (["navigation"], {"BASE_CMD_TIMEOUT": "nan"}, "finite"),
        (["navigation"], {"BASE_CMD_LOG_INTERVAL_SEC": "5"}, "unsupported"),
    ],
)
def test_invalid_or_deprecated_runtime_inputs_fail_before_compilation(
    arguments, environment, error
):
    with pytest.raises(ConfigError, match=error):
        runtime_overrides(arguments, load_effective_config(CONFIG), environment=environment)


class _Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class _Declaration(_Action):
    pass


class _Opaque(_Action):
    pass


class _Include(_Action):
    pass


class _Node(_Action):
    pass


class _Group(_Action):
    pass


class _Configuration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return str(context[self.name])


class _Description:
    def __init__(self, entities):
        self.entities = entities


class _Source:
    def __init__(self, path):
        self.path = Path(path)


def _resolve(value, context):
    if isinstance(value, _Configuration):
        return value.perform(context)
    if isinstance(value, dict):
        return {key: _resolve(child, context) for key, child in value.items()}
    if isinstance(value, (list, tuple)):
        return type(value)(_resolve(child, context) for child in value)
    return value


@pytest.fixture
def launch_composer(monkeypatch):
    def module(name, **exports):
        result = ModuleType(name)
        result.__dict__.update(exports)
        monkeypatch.setitem(sys.modules, name, result)

    module("ament_index_python")
    module(
        "ament_index_python.packages",
        get_package_share_directory=lambda name: str(ROOT / "src" / name),
    )
    module("launch", LaunchDescription=_Description)
    module(
        "launch.actions",
        DeclareLaunchArgument=_Declaration,
        GroupAction=_Group,
        IncludeLaunchDescription=_Include,
        LogInfo=_Action,
        OpaqueFunction=_Opaque,
    )
    module("launch.conditions", IfCondition=_Action)
    module("launch.launch_description_sources", PythonLaunchDescriptionSource=_Source)
    module("launch.substitutions", LaunchConfiguration=_Configuration)
    module("launch_ros")
    module("launch_ros.actions", Node=_Node, PushRosNamespace=_Action)

    def load(path):
        spec = importlib.util.spec_from_file_location(
            "isolated_launch_" + path.stem.replace(".", "_"), path
        )
        imported = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(imported)
        return imported

    def compose(path, context):
        nodes, includes = [], []

        def expand(actions, values):
            for action in actions:
                condition = action.kwargs.get("condition") if isinstance(action, _Action) else None
                if (
                    condition is not None
                    and str(_resolve(condition.args[0], values)).lower() != "true"
                ):
                    continue
                if isinstance(action, _Declaration):
                    values.setdefault(
                        action.args[0], _resolve(action.kwargs.get("default_value", ""), values)
                    )
                elif isinstance(action, _Opaque):
                    expand(action.kwargs["function"](values), values)
                elif isinstance(action, _Node):
                    nodes.append(_resolve(action.kwargs, values))
                elif isinstance(action, _Include):
                    child = dict(values)
                    child.update(_resolve(dict(action.kwargs.get("launch_arguments", [])), values))
                    source = action.args[0].path
                    includes.append((source, deepcopy(child)))
                    expand(load(source).generate_launch_description().entities, child)
                elif isinstance(action, _Group):
                    expand(action.kwargs["actions"], dict(values))

        expand(load(path).generate_launch_description().entities, dict(context))
        return nodes, includes

    return compose


def test_base_only_composes_complete_platform_and_consumes_generated_estimation(
    compiled_workdir, launch_composer
):
    paths, _ = compile_run(
        compiled_workdir,
        ["base", "--fake-base", "--ekf-base"],
        overrides=custom_platform(),
        profile="diagnostics",
    )
    directory = paths["ros_params"].parent
    nodes, includes = launch_composer(
        ROOT / "src/robot_bringup/launch/base_only.launch.py",
        {
            "effective_params_file": str(paths["ros_params"]),
            "base_mode": "fake",
            "base_port": "",
            "base_fusion_mode": "",
            "lidar_params_file": str(directory / "lidar-params.yaml"),
        },
    )
    by_name = {node["name"]: node for node in nodes}
    required = {
        "platform_compatibility",
        "chassis_ops",
        "cmd_vel_mux",
        "fake_base",
        "wheel_odometry",
        "motion_supervisor",
    }
    assert required <= by_name.keys()
    assert {"imu_adapter", "formal_odometry", "base_ekf"} <= by_name.keys()
    assert not any(
        node["package"] in {"slam_toolbox", "nav2_bringup", "rviz2", "ydlidar_ros2_driver"}
        for node in nodes
    )
    assert not any(
        path.name in {"slam.launch.py", "nav2.launch.py", "localization.launch.py"}
        for path, _ in includes
    )
    assert by_name["chassis_ops"]["parameters"] == [
        str(paths["ros_params"]),
        {"wire_adapter_name": "fake_base"},
    ]
    assert by_name["base_ekf"]["parameters"] == [str(directory / "ekf-wheel-imu.yaml")]
    formal = by_name["formal_odometry"]["parameters"][-1]
    assert formal == {
        "input_topic": "odometry/filtered_internal",
        "output_topic": "observed/odom",
    }
    description = next(values for path, values in includes if path.name == "description.launch.py")
    assert description["base_frame"] == "survey_base" and description["imu_frame"] == "survey_imu"
    assert by_name["static_tf_imu_link"]["arguments"][-2:] == ["survey_base", "survey_imu"]


def test_mapping_custom_laser_extrinsics_and_frames_reach_description_nodes(
    compiled_workdir, launch_composer
):
    paths, _ = compile_run(
        compiled_workdir,
        ["mapping", "--fake-base", "--lidar-yaw-deg", "15"],
        overrides=custom_platform(),
        profile="mapping",
    )
    directory = paths["ros_params"].parent
    nodes, _ = launch_composer(
        ROOT / "src/robot_bringup/launch/system.launch.py",
        {
            "effective_params_file": str(paths["ros_params"]),
            "base_mode": "fake",
            "base_port": "",
            "base_fusion_mode": "",
            "lidar_params_file": str(directory / "lidar-params.yaml"),
        },
    )
    by_name = {node["name"]: node for node in nodes}
    laser_arguments = by_name["static_tf_pub_laser"]["arguments"]
    assert laser_arguments[-2:] == ["survey_base", "survey_laser"]
    assert float(laser_arguments[3]) == pytest.approx(math.radians(15))
    assert by_name["formal_odometry"]["parameters"][-1]["input_topic"] == "observed/wheel_odom"
    assert "base_ekf" not in by_name
    assert by_name["slam_toolbox"]["parameters"][0] == str(directory / "slam-params.yaml")


def test_mapping_without_base_uses_compiled_frames_for_static_odometry_fallback(
    compiled_workdir, launch_composer
):
    paths, _ = compile_run(
        compiled_workdir, ["mapping"], overrides=custom_platform(), profile="mapping"
    )
    directory = paths["ros_params"].parent
    nodes, _ = launch_composer(
        ROOT / "src/robot_bringup/launch/system.launch.py",
        {
            "effective_params_file": str(paths["ros_params"]),
            "base_mode": "none",
            "use_base": "false",
            "base_port": "",
            "base_fusion_mode": "",
            "lidar_params_file": str(directory / "lidar-params.yaml"),
            "slam_params_file": str(directory / "slam-params.yaml"),
        },
    )
    by_name = {node["name"]: node for node in nodes}
    assert by_name["odom_fallback_static_tf"]["arguments"][-2:] == [
        "survey_odom",
        "survey_base",
    ]


@pytest.mark.parametrize(
    "override,error",
    [
        ({"namespace": "robot1"}, "nonempty namespace"),
        ({"base_port": "/dev/not_the_compiled_port"}, "base_port differs"),
        ({"base_fusion_mode": "wheel_imu"}, "base_fusion_mode differs"),
        ({"lidar_tf_x": "0.2"}, "lidar_tf_x differs"),
        ({"lidar_tf_yaw": "nan"}, "lidar_tf_yaw differs"),
        (
            {"slam_params_file": "uncompiled-slam.yaml"},
            "slam_params_file must be the compiled artifact",
        ),
        (
            {"nav2_params_file": "uncompiled-nav2.yaml"},
            "nav2_params_file must be the compiled artifact",
        ),
        (
            {"lidar_params_file": "uncompiled-lidar.yaml"},
            "lidar params must be the compiled artifact",
        ),
    ],
)
def test_system_composition_rejects_uncompiled_launch_overrides(
    compiled_workdir, launch_composer, override, error
):
    paths, _ = compile_run(compiled_workdir, ["mapping", "--fake-base"], profile="mapping")
    directory = paths["ros_params"].parent
    context = {
        "effective_params_file": str(paths["ros_params"]),
        "base_mode": "fake",
        "base_port": "",
        "base_fusion_mode": "",
        "lidar_params_file": str(directory / "lidar-params.yaml"),
    }
    context.update(override)
    with pytest.raises(RuntimeError, match=error):
        launch_composer(ROOT / "src/robot_bringup/launch/system.launch.py", context)
