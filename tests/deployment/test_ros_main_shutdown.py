"""Host-only execution of unchanged entrypoint error/cleanup branches, without ROS mocks as evidence."""

import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest


ROOT = Path(__file__).resolve().parents[2]
ENTRYPOINTS = (
    ("robot_config", "compatibility_node", "PlatformCompatibilityNode"),
    ("robot_chassis_ops", "chassis_ops_node", "ChassisOpsNode"),
    ("robot_control", "cmd_vel_mux", "CmdVelMuxNode"),
    ("robot_navigation_guard", "navigation_guard_node", "NavigationGuardNode"),
    ("robot_verification", "fake_base_node", "FakeBaseNode"),
    ("robot_state_estimation", "wheel_odometry_node", "WheelOdometryNode"),
    ("robot_state_estimation", "imu_adapter_node", "ImuAdapterNode"),
    ("robot_state_estimation", "formal_odometry_node", "FormalOdometryNode"),
    ("robot_supervision", "motion_supervisor_node", "MotionSupervisorNode"),
)


class RCLError(RuntimeError):
    pass


class ExternalShutdownException(Exception):
    pass


def entrypoint(package, module, constructor, *, context_ok, failure):
    path = ROOT / "src" / package / package / f"{module}.py"
    source = ast.parse(path.read_text(encoding="utf-8"))
    main = next(
        item for item in source.body if isinstance(item, ast.FunctionDef) and item.name == "main"
    )
    canonical_import = next(
        item
        for item in source.body
        if isinstance(item, ast.ImportFrom) and item.module == "rclpy.impl.implementation_singleton"
    )
    assert [(item.name, item.asname) for item in canonical_import.names] == [
        ("rclpy_implementation", "_rclpy")
    ]
    events = []
    node = NS(
        context=NS(ok=lambda: context_ok),
        destroy_node=lambda: events.append("destroy"),
    )

    def spin(current):
        assert current is node
        events.append("spin")
        raise failure

    namespace = {
        "rclpy": NS(
            init=lambda *args, **kwargs: events.append("init"),
            spin=spin,
            # An unrelated default context must not decide this node's validity.
            ok=lambda: not context_ok,
            try_shutdown=lambda: events.append("shutdown"),
            executors=NS(ExternalShutdownException=ExternalShutdownException),
        ),
        "_rclpy": NS(RCLError=RCLError),
        "ExternalShutdownException": ExternalShutdownException,
        constructor: lambda: node,
    }
    exec(compile(ast.Module(body=[main], type_ignores=[]), str(path), "exec"), namespace)
    return namespace["main"], events


@pytest.mark.parametrize(
    "package,module,constructor", ENTRYPOINTS, ids=[entry[1] for entry in ENTRYPOINTS]
)
@pytest.mark.parametrize("context_ok", [False, True], ids=["shutdown", "live-context"])
def test_rcl_error_is_ignored_only_after_node_context_shutdown(
    package, module, constructor, context_ok
):
    error = RCLError("wait set context invalid")
    main, events = entrypoint(package, module, constructor, context_ok=context_ok, failure=error)
    if context_ok:
        with pytest.raises(RCLError) as raised:
            main()
        assert raised.value is error
    else:
        main()
    assert events == ["init", "spin", "destroy", "shutdown"]


@pytest.mark.parametrize(
    "package,module,constructor", ENTRYPOINTS, ids=[entry[1] for entry in ENTRYPOINTS]
)
def test_shutdown_does_not_swallow_unrelated_runtime_error(package, module, constructor):
    error = RuntimeError("business failure")
    main, events = entrypoint(package, module, constructor, context_ok=False, failure=error)
    with pytest.raises(RuntimeError) as raised:
        main()
    assert raised.value is error
    assert events == ["init", "spin", "destroy", "shutdown"]


@pytest.mark.parametrize("context_ok", [False, True], ids=["shutdown", "live-context"])
@pytest.mark.parametrize("package,module,constructor", [
    ("robot_supervision", "motion_supervisor_node", "MotionSupervisorNode"),
    ("robot_control", "cmd_vel_mux", "CmdVelMuxNode"),
])
def test_message_conversion_shutdown_race_requires_invalid_context(
    package, module, constructor, context_ok
):
    error = RuntimeError(
        "Unable to convert call argument to Python object (compile in debug mode for details)"
    )
    main, events = entrypoint(
        package, module, constructor,
        context_ok=context_ok, failure=error,
    )
    if context_ok:
        with pytest.raises(RuntimeError) as raised:
            main()
        assert raised.value is error
    else:
        main()
    assert events == ["init", "spin", "destroy", "shutdown"]
