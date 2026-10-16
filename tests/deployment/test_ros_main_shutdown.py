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
BOUNDED_EXECUTORS = {
    "compatibility_node", "chassis_ops_node", "cmd_vel_mux", "motion_supervisor_node",
    "wheel_odometry_node", "imu_adapter_node", "formal_odometry_node",
}


class RCLError(RuntimeError):
    pass


class ExternalShutdownException(Exception):
    pass


def entrypoint(package, module, constructor, *, context_ok, failure, signal_to_send=2):
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
    init_options, signal_handlers = [], []
    node = NS(
        # The simulated context can become invalid while taking a message.
        context=NS(ok=lambda: context_ok if "spin" in events else True),
        destroy_node=lambda: events.append("destroy"),
    )

    def spin(current):
        assert current is node
        events.append("spin")
        if failure is None:
            handler = dict(signal_handlers)[signal_to_send]
            handler(signal_to_send, None)
            return
        raise failure

    class Executor:
        def __init__(self, *, context):
            assert context is node.context

        def add_node(self, current):
            assert current is node

        def spin_once(self, *, timeout_sec):
            assert timeout_sec == 0.05
            spin(node)

        def shutdown(self):
            events.append("executor_shutdown")

    namespace = {
        "rclpy": NS(
            init=lambda *args, **kwargs: (events.append("init"), init_options.append(kwargs)),
            spin=spin,
            # An unrelated default context must not decide this node's validity.
            ok=lambda: not context_ok,
            try_shutdown=lambda: events.append("shutdown"),
            executors=NS(ExternalShutdownException=ExternalShutdownException),
        ),
        "_rclpy": NS(RCLError=RCLError),
        "SingleThreadedExecutor": Executor,
        "ExternalShutdownException": ExternalShutdownException,
        "SignalHandlerOptions": NS(NO="no-native-signal-handler"),
        "signal": NS(SIGINT=2, SIGTERM=15, default_int_handler="python-interrupt",
                     signal=lambda *args: signal_handlers.append(args)),
        constructor: lambda: node,
    }
    exec(compile(ast.Module(body=[main], type_ignores=[]), str(path), "exec"), namespace)
    namespace["main"].init_options = init_options
    namespace["main"].signal_handlers = signal_handlers
    return namespace["main"], events


def expected_cleanup(module):
    return ["init", "spin"] + (
        ["executor_shutdown"] if module in BOUNDED_EXECUTORS else []
    ) + ["destroy", "shutdown"]


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
    assert events == expected_cleanup(module)


@pytest.mark.parametrize(
    "package,module,constructor", ENTRYPOINTS, ids=[entry[1] for entry in ENTRYPOINTS]
)
def test_shutdown_does_not_swallow_unrelated_runtime_error(package, module, constructor):
    error = RuntimeError("business failure")
    main, events = entrypoint(package, module, constructor, context_ok=False, failure=error)
    with pytest.raises(RuntimeError) as raised:
        main()
    assert raised.value is error
    assert events == expected_cleanup(module)


@pytest.mark.parametrize("package,module,constructor", [
    ("robot_config", "compatibility_node", "PlatformCompatibilityNode"),
    ("robot_chassis_ops", "chassis_ops_node", "ChassisOpsNode"),
    ("robot_supervision", "motion_supervisor_node", "MotionSupervisorNode"),
    ("robot_control", "cmd_vel_mux", "CmdVelMuxNode"),
    ("robot_state_estimation", "wheel_odometry_node", "WheelOdometryNode"),
    ("robot_state_estimation", "imu_adapter_node", "ImuAdapterNode"),
    ("robot_state_estimation", "formal_odometry_node", "FormalOdometryNode"),
])
@pytest.mark.parametrize("signal_to_send", [2, 15])
def test_python_signal_flag_cleans_up_without_raising_during_message_conversion(
    package, module, constructor, signal_to_send
):
    main, events = entrypoint(
        package, module, constructor,
        context_ok=True, failure=None, signal_to_send=signal_to_send,
    )
    main()
    assert main.init_options[0]["signal_handler_options"] == "no-native-signal-handler"
    assert [kind for kind, handler in main.signal_handlers] == [2, 15]
    assert main.signal_handlers[0][1] is main.signal_handlers[1][1]
    assert callable(main.signal_handlers[0][1])
    assert events == expected_cleanup(module)
