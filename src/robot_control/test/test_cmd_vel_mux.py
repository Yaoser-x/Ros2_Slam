"""cmd_vel_mux 节点单元测试 —— 验证多源仲裁、优先级、超时逻辑。

使用 mock rclpy 环境，不依赖 ROS2 运行时。
"""

import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

# ---------------------------------------------------------------------------
# Mock rclpy + 依赖
# ---------------------------------------------------------------------------

_mock_rclpy = MagicMock(name="rclpy")
_mock_rclpy.spin = MagicMock()
_mock_rclpy.init = MagicMock()
_mock_rclpy.shutdown = MagicMock()
_mock_rclpy.ok = MagicMock(return_value=True)

_mock_geometry_msgs = MagicMock(name="geometry_msgs")
_mock_sensor_msgs = MagicMock(name="sensor_msgs")

sys.modules["rclpy"] = _mock_rclpy
_mock_executors = ModuleType("rclpy.executors")
_mock_executors.ExternalShutdownException = type("ExternalShutdownException", (Exception,), {})
sys.modules["rclpy.executors"] = _mock_executors
_mock_implementation = ModuleType("rclpy.impl.implementation_singleton")
_mock_implementation.rclpy_implementation = SimpleNamespace(
    RCLError=type("RCLError", (RuntimeError,), {})
)
sys.modules["rclpy.impl"] = ModuleType("rclpy.impl")
sys.modules["rclpy.impl.implementation_singleton"] = _mock_implementation
sys.modules["geometry_msgs"] = _mock_geometry_msgs
sys.modules["geometry_msgs.msg"] = MagicMock()
sys.modules["sensor_msgs"] = _mock_sensor_msgs
sys.modules["sensor_msgs.msg"] = MagicMock()


class _FakeHostMotionCommand:
    HOST_SUBSOURCE_NONE = 0
    HOST_SUBSOURCE_TELEOP = 1
    HOST_SUBSOURCE_TEST = 2
    HOST_SUBSOURCE_NAV = 3
    HOST_SUBSOURCE_RESEARCH = 4

    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.twist = SimpleNamespace(
            linear=SimpleNamespace(x=0.0),
            angular=SimpleNamespace(z=0.0),
        )
        self.enable = False
        self.host_subsource = self.HOST_SUBSOURCE_NONE
        self.command_epoch = 0
        self.sequence = 0


class _FakeHostControlState:
    STATE_HOST_CLEARED = 0
    STATE_WAIT_SOURCE_QUIET = 1
    STATE_WAIT_WIRE_READY = 2
    STATE_WAIT_FRESH_HOST_INTENT = 3
    STATE_HOST_ACTIVE = 4
    REJECT_NONE = 0
    REJECT_INVALID = 1
    REJECT_STALE = 2
    REJECT_REARM_REQUIRED = 3
    REJECT_SUPERVISION_STALE = 4
    REJECT_MOTION_CRITICAL = 5
    REARM_TRANSPORT = 1
    REARM_MOTION_CRITICAL = 128
    REARM_SUPERVISION_STALE = 256

    def __init__(self):
        self.header = SimpleNamespace(stamp=None)


class _FakeMotionSupervisionState:
    LEVEL_CRITICAL = 4


class _FakeChassisLinkState:
    STATE_DISCONNECTED = 0
    STATE_WIRE_REARM_READY = 6
    STATE_DRIVE_ACTIVE = 7


class _FakeNavigationGuardState:
    STATE_ACTIVE = 4


_mock_robot_interfaces_msg = MagicMock()
_mock_robot_interfaces_msg.HostMotionCommand = _FakeHostMotionCommand
_mock_robot_interfaces_msg.ChassisLinkState = _FakeChassisLinkState
_mock_robot_interfaces_msg.HostControlState = _FakeHostControlState
_mock_robot_interfaces_msg.MotionSupervisionState = _FakeMotionSupervisionState
_mock_robot_interfaces_msg.NavigationGuardState = _FakeNavigationGuardState
sys.modules["robot_interfaces"] = MagicMock()
sys.modules["robot_interfaces.msg"] = _mock_robot_interfaces_msg

# ---------------------------------------------------------------------------
# Mock Node 基类
# ---------------------------------------------------------------------------


class MockNode:
    """替代 rclpy.node.Node 的 mock 基类。"""

    _param_overrides: dict = {}

    def __init__(self, node_name: str = "", **kwargs):
        self._node_name = node_name
        self._declared_params = {}
        self._clock = MagicMock()
        self._clock.now.return_value.nanoseconds = 0

    def declare_parameter(self, name, value=None):
        self._declared_params[name] = value

    def get_parameter(self, name):
        mock = MagicMock()
        if name in MockNode._param_overrides:
            mock.value = MockNode._param_overrides[name]
        elif name in self._declared_params:
            mock.value = self._declared_params[name]
        else:
            mock.value = name  # fallback
        return mock

    def get_logger(self):
        return MagicMock()

    def get_clock(self):
        return self._clock

    def create_publisher(self, msg_type, topic, qos_profile, **_kwargs):
        return MagicMock()

    def create_subscription(self, msg_type, topic, callback, qos_profile, **_kwargs):
        return MagicMock()

    def create_timer(self, timer_period_sec, callback, **_kwargs):
        self._timer_callback = callback
        return MagicMock()

    def destroy_node(self):
        pass

    @classmethod
    def set_params(cls, **overrides):
        cls._param_overrides = dict(overrides)


_mock_rclpy.node.Node = MockNode

# 设置 mock 子模块
sys.modules["rclpy.node"] = MagicMock()
sys.modules["rclpy.node"].Node = MockNode
sys.modules["rclpy.clock"] = MagicMock()
sys.modules["rclpy.clock"].Clock = MagicMock()
sys.modules["rclpy.clock"].ClockType = MagicMock()
sys.modules["rclpy.duration"] = MagicMock()
sys.modules["rclpy.duration"].Duration = MagicMock()
sys.modules["rclpy.qos"] = MagicMock()
sys.modules["rclpy.qos"].QoSProfile = MagicMock()
sys.modules["rclpy.qos_event"] = MagicMock()

# ---------------------------------------------------------------------------
# 导入被测试模块
# ---------------------------------------------------------------------------

# robot_control 包已在 sys.path（通过 conftest.py）
import robot_control.control_policy as _cp

sys.modules["robot_control.control_policy"] = _cp

from robot_control.cmd_vel_mux import CmdVelMuxNode


def make_mux(**param_overrides):
    """创建 CmdVelMuxNode 实例的工厂函数。"""
    param_overrides.setdefault("require_motion_supervision", False)
    MockNode.set_params(**param_overrides)
    with patch("os.path.exists", return_value=False):
        node = CmdVelMuxNode()
    node.publisher = MagicMock()
    node._logger = MagicMock()
    node.get_logger = MagicMock(return_value=node._logger)
    node.monotonic_clock = lambda: node.get_clock().now.return_value.nanoseconds * 1e-9
    node.rearm_required = False
    node.rearm_reason_flags = 0
    node.wire_ready = True
    node.gate_state = _FakeHostControlState.STATE_WAIT_FRESH_HOST_INTENT
    set_navigation_guard(node, generation=1)
    return node


def set_navigation_guard(node, generation=1, *, active=True):
    node._on_navigation_guard(
        SimpleNamespace(
            state=_FakeNavigationGuardState.STATE_ACTIVE if active else 0,
            goal_generation=generation,
        )
    )


# ---------------------------------------------------------------------------
# 测试
# ---------------------------------------------------------------------------


class TestCmdVelMuxNode:
    """验证 CmdVelMuxNode 的参数和结构。"""

    def test_default_parameters_are_set(self):
        node = make_mux()
        assert node.get_parameter("linear_limit").value == 0.4
        assert node.get_parameter("angular_limit").value == 1.5
        assert node.get_parameter("input_linear_abs_max").value == 5.0
        assert node.get_parameter("input_angular_abs_max").value == 20.0
        assert node.get_parameter("max_linear_accel").value == 0.5
        assert node.get_parameter("max_angular_accel").value == 1.5
        assert node.get_parameter("max_linear_jerk").value == 2.0
        assert node.get_parameter("max_angular_jerk").value == 6.0
        assert node.get_parameter("timeout_sec").value == 0.25
        assert node.get_parameter("publish_hz").value == 20.0
        assert (
            node.get_parameter("host_motion_command_topic").value == "chassis/host_motion_command"
        )

    def test_research_sources_default_to_empty(self):
        node = make_mux()
        assert node.source_config.research_sources == ()

    def test_research_sources_are_passed_to_config(self):
        node = make_mux(research_sources=["avoidance", "mpc"])
        assert node.source_config.research_sources == ("avoidance", "mpc")

    def test_subscriptions_created_for_all_sources(self):
        node = make_mux(research_sources=["avoidance"])
        sources = set(node.subscriptions_by_source.keys())
        assert "teleop" in sources
        assert "nav" in sources
        assert "test" in sources
        assert "research/avoidance" in sources

    def test_driver_topic_is_configurable(self):
        node = make_mux(host_motion_command_topic="/chassis/custom")
        assert node.get_parameter("host_motion_command_topic").value == "/chassis/custom"


class TestCmdVelMuxPublish:
    """验证 _publish_selected 发布逻辑。"""

    def test_publishes_selected_command(self):
        from robot_control.control_policy import Command

        node = make_mux()
        node.mux.update("teleop", Command(linear_x=0.3, angular_z=0.1), now_sec=1.0)
        node.get_clock().now.return_value.nanoseconds = int(1.0e9)

        node._publish_selected()

        node.publisher.publish.assert_called_once()
        msg = node.publisher.publish.call_args[0][0]
        assert msg.twist.linear.x == 0.0
        assert msg.twist.angular.z == 0.0
        assert msg.enable is True
        assert msg.host_subsource == _FakeHostMotionCommand.HOST_SUBSOURCE_TELEOP
        assert msg.sequence == 1

        node.get_clock().now.return_value.nanoseconds = int(1.05e9)
        node._publish_selected()
        ramped = node.publisher.publish.call_args[0][0]
        assert 0.0 < ramped.twist.linear.x < 0.3
        assert 0.0 < ramped.twist.angular.z < 0.1

    def test_idle_does_not_publish_continuously(self):
        node = make_mux()

        node._publish_selected()
        node._publish_selected()

        node.publisher.publish.assert_not_called()

    def test_active_to_idle_publishes_one_release_then_silent(self):
        from robot_control.control_policy import Command

        node = make_mux()
        node.mux.update("teleop", Command(0.3, 0.1), now_sec=0.0)
        node._publish_selected()
        node.publisher.reset_mock()
        node.get_clock().now.return_value.nanoseconds = int(0.30 * 1e9)

        node._publish_selected()
        node._publish_selected()

        node.publisher.publish.assert_called_once()
        msg = node.publisher.publish.call_args[0][0]
        assert msg.enable is False
        assert msg.host_subsource == _FakeHostMotionCommand.HOST_SUBSOURCE_NONE
        assert node.motion_limiter.current.linear_x == 0.0

    def test_new_source_after_idle_resumes(self):
        node = make_mux()
        callback = node._make_callback("nav")
        msg = MagicMock()
        msg.linear.x = 0.2
        msg.angular.z = 0.0

        callback(msg)

        published = node.publisher.publish.call_args[0][0]
        assert published.enable is True
        assert published.host_subsource == _FakeHostMotionCommand.HOST_SUBSOURCE_NAV

    def test_high_priority_timeout_falls_back_without_release(self):
        from robot_control.control_policy import Command

        node = make_mux()
        node.mux.update("teleop", Command(0.2, 0.0), now_sec=0.10)
        node.mux.update("nav", Command(0.1, 0.0), now_sec=0.20)
        node.get_clock().now.return_value.nanoseconds = int(0.20e9)
        set_navigation_guard(node)
        node._publish_selected()
        node.publisher.reset_mock()
        node.get_clock().now.return_value.nanoseconds = int(0.36e9)

        node._publish_selected()

        published = node.publisher.publish.call_args[0][0]
        assert published.enable is True
        assert published.host_subsource == _FakeHostMotionCommand.HOST_SUBSOURCE_NAV

    def test_legacy_twist_output_is_removed(self):
        node = make_mux()

        node._publish_selected()

        assert not hasattr(node, "legacy_publisher")

    def test_dead_navigation_guard_revokes_fresh_nav_candidate(self):
        from robot_control.control_policy import Command

        node = make_mux(timeout_sec=1.0)
        node.mux.update("nav", Command(0.2, 0.0), now_sec=0.0)
        node._publish_selected()
        node.publisher.reset_mock()
        node.get_clock().now.return_value.nanoseconds = int(0.30e9)
        message = MagicMock()
        message.linear.x, message.angular.z = 0.3, 0.0
        node._make_callback("nav")(message)
        node._publish_selected()

        released = node.publisher.publish.call_args[0][0]
        assert not released.enable
        assert not node.mux.select(0.30).active
        assert node.invalid_command_count == 0

    def test_dead_navigation_guard_preserves_manual_fallback(self):
        from robot_control.control_policy import Command

        node = make_mux(timeout_sec=1.0)
        node.mux.update("nav", Command(0.2, 0.0), now_sec=0.0)
        node._publish_selected()
        node.mux.update("teleop", Command(0.1, 0.0), now_sec=0.2)
        node.publisher.reset_mock()
        node.get_clock().now.return_value.nanoseconds = int(0.30e9)
        node._publish_selected()

        selected = node.publisher.publish.call_args[0][0]
        assert selected.enable
        assert selected.host_subsource == _FakeHostMotionCommand.HOST_SUBSOURCE_TELEOP

    def test_goal_terminal_state_releases_cached_navigation_immediately(self):
        from robot_control.control_policy import Command

        node = make_mux()
        node.mux.update("nav", Command(0.2, 0.0), now_sec=0.0)
        node._publish_selected()
        node.publisher.reset_mock()
        set_navigation_guard(node, active=False)

        assert not node.publisher.publish.call_args[0][0].enable
        assert not node.mux.select(0.0).active

    def test_transport_rearm_clears_old_goal_and_requires_quiet_then_fresh_input(self):
        from robot_control.control_policy import Command

        node = make_mux()
        node.mux.update("nav", Command(0.2, 0.0), now_sec=0.0)
        node._publish_selected()
        node.publisher.reset_mock()
        state = MagicMock()
        state.wire_session_id = 2
        state.wire_rearm_required = True
        state.wire_rearm_reason_flags = _FakeHostControlState.REARM_TRANSPORT
        state.link_state = _FakeChassisLinkState.STATE_DISCONNECTED
        state.protocol_compatible = False

        node._on_chassis_state(state)

        release = node.publisher.publish.call_args[0][0]
        assert release.enable is False
        assert node.rearm_required
        assert not node.mux.select(0.0).active

        node.publisher.reset_mock()
        node.get_clock().now.return_value.nanoseconds = int(0.30e9)
        node._publish_selected()
        node.get_clock().now.return_value.nanoseconds = int(0.60e9)
        node._publish_selected()
        assert node.gate_state == _FakeHostControlState.STATE_WAIT_WIRE_READY

        ready = MagicMock()
        ready.wire_session_id = 2
        ready.wire_rearm_required = False
        ready.wire_rearm_reason_flags = 0
        ready.link_state = _FakeChassisLinkState.STATE_WIRE_REARM_READY
        ready.protocol_compatible = True
        node._on_chassis_state(ready)
        node._publish_selected()
        assert node.gate_state == _FakeHostControlState.STATE_WAIT_FRESH_HOST_INTENT

        set_navigation_guard(node, generation=2)
        fresh = MagicMock()
        fresh.linear.x, fresh.angular.z = 0.1, 0.0
        node._make_callback("nav")(fresh)
        resumed = node.publisher.publish.call_args[0][0]
        assert resumed.enable is True
        assert not node.rearm_required

    def test_rearm_retries_lost_disable_until_wire_ready_without_automatic_enable(self):
        node = make_mux()
        node.rearm_required = True
        node.wire_ready = False
        node.gate_state = _FakeHostControlState.STATE_HOST_CLEARED
        node.release_sent_wire_session = None
        wire = SimpleNamespace(
            wire_session_id=7,
            wire_rearm_required=True,
            wire_rearm_reason_flags=_FakeHostControlState.REARM_TRANSPORT,
            link_state=5,
            protocol_compatible=True,
        )
        node._on_chassis_state(wire)
        first = node.publisher.publish.call_args[0][0]
        assert not first.enable and first.command_epoch
        node.get_clock().now.return_value.nanoseconds = int(0.02e9)
        node._publish_selected()
        assert node.publisher.publish.call_count == 1
        node.get_clock().now.return_value.nanoseconds = int(0.04e9)
        node._on_chassis_state(wire)
        node._publish_selected()
        assert node.publisher.publish.call_count == 1
        for elapsed in (0.06, 0.30, 0.60):
            node.get_clock().now.return_value.nanoseconds = int(elapsed * 1e9)
            node._on_chassis_state(wire)
            node._publish_selected()
        messages = [call.args[0] for call in node.publisher.publish.call_args_list]
        assert len(messages) == 4
        assert all(not message.enable for message in messages)
        assert all(message.command_epoch == first.command_epoch for message in messages)
        assert [message.sequence for message in messages] == [1, 2, 3, 4]
        assert node.gate_state == _FakeHostControlState.STATE_WAIT_WIRE_READY
        assert node.rearm_required
        wire.link_state = _FakeChassisLinkState.STATE_WIRE_REARM_READY
        wire.wire_rearm_required = False
        node._on_chassis_state(wire)
        node._publish_selected()
        assert node.gate_state == _FakeHostControlState.STATE_WAIT_FRESH_HOST_INTENT
        assert node.rearm_required and not node.last_active
        for elapsed in (0.80, 1.0):
            node.get_clock().now.return_value.nanoseconds = int(elapsed * 1e9)
            node._publish_selected()
        assert node.publisher.publish.call_count == 4

    def test_stale_supervision_still_retries_disable_without_advancing_host_gate(self):
        node = make_mux(require_motion_supervision=True)
        node.rearm_required = True
        node.wire_ready = False
        node.gate_state = _FakeHostControlState.STATE_HOST_CLEARED
        node._publish_selected()
        assert node.publisher.publish.call_count == 1
        node.get_clock().now.return_value.nanoseconds = int(0.06e9)
        node._publish_selected()
        assert node.publisher.publish.call_count == 2
        assert node.gate_state == _FakeHostControlState.STATE_HOST_CLEARED
        messages = [call.args[0] for call in node.publisher.publish.call_args_list]
        assert all(not message.enable for message in messages)
        assert messages[0].command_epoch == messages[1].command_epoch
        assert messages[0].sequence < messages[1].sequence

    def test_critical_motion_state_releases_and_latches_rearm(self):
        node = make_mux(require_motion_supervision=True)
        node.last_active = True
        message = MagicMock()
        message.command_scale = 0.0
        message.release_host_candidate = True
        message.level = _FakeMotionSupervisionState.LEVEL_CRITICAL

        node._on_motion_state(message)

        release = node.publisher.publish.call_args[0][0]
        assert release.enable is False
        assert node.rearm_required
        assert node.rearm_reason_flags & _FakeHostControlState.REARM_MOTION_CRITICAL


class TestCmdVelMuxCallback:
    """验证 _make_callback 产生的回调函数。"""

    def test_callback_updates_mux_with_clamped_command(self):
        from robot_control.control_policy import Command

        node = make_mux()
        callback = node._make_callback("teleop")

        mock_twist = MagicMock()
        mock_twist.linear.x = 0.5
        mock_twist.angular.z = -0.2

        callback(mock_twist)

        selected = node.mux.select(now_sec=0.1)  # 在 0.25s 超时窗口内
        assert selected.source == "teleop"
        assert selected.command == Command(linear_x=0.4, angular_z=-0.2)

    @pytest.mark.parametrize(
        "linear,angular",
        [
            (float("nan"), 0.0),
            (0.0, float("nan")),
            (float("inf"), 0.0),
            (0.0, float("-inf")),
        ],
    )
    def test_invalid_command_triggers_one_release_and_clears_source(self, linear, angular):
        from robot_control.control_policy import Command

        node = make_mux()
        node.mux.update("teleop", Command(0.2, 0.0), now_sec=0.0)
        node.last_active = True
        callback = node._make_callback("teleop")
        msg = MagicMock()
        msg.linear.x = linear
        msg.angular.z = angular

        callback(msg)

        published = node.publisher.publish.call_args[0][0]
        assert published.enable is False
        assert node.mux.select(0.0).active is False
        assert node.invalid_command_count == 1

    def test_invalid_high_priority_source_allows_lower_source_on_next_tick(self):
        from robot_control.control_policy import Command

        node = make_mux()
        node.mux.update("nav", Command(0.1, 0.0), now_sec=0.0)
        node.mux.update("teleop", Command(0.2, 0.0), now_sec=0.0)
        node.last_active = True
        callback = node._make_callback("teleop")
        msg = MagicMock()
        msg.linear.x = float("nan")
        msg.angular.z = 0.0

        callback(msg)
        node.publisher.reset_mock()
        node._publish_selected()

        published = node.publisher.publish.call_args[0][0]
        assert published.enable is True
        assert published.host_subsource == _FakeHostMotionCommand.HOST_SUBSOURCE_NAV

    def test_invalid_nav_does_not_interrupt_teleop(self):
        from robot_control.control_policy import Command

        node = make_mux()
        node.mux.update("teleop", Command(0.2, 0.0), now_sec=0.0)
        node.mux.update("nav", Command(0.1, 0.0), now_sec=0.0)
        node.last_active = True
        node.last_source = "teleop"
        msg = MagicMock()
        msg.linear.x = float("nan")
        msg.angular.z = 0.0

        node._make_callback("nav")(msg)

        node.publisher.publish.assert_not_called()
        assert node.mux.select(0.0).source == "teleop"

    def test_invalid_research_does_not_interrupt_nav(self):
        from robot_control.control_policy import Command

        node = make_mux(research_sources=["mpc"])
        node.mux.update("nav", Command(0.1, 0.0), now_sec=0.0)
        node.last_active = True
        node.last_source = "nav"
        msg = MagicMock()
        msg.linear.x = 0.0
        msg.angular.z = float("inf")

        node._make_callback("research/mpc")(msg)

        node.publisher.publish.assert_not_called()
        assert node.mux.select(0.0).source == "nav"

    def test_invalid_active_source_falls_back_immediately(self):
        from robot_control.control_policy import Command

        node = make_mux()
        node.mux.update("nav", Command(0.1, 0.0), now_sec=0.0)
        node.mux.update("teleop", Command(0.2, 0.0), now_sec=0.0)
        node.last_active = True
        node.last_source = "teleop"
        msg = MagicMock()
        msg.linear.x = float("nan")
        msg.angular.z = 0.0

        node._make_callback("teleop")(msg)

        published = node.publisher.publish.call_args[0][0]
        assert published.enable is True
        assert published.host_subsource == _FakeHostMotionCommand.HOST_SUBSOURCE_NAV

    def test_invalid_source_spam_does_not_toggle_enable(self):
        from robot_control.control_policy import Command

        node = make_mux()
        node.mux.update("teleop", Command(0.2, 0.0), now_sec=0.0)
        node.last_active = True
        node.last_source = "teleop"
        msg = MagicMock()
        msg.linear.x = float("nan")
        msg.angular.z = 0.0
        callback = node._make_callback("nav")

        for _ in range(10):
            callback(msg)

        node.publisher.publish.assert_not_called()
        assert node.invalid_command_count_by_source["nav"] == 10

    def test_qos_event_counters_are_exposed_in_host_control_state(self):
        node = make_mux()
        node._on_publish_deadline(SimpleNamespace(total_count_change=2))
        node._on_candidate_liveliness(SimpleNamespace(alive_count_change=-1))
        node._on_candidate_message_lost(SimpleNamespace(total_count_change=3))

        node._publish_control_state(0.0)

        state = node.state_publisher.publish.call_args[0][0]
        assert state.dds_deadline_miss_count == 2
        assert state.publisher_liveliness_lost_count == 1
        assert state.command_queue_drop_count == 3
