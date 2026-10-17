from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tests"))
from ros_adapter_helpers import ManualFuture, adapter_classes
from robot_navigation_guard.guard import GoalGuard


def make_node():
    clock = NS(value=10.0)
    classes = adapter_classes(
        ROOT / "src/robot_navigation_guard/robot_navigation_guard/navigation_guard_node.py",
        time=NS(monotonic=lambda: clock.value),
        GoalGuard=GoalGuard,
        GoalStatus=NS(STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6),
        NavigateToPose=NS(Result=lambda: NS(), Goal=lambda: NS(pose=None)),
        HostMotionCommand=NS(HOST_SUBSOURCE_NONE=0, HOST_SUBSOURCE_NAV=3),
        HostControlState=NS(STATE_WAIT_FRESH_HOST_INTENT=3, STATE_HOST_ACTIVE=4),
        NavigationGuardState=NS(
            REASON_LINK=1,
            REASON_FIRMWARE=2,
            REASON_SUPERVISION=4,
            REASON_COMPATIBILITY=8,
            REASON_HOST_CONTROL=16,
        ),
    )
    node = classes.NavigationGuardNode.__new__(classes.NavigationGuardNode)
    node.guard = GoalGuard(safe=True)
    node.link_safe = node.firmware_safe = node.supervision_safe = node.compatible = (
        node.host_safe
    ) = True
    node.link_session = node.firmware_session = node.supervision_session = (
        node.compatibility_session
    ) = 7
    node.state_received_at = {
        name: clock.value for name in ("link", "firmware", "supervision", "compatibility", "host")
    }
    node.state_timeout_sec = 0.25
    node.cancel_retry_sec = 0.20
    node.backend_response_timeout_sec = 1.0
    node.last_cancel_at = None
    node.backend_handle = node.run = None
    node.pose_send_future = None
    node._publish = lambda: None
    node.get_logger = lambda: NS(error=lambda _: None, warning=lambda _: None, info=lambda _: None)
    send, result = ManualFuture(), ManualFuture()
    handle = NS(accepted=True, get_result_async=lambda: result, cancel_count=0)

    def cancel():
        handle.cancel_count += 1
        return ManualFuture()

    handle.cancel_goal_async = cancel
    node.backend = NS(server_is_ready=lambda: True, send_goal_async=lambda *args, **kwargs: send)
    goal = NS(
        goal_id=NS(uuid=bytes.fromhex("01" * 16)),
        request=NS(),
        is_cancel_requested=False,
        is_active=True,
        terminal="",
    )
    goal.publish_feedback = lambda _: None
    goal.abort = lambda: setattr(goal, "terminal", "abort")
    goal.canceled = lambda: setattr(goal, "terminal", "cancel")
    goal.succeed = lambda: setattr(goal, "terminal", "success")
    return node, clock, send, result, handle, goal


@pytest.mark.parametrize("source", [1, 2, 4, 255])
def test_host_takeover_cancels_navigation_and_neutral_does_not_resume_old_goal(source):
    node, clock, send, result, handle, goal = make_node()
    coroutine = node._execute(goal)
    coroutine.send(None)
    send.set_result(handle)
    node._cancel_tick()

    node._on_host(NS(control_state=4, rearm_required=False, selected_host_subsource=source))
    assert node.reason_flags == 16
    assert node.guard.cancel_pending and not node.guard.safe
    node._cancel_tick()
    assert handle.cancel_count == 1

    node._on_host(NS(control_state=3, rearm_required=True, selected_host_subsource=0))
    node._cancel_tick()
    assert node.guard.active_uuid and not node.guard.ready
    result.set_result(NS(status=5, result=NS()))
    node._cancel_tick()
    with pytest.raises(StopIteration):
        coroutine.send(None)
    assert goal.terminal == "abort"
    assert node.guard.ready
    assert not node.guard.observe_goal(bytes(goal.goal_id.uuid).hex())
    assert node.guard.observe_goal("fresh-explicit-goal")


@pytest.mark.parametrize("source", [2, 3, 4, 5, 255])
def test_physical_control_takeover_revokes_navigation(source):
    node, clock, send, result, handle, goal = make_node()
    coroutine = node._execute(goal)
    coroutine.send(None)
    send.set_result(handle)
    node._cancel_tick()
    node._on_firmware(
        NS(
            wire_session_id=7,
            estop_active=False,
            fault_stop_active=False,
            firmware_control_source=source,
        )
    )
    assert node.reason_flags == 2
    node._cancel_tick()
    assert handle.cancel_count == 1 and node.guard.cancel_pending
    result.set_result(NS(status=5, result=NS()))
    node._cancel_tick()
    with pytest.raises(StopIteration):
        coroutine.send(None)
    assert goal.terminal == "abort"


@pytest.mark.parametrize("firmware_source", [0, 1])
@pytest.mark.parametrize("host_source", [0, 3])
def test_idle_and_navigation_sources_remain_admissible(firmware_source, host_source):
    node, *_ = make_node()
    node._on_firmware(
        NS(
            wire_session_id=7,
            estop_active=False,
            fault_stop_active=False,
            firmware_control_source=firmware_source,
        )
    )
    node._on_host(NS(control_state=4, rearm_required=False, selected_host_subsource=host_source))
    assert node.guard.ready and node.reason_flags == 0


def test_pending_navigation_yields_native_future_and_releases_only_after_backend_terminal():
    node, clock, send, result, handle, goal = make_node()
    coroutine = node._execute(goal)
    completion = coroutine.send(None)
    assert isinstance(completion, ManualFuture)
    send.set_result(handle)
    node._cancel_tick()
    node.guard.revoke()
    node._cancel_tick()
    assert handle.cancel_count == 1
    assert not node.guard.ready
    assert not node.guard.observe_goal("new")
    result.set_result(NS(status=5, result=NS()))
    node._cancel_tick()
    with pytest.raises(StopIteration):
        coroutine.send(None)
    assert goal.terminal == "abort"
    assert not node.guard.active_uuid
    assert node.run is None and node.backend_handle is None


def test_compatibility_publisher_disappearance_revokes_even_with_other_states_fresh():
    node, clock, *_ = make_node()
    clock.value += 0.30
    for name in node.state_received_at:
        if name != "compatibility":
            node.state_received_at[name] = clock.value
    node._cancel_tick()
    assert not node.guard.safe
    assert node.reason_flags == 8


def test_backend_exception_keeps_old_goal_owned_until_result_retry_confirms_termination():
    node, clock, send, result, handle, goal = make_node()
    coroutine = node._execute(goal)
    coroutine.send(None)
    send.set_result(handle)
    node._cancel_tick()
    result.set_exception(RuntimeError("lost result response"))
    node._cancel_tick()
    with pytest.raises(StopIteration):
        coroutine.send(None)
    assert goal.terminal == "abort"
    assert node.guard.active_uuid and node.guard.cancel_pending
    assert not node.guard.observe_goal("new")
    retry = ManualFuture()
    retry.set_result(NS(status=6, result=NS()))
    handle.get_result_async = lambda: retry
    clock.value += 0.21
    node._cancel_tick()
    assert not node.guard.active_uuid and node.run is None


def test_late_backend_admission_after_response_timeout_is_cancelled_before_reopening():
    node, clock, send, result, handle, goal = make_node()
    coroutine = node._execute(goal)
    coroutine.send(None)
    clock.value += 1.01
    node._cancel_tick()
    with pytest.raises(StopIteration):
        coroutine.send(None)
    assert goal.terminal == "abort"
    assert node.guard.active_uuid and not node.guard.ready
    send.set_result(handle)
    node._cancel_tick()
    assert handle.cancel_count == 1
    assert node.guard.active_uuid
    result.set_result(NS(status=6, result=NS()))
    node._cancel_tick()
    assert node.run is None and not node.guard.active_uuid


def test_unanswered_cancel_keeps_one_request_and_old_goal_owned():
    node, clock, send, result, handle, goal = make_node()
    coroutine = node._execute(goal)
    coroutine.send(None)
    send.set_result(handle)
    node._cancel_tick()
    node.guard.revoke()
    node._cancel_tick()
    cancel_future = node.run.cancel_future
    for _ in range(20):
        clock.value += 0.21
        node._cancel_tick()
    assert handle.cancel_count == 1
    assert node.run.cancel_future is cancel_future
    assert node.guard.active_uuid and not node.guard.ready
    cancel_future.set_result(NS())
    node._cancel_tick()
    assert handle.cancel_count == 2
    result.set_result(NS(status=6, result=NS()))
    pending_cancel = node.run.cancel_future
    node._cancel_tick()
    assert pending_cancel.done()
    with pytest.raises(StopIteration):
        coroutine.send(None)


def test_rviz_pose_uses_public_action_once_and_does_not_queue_busy_goals():
    node, clock, *_ = make_node()
    requests, pending = [], ManualFuture()
    node.pose_client = NS(
        server_is_ready=lambda: True,
        send_goal_async=lambda request: requests.append(request) or pending,
    )
    node.backend.send_goal_async = lambda *_args, **_kwargs: pytest.fail("backend bypass")
    pose = NS(frame="map", target=1)
    node._on_goal_pose(pose)
    assert len(requests) == 1 and requests[0].pose is pose
    assert node.pose_send_future is pending
    for _ in range(10):
        node._on_goal_pose(NS(target=2))
        node._poll_goal_pose()
    assert len(requests) == 1 and node.pose_send_future is pending
    pending.set_result(NS(accepted=True))
    node._poll_goal_pose()
    assert node.pose_send_future is None
    assert node.guard.observe_goal("active-backend")
    node._on_goal_pose(NS(target=3))
    assert len(requests) == 1
    node.guard.revoke()
    node.guard.terminate("active-backend")
    node._evaluate()
    assert len(requests) == 1, "recovery must not automatically replay a discarded RViz goal"


@pytest.mark.parametrize("reason", ["stale", "public_unready", "backend_unready"])
def test_rviz_pose_rejects_unavailable_permissions_and_servers(reason):
    node, clock, *_ = make_node()
    requests = []
    node.pose_client = NS(
        server_is_ready=lambda: reason != "public_unready",
        send_goal_async=lambda request: requests.append(request),
    )
    node.backend.server_is_ready = lambda: reason != "backend_unready"
    if reason == "stale":
        clock.value += 0.30
    node._on_goal_pose(NS())
    assert not requests and node.pose_send_future is None
    for key in node.state_received_at:
        node.state_received_at[key] = clock.value
    node._cancel_tick()
    assert not requests, "readiness recovery must not replay the previously rejected pose"
