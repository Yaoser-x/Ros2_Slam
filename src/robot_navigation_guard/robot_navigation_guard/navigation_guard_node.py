#!/usr/bin/env python3
"""Fail-closed public NavigateToPose proxy with cancel-and-reissue semantics."""

from dataclasses import dataclass
import time
from typing import Any, Optional

import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from robot_interfaces.msg import (
    ChassisLinkState,
    FirmwareControlState,
    HostControlState,
    MotionSupervisionState,
    NavigationGuardState,
    PlatformCompatibilityState,
)

from robot_navigation_guard.guard import GoalGuard


@dataclass
class _BackendRun:
    uuid: str
    goal: Any
    completion: Future
    started_at: float
    send_future: Optional[Future] = None
    result_future: Optional[Future] = None
    cancel_future: Optional[Future] = None
    handle: Any = None
    next_result_request_at: float = 0.0


class NavigationGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_guard")
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.declare_parameter("public_action", "navigate_to_pose")
        self.declare_parameter("backend_action", "nav2/navigate_to_pose")
        self.declare_parameter("goal_pose_topic", "goal_pose")
        self.declare_parameter("cancel_retry_sec", 0.20)
        self.declare_parameter("state_timeout_sec", 0.25)
        self.declare_parameter("backend_response_timeout_sec", 1.0)
        for name, default in (
            ("chassis_link_state_topic", "chassis/link_state"),
            ("firmware_control_state_topic", "chassis/firmware_control_state"),
            ("motion_supervision_topic", "motion/supervision_state"),
            ("compatibility_state_topic", "platform/compatibility_state"),
            ("host_control_state_topic", "chassis/host_control_state"),
            ("navigation_guard_state_topic", "navigation/guard_state"),
        ):
            self.declare_parameter(name, default)
        self.guard = GoalGuard()
        self.link_safe = self.firmware_safe = self.supervision_safe = False
        self.compatible = self.host_safe = False
        self.reason_flags = (
            NavigationGuardState.REASON_LINK
            | NavigationGuardState.REASON_FIRMWARE
            | NavigationGuardState.REASON_SUPERVISION
            | NavigationGuardState.REASON_COMPATIBILITY
            | NavigationGuardState.REASON_HOST_CONTROL
        )
        self.backend_handle = None
        self.last_cancel_at = None
        self.cancel_retry_sec = float(self.get_parameter("cancel_retry_sec").value)
        self.state_timeout_sec = float(self.get_parameter("state_timeout_sec").value)
        self.backend_response_timeout_sec = float(
            self.get_parameter("backend_response_timeout_sec").value
        )
        if (
            min(self.cancel_retry_sec, self.state_timeout_sec, self.backend_response_timeout_sec)
            <= 0
        ):
            raise ValueError("guard timing parameters must be positive")
        self.state_received_at = {}
        self.link_session = self.firmware_session = self.compatibility_session = 0
        self.supervision_session = 0
        self.run: Optional[_BackendRun] = None
        self.pose_send_future: Optional[Future] = None
        self.backend = ActionClient(
            self, NavigateToPose, str(self.get_parameter("backend_action").value)
        )
        self.server = ActionServer(
            self,
            NavigateToPose,
            str(self.get_parameter("public_action").value),
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
        )
        self.pose_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter("public_action").value)
        )
        topic = lambda name: str(self.get_parameter(name).value)
        self.publisher = self.create_publisher(
            NavigationGuardState, topic("navigation_guard_state_topic"), 10
        )
        self.create_subscription(
            ChassisLinkState, topic("chassis_link_state_topic"), self._on_link, 10
        )
        self.create_subscription(
            FirmwareControlState, topic("firmware_control_state_topic"), self._on_firmware, 10
        )
        self.create_subscription(
            MotionSupervisionState, topic("motion_supervision_topic"), self._on_supervision, 10
        )
        self.create_subscription(
            PlatformCompatibilityState,
            topic("compatibility_state_topic"),
            self._on_compatibility,
            10,
        )
        self.create_subscription(
            HostControlState, topic("host_control_state_topic"), self._on_host, 10
        )
        self.create_subscription(PoseStamped, topic("goal_pose_topic"), self._on_goal_pose, 10)
        self.create_timer(0.05, self._cancel_tick, clock=self.steady_clock)

    def _on_goal_pose(self, pose) -> None:
        self._evaluate()
        if self.pose_send_future is not None:
            self.get_logger().warning("RViz goal rejected: previous public goal admission pending")
            return
        if not self.guard.ready:
            self.get_logger().warning(
                f"RViz goal rejected: guard closed or busy, reasons={self.reason_flags}"
            )
            return
        if not self.pose_client.server_is_ready() or not self.backend.server_is_ready():
            self.get_logger().warning("RViz goal rejected: navigation Action server unavailable")
            return
        request = NavigateToPose.Goal()
        request.pose = pose
        try:
            self.pose_send_future = self.pose_client.send_goal_async(request)
        except Exception as exc:
            self.get_logger().error(f"RViz public goal submission failed: {exc}")

    def _poll_goal_pose(self) -> None:
        future = self.pose_send_future
        if future is None or not future.done():
            return
        self.pose_send_future = None
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"RViz public goal admission failed: {exc}")
            return
        if handle is None or not handle.accepted:
            self.get_logger().warning("RViz goal rejected by the public navigation guard")
        else:
            self.get_logger().info("RViz goal admitted through the public navigation guard")

    def _on_link(self, msg) -> None:
        self.state_received_at["link"] = time.monotonic()
        self.link_session = int(msg.wire_session_id)
        self.link_safe = bool(
            msg.link_state
            in (ChassisLinkState.STATE_WIRE_REARM_READY, ChassisLinkState.STATE_DRIVE_ACTIVE)
            and not msg.wire_rearm_required
            and msg.protocol_compatible
        )
        self._evaluate()

    def _on_firmware(self, msg) -> None:
        self.state_received_at["firmware"] = time.monotonic()
        self.firmware_session = int(msg.wire_session_id)
        self.firmware_safe = not msg.estop_active and not msg.fault_stop_active
        self._evaluate()

    def _on_supervision(self, msg) -> None:
        self.state_received_at["supervision"] = time.monotonic()
        self.supervision_session = int(msg.observation_session_id)
        self.supervision_safe = bool(
            msg.level
            in (
                MotionSupervisionState.LEVEL_NORMAL,
                MotionSupervisionState.LEVEL_WARN,
                MotionSupervisionState.LEVEL_DEGRADED,
            )
            and not msg.release_host_candidate
        )
        self._evaluate()

    def _on_compatibility(self, msg) -> None:
        self.state_received_at["compatibility"] = time.monotonic()
        self.compatibility_session = int(msg.wire_session_id)
        self.compatible = bool(
            msg.permit_navigation
            and msg.observation_ready
            and int(msg.observation_session_id) == self.compatibility_session
        )
        self._evaluate()

    def _on_host(self, msg) -> None:
        self.state_received_at["host"] = time.monotonic()
        # WAIT_FRESH_HOST_INTENT is intentionally admissible: the newly
        # accepted public goal is the fresh intent that creates the next Host
        # command epoch.  All earlier quiet/wire gates remain closed.
        self.host_safe = bool(
            msg.control_state == HostControlState.STATE_WAIT_FRESH_HOST_INTENT
            or (msg.control_state == HostControlState.STATE_HOST_ACTIVE and not msg.rearm_required)
        )
        self._evaluate()

    def _evaluate(self) -> None:
        now = time.monotonic()

        def fresh(name):
            received = self.state_received_at.get(name)
            return received is not None and 0.0 <= now - received <= self.state_timeout_sec

        flags = 0
        if not self.link_safe or not fresh("link") or self.link_session == 0:
            flags |= NavigationGuardState.REASON_LINK
        if (
            not self.firmware_safe
            or not fresh("firmware")
            or self.firmware_session != self.link_session
        ):
            flags |= NavigationGuardState.REASON_FIRMWARE
        if (
            not self.supervision_safe
            or not fresh("supervision")
            or self.supervision_session != self.link_session
        ):
            flags |= NavigationGuardState.REASON_SUPERVISION
        if (
            not self.compatible
            or not fresh("compatibility")
            or self.compatibility_session != self.link_session
        ):
            flags |= NavigationGuardState.REASON_COMPATIBILITY
        if not self.host_safe or not fresh("host"):
            flags |= NavigationGuardState.REASON_HOST_CONTROL
        self.reason_flags = flags
        if flags == 0:
            self.guard.recover()
        else:
            self.guard.revoke()
        self._publish()

    def _goal(self, _request):
        self._evaluate()
        return GoalResponse.ACCEPT if self.guard.ready else GoalResponse.REJECT

    def _cancel(self, _goal):
        self.guard.revoke()
        self._publish()
        return CancelResponse.ACCEPT

    async def _execute(self, goal):
        if goal.is_cancel_requested:
            goal.canceled()
            return NavigateToPose.Result()
        uuid = bytes(goal.goal_id.uuid).hex()
        if not self.guard.observe_goal(uuid):
            goal.canceled() if goal.is_cancel_requested else goal.abort()
            return NavigateToPose.Result()
        self._publish()
        if not self.backend.server_is_ready():
            self.guard.revoke()
            self.guard.terminate(uuid)
            goal.abort()
            self._publish()
            return NavigateToPose.Result()
        run = _BackendRun(uuid, goal, Future(), time.monotonic())
        self.run = run
        self.last_cancel_at = None
        try:
            run.send_future = self.backend.send_goal_async(
                goal.request,
                goal_uuid=goal.goal_id,
                feedback_callback=lambda feedback: (
                    goal.publish_feedback(feedback.feedback) if goal.is_active else None
                ),
            )
            status, result, was_revoked = await run.completion
            if goal.is_cancel_requested:
                goal.canceled()
            elif status == GoalStatus.STATUS_SUCCEEDED and not was_revoked:
                goal.succeed()
            else:
                goal.abort()
            return result
        except Exception as exc:
            # A failed transport future does not prove the remote goal stopped.
            # Keep its ownership until a terminal backend result is observed.
            self.guard.revoke()
            self.get_logger().error(f"Navigation backend failed: {exc}")
            if goal.is_active:
                goal.canceled() if goal.is_cancel_requested else goal.abort()
            self._publish()
            return NavigateToPose.Result()

    def _cancel_tick(self) -> None:
        self._poll_goal_pose()
        self._evaluate()
        run = self.run
        now = time.monotonic()
        if run is not None:
            if run.goal.is_cancel_requested:
                self.guard.revoke()
            if run.handle is None and run.send_future is not None and run.send_future.done():
                try:
                    run.handle = run.send_future.result()
                except Exception as exc:
                    self._backend_error(run, exc)
                else:
                    if run.handle is None or not run.handle.accepted:
                        self._complete_backend(
                            run, GoalStatus.STATUS_ABORTED, NavigateToPose.Result()
                        )
                        return
                    self.backend_handle = run.handle
            if run.handle is None:
                if now - run.started_at >= self.backend_response_timeout_sec:
                    self._backend_error(run, RuntimeError("backend goal response timed out"))
            elif run.result_future is None and now >= run.next_result_request_at:
                try:
                    run.result_future = run.handle.get_result_async()
                except Exception as exc:
                    run.next_result_request_at = now + self.cancel_retry_sec
                    self._backend_error(run, exc)
            if run.result_future is not None and run.result_future.done():
                try:
                    wrapped = run.result_future.result()
                    if wrapped.status not in (
                        GoalStatus.STATUS_SUCCEEDED,
                        GoalStatus.STATUS_ABORTED,
                        GoalStatus.STATUS_CANCELED,
                    ):
                        raise RuntimeError("backend result did not prove goal termination")
                except Exception as exc:
                    run.result_future = None
                    run.next_result_request_at = now + self.cancel_retry_sec
                    self._backend_error(run, exc)
                else:
                    self._complete_backend(run, wrapped.status, wrapped.result)
                    return
        if not self.guard.cancel_pending or self.backend_handle is None:
            return
        if run.cancel_future is not None:
            if not run.cancel_future.done():
                return
            try:
                run.cancel_future.result()
            except Exception as exc:
                self.get_logger().error(f"Backend cancel response failed; will retry: {exc}")
            run.cancel_future = None
        if self.last_cancel_at is not None and now - self.last_cancel_at < self.cancel_retry_sec:
            return
        self.last_cancel_at = now
        self.guard.note_cancel_attempt()
        try:
            run.cancel_future = self.backend_handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().error(f"Backend cancel request failed; will retry: {exc}")
        self._publish()

    def _backend_error(self, run, exc) -> None:
        if not run.completion.done():
            self.guard.revoke()
            run.completion.set_exception(exc)
            self._publish()

    def _complete_backend(self, run, status, result) -> None:
        was_revoked = self.guard.cancel_pending or run.uuid == self.guard.revoked_uuid
        self.guard.terminate(run.uuid)
        if run.cancel_future is not None and not run.cancel_future.done():
            # The terminal result is sufficient proof; abandon only the local
            # cancel-response wait so ActionClient can discard its pending entry.
            run.cancel_future.cancel()
        self.backend_handle = None
        self.run = None
        if not run.completion.done():
            run.completion.set_result((status, result, was_revoked))
        self._publish()

    def _publish(self) -> None:
        msg = NavigationGuardState()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.guard.cancel_pending:
            msg.state = (
                NavigationGuardState.STATE_CANCEL_PENDING
                if self.guard.cancel_attempts == 0
                else NavigationGuardState.STATE_WAIT_OLD_GOAL_TERMINATED
            )
        elif self.guard.active_uuid:
            msg.state = NavigationGuardState.STATE_ACTIVE
        elif self.guard.ready:
            msg.state = NavigationGuardState.STATE_READY_FOR_NEW_GOAL
        else:
            msg.state = NavigationGuardState.STATE_CLOSED
        msg.revoke_generation = self.guard.revoke_generation
        msg.goal_generation = self.guard.goal_generation
        msg.cancel_attempts = self.guard.cancel_attempts
        msg.reason_flags = self.reason_flags
        msg.active_goal_uuid = self.guard.active_uuid
        msg.revoked_goal_uuid = self.guard.revoked_uuid
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationGuardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if node.context.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
