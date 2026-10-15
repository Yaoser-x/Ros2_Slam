#!/usr/bin/env python3
"""Expose chassis operations without claiming queue admission means applied."""

from dataclasses import dataclass
import time
from typing import Any

import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.task import Future
from robot_interfaces.action import ChassisOperation
from robot_interfaces.msg import ChassisLinkState, FirmwareControlState, FirmwareInfo
from std_srvs.srv import SetBool, Trigger


@dataclass
class _OperationRun:
    goal: Any
    response: Future
    completion: Future
    session: int
    status_sequence: Any
    info_generation: int
    started_at: float
    queued: bool = False


def _sequence_advanced(current: int, previous: int) -> bool:
    return 0 < ((int(current) - int(previous)) & 0xFFFFFFFF) < 0x80000000


class ChassisOpsNode(Node):
    def __init__(self) -> None:
        super().__init__("chassis_ops")
        self.declare_parameter("wire_adapter_name", "stm32_bridge")
        self.declare_parameter("operation_timeout_sec", 1.0)
        self.declare_parameter("state_timeout_sec", 0.25)
        self.declare_parameter("firmware_control_state_topic", "chassis/firmware_control_state")
        self.declare_parameter("firmware_info_topic", "chassis/firmware_info")
        self.declare_parameter("chassis_link_state_topic", "chassis/link_state")
        adapter = str(self.get_parameter("wire_adapter_name").value).strip("/")
        self.state = None
        self.state_at = None
        self.wire_session = 0
        self.info_generations = {}
        self.operations = []
        self.operation_timeout = float(self.get_parameter("operation_timeout_sec").value)
        self.state_timeout = float(self.get_parameter("state_timeout_sec").value)
        if min(self.operation_timeout, self.state_timeout) <= 0:
            raise ValueError("operation timing parameters must be positive")
        self.estop = self.create_client(SetBool, f"{adapter}/wire_estop")
        self.clear = self.create_client(Trigger, f"{adapter}/wire_clear_fault")
        self.line = self.create_client(SetBool, f"{adapter}/wire_line_ctrl")
        self.info = self.create_client(Trigger, f"{adapter}/wire_get_info")
        self.create_subscription(
            FirmwareControlState,
            str(self.get_parameter("firmware_control_state_topic").value),
            self._state,
            10,
        )
        self.create_subscription(
            FirmwareInfo, str(self.get_parameter("firmware_info_topic").value), self._info, 10
        )
        self.create_subscription(
            ChassisLinkState,
            str(self.get_parameter("chassis_link_state_topic").value),
            self._link,
            10,
        )
        self.server = ActionServer(
            self,
            ChassisOperation,
            "chassis/operation",
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
        )
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.02, self._tick, clock=self.steady_clock)

    def _state(self, msg):
        self.state = msg
        self.state_at = time.monotonic()

    def _info(self, msg):
        session = int(msg.wire_session_id)
        self.info_generations[session] = self.info_generations.get(session, 0) + 1

    def _link(self, msg):
        self.wire_session = int(msg.wire_session_id)

    def _goal(self, request):
        valid = request.operation in (
            ChassisOperation.Goal.OP_ESTOP_SET,
            ChassisOperation.Goal.OP_CLEAR_FAULT,
            ChassisOperation.Goal.OP_LINE_CONTROL,
            ChassisOperation.Goal.OP_GET_INFO,
        )
        if request.operation == ChassisOperation.Goal.OP_ESTOP_SET and not request.enable:
            valid = False
        return GoalResponse.ACCEPT if valid else GoalResponse.REJECT

    def _cancel(self, _goal):
        return CancelResponse.ACCEPT

    async def _execute(self, goal):
        op = goal.request.operation
        if goal.is_cancel_requested:
            return self._finish(goal, ChassisOperation.Result.RESULT_CANCELLED, "cancelled")
        if op == ChassisOperation.Goal.OP_CLEAR_FAULT and self.state and self.state.estop_active:
            return self._finish(goal, ChassisOperation.Result.RESULT_REJECTED, "ESTOP is active")
        session = self.wire_session
        sequence = (
            int(self.state.status_sequence)
            if self.state is not None and int(self.state.wire_session_id) == session
            else None
        )
        try:
            if op == ChassisOperation.Goal.OP_ESTOP_SET:
                response = self.estop.call_async(SetBool.Request(data=True))
            elif op == ChassisOperation.Goal.OP_CLEAR_FAULT:
                response = self.clear.call_async(Trigger.Request())
            elif op == ChassisOperation.Goal.OP_LINE_CONTROL:
                response = self.line.call_async(SetBool.Request(data=goal.request.enable))
            else:
                response = self.info.call_async(Trigger.Request())
            run = _OperationRun(
                goal,
                response,
                Future(),
                session,
                sequence,
                self.info_generations.get(session, 0),
                time.monotonic(),
            )
            self.operations.append(run)
            status, message = await run.completion
            return self._finish(goal, status, message)
        except Exception as exc:
            return self._finish(
                goal, ChassisOperation.Result.RESULT_REJECTED, f"wire request failed: {exc}"
            )

    def _tick(self):
        now = time.monotonic()
        for run in tuple(self.operations):
            op = run.goal.request.operation
            if run.goal.is_cancel_requested:
                self._complete(run, ChassisOperation.Result.RESULT_CANCELLED, "cancelled")
                continue
            if self.wire_session != run.session:
                self._complete(run, ChassisOperation.Result.RESULT_REJECTED, "wire session changed")
                continue
            if not run.queued and run.response.done():
                try:
                    response = run.response.result()
                except Exception as exc:
                    self._complete(run, ChassisOperation.Result.RESULT_REJECTED, str(exc))
                    continue
                if response is None or not response.success:
                    self._complete(
                        run,
                        ChassisOperation.Result.RESULT_REJECTED,
                        "wire adapter rejected request" if response is None else response.message,
                    )
                    continue
                run.queued = True
                if op in (
                    ChassisOperation.Goal.OP_CLEAR_FAULT,
                    ChassisOperation.Goal.OP_LINE_CONTROL,
                ):
                    # v3 has no request-correlated CLEAR_FAULT/LINE_CTRL result.
                    self._complete(
                        run,
                        ChassisOperation.Result.RESULT_QUEUED,
                        "wire request queued; Upper v3 exposes no request-correlated applied result",
                    )
                    continue
            state_is_new = bool(
                run.session != 0
                and self.state is not None
                and self.state_at is not None
                and int(self.state.wire_session_id) == run.session
                and run.started_at <= self.state_at <= now
                and now - self.state_at <= self.state_timeout
                and (
                    run.status_sequence is None
                    or _sequence_advanced(self.state.status_sequence, run.status_sequence)
                )
            )
            if (
                run.queued
                and op == ChassisOperation.Goal.OP_ESTOP_SET
                and state_is_new
                and self.state.estop_active
            ):
                self._complete(
                    run, ChassisOperation.Result.RESULT_APPLIED, "same-session ESTOP observed"
                )
            elif (
                run.queued
                and op == ChassisOperation.Goal.OP_GET_INFO
                and run.session != 0
                and self.info_generations.get(run.session, 0) > run.info_generation
            ):
                self._complete(
                    run, ChassisOperation.Result.RESULT_APPLIED, "new same-session HELLO observed"
                )
            elif now - run.started_at >= self.operation_timeout:
                self._complete(
                    run,
                    ChassisOperation.Result.RESULT_TIMEOUT,
                    "no confirming firmware evidence" if run.queued else "request was not queued",
                )

    def _complete(self, run, status, message):
        self.operations.remove(run)
        if not run.completion.done():
            run.completion.set_result((status, message))

    def _finish(self, goal, status, message):
        if status in (
            ChassisOperation.Result.RESULT_APPLIED,
            ChassisOperation.Result.RESULT_QUEUED,
        ):
            goal.succeed()
        elif status == ChassisOperation.Result.RESULT_CANCELLED:
            goal.canceled()
        else:
            goal.abort()
        return self._result(status, message)

    @staticmethod
    def _result(status, message):
        result = ChassisOperation.Result()
        result.status = status
        result.message = message
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ChassisOpsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if node.context.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
