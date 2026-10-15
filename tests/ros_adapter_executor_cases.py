"""Actual rclpy integration scenarios. Never imported into host-mocked pytest."""

import sys
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from robot_interfaces.action import ChassisOperation
from robot_interfaces.msg import (
    ChassisLinkState,
    FirmwareControlState,
    FirmwareInfo,
    HostControlState,
    HostMotionCommand,
    MotionSupervisionState,
    PlatformCompatibilityState,
    WheelObservation,
)
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool, Trigger

from robot_chassis_ops.chassis_ops_node import ChassisOpsNode
from robot_control.cmd_vel_mux import CmdVelMuxNode
from robot_config.compatibility_node import PlatformCompatibilityNode
from robot_navigation_guard.navigation_guard_node import NavigationGuardNode
from robot_state_estimation.formal_odometry_node import FormalOdometryNode
from robot_supervision.motion_supervisor_node import MotionSupervisorNode
from robot_verification.fake_base_node import FakeBaseNode


def spin_until(executor, condition, timeout=3.0):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert condition(), "ROS condition did not complete within its bounded wait"


def spin_for(executor, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)


def navigation(executor, nodes, *, pending_cancel=False):
    guard, backend, fixture = NavigationGuardNode(), Node("backend"), Node("fixture")
    nodes.extend([guard, backend, fixture])
    finish = Future()
    cancel_response = Future()
    cancel_calls = [0]
    if not pending_cancel:
        cancel_response.set_result(True)
    handles = []

    async def execute(goal):
        handles.append(goal)
        await finish
        goal.canceled() if goal.is_cancel_requested else goal.succeed()
        return NavigateToPose.Result()

    async def cancel(_goal):
        cancel_calls[0] += 1
        await cancel_response
        return CancelResponse.ACCEPT

    server = ActionServer(
        backend,
        NavigateToPose,
        "nav2/navigate_to_pose",
        execute_callback=execute,
        cancel_callback=cancel,
    )
    types = [
        ChassisLinkState,
        FirmwareControlState,
        MotionSupervisionState,
        PlatformCompatibilityState,
        HostControlState,
    ]
    topics = [
        "chassis/link_state",
        "chassis/firmware_control_state",
        "motion/supervision_state",
        "platform/compatibility_state",
        "chassis/host_control_state",
    ]
    publishers = [fixture.create_publisher(kind, topic, 10) for kind, topic in zip(types, topics)]
    publish_compatibility = [True]

    def publish():
        link = ChassisLinkState()
        link.link_state = link.STATE_WIRE_REARM_READY
        link.protocol_compatible, link.wire_session_id = True, 7
        firmware = FirmwareControlState()
        firmware.wire_session_id = 7
        supervision = MotionSupervisionState()
        supervision.level, supervision.observation_session_id = supervision.LEVEL_NORMAL, 7
        compatibility = PlatformCompatibilityState()
        compatibility.permit_navigation = compatibility.observation_ready = True
        compatibility.wire_session_id = compatibility.observation_session_id = 7
        host = HostControlState()
        host.control_state = host.STATE_WAIT_FRESH_HOST_INTENT
        for index, message in enumerate([link, firmware, supervision, compatibility, host]):
            if index != 3 or publish_compatibility[0]:
                message.header.stamp = fixture.get_clock().now().to_msg()
                publishers[index].publish(message)

    timer = fixture.create_timer(0.02, publish)
    client = ActionClient(fixture, NavigateToPose, "navigate_to_pose")
    for node in nodes:
        executor.add_node(node)
    spin_until(executor, lambda: guard.guard.ready and client.server_is_ready())
    sent = client.send_goal_async(NavigateToPose.Goal())
    spin_until(executor, sent.done)
    public = sent.result()
    assert public.accepted
    result = public.get_result_async()
    spin_until(executor, lambda: bool(handles) and guard.backend_handle is not None)
    spin_for(executor, 0.1)
    assert not result.done() and guard.guard.active_uuid
    publish_compatibility[0] = False
    if pending_cancel:
        spin_until(executor, lambda: cancel_calls[0] == 1)
        assert len(guard.backend._pending_cancel_requests) == 1
        spin_for(executor, 0.65)
        assert cancel_calls[0] == 1
        assert len(guard.backend._pending_cancel_requests) == 1
        assert guard.guard.active_uuid and not guard.guard.ready
        cancel_response.set_result(True)
    spin_until(executor, lambda: guard.guard.cancel_pending and handles[0].is_cancel_requested)
    assert not result.done(), "cancel admission is not backend termination evidence"
    publish_compatibility[0] = True
    spin_for(executor, 0.05)
    second = client.send_goal_async(NavigateToPose.Goal())
    spin_until(executor, second.done)
    assert not second.result().accepted
    finish.set_result(True)
    spin_until(executor, lambda: result.done() and guard.guard.ready)
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert guard.backend_handle is None and guard.run is None
    spin_until(executor, lambda: not guard.backend._pending_cancel_requests)
    server.destroy()
    client.destroy()
    fixture.destroy_timer(timer)


def navigation_cancel_pending(executor, nodes):
    navigation(executor, nodes, pending_cancel=True)


def navigation_pose(executor, nodes):
    guard, backend, fixture = NavigationGuardNode(), Node("backend"), Node("fixture")
    nodes.extend([guard, backend, fixture])
    finish, handles = Future(), []

    async def execute(goal):
        handles.append(goal)
        await finish
        goal.canceled() if goal.is_cancel_requested else goal.succeed()
        return NavigateToPose.Result()

    server = ActionServer(
        backend,
        NavigateToPose,
        "nav2/navigate_to_pose",
        execute_callback=execute,
        cancel_callback=lambda _goal: CancelResponse.ACCEPT,
    )
    types = [
        ChassisLinkState,
        FirmwareControlState,
        MotionSupervisionState,
        PlatformCompatibilityState,
        HostControlState,
    ]
    topics = [
        "chassis/link_state",
        "chassis/firmware_control_state",
        "motion/supervision_state",
        "platform/compatibility_state",
        "chassis/host_control_state",
    ]
    publishers = [fixture.create_publisher(kind, topic, 10) for kind, topic in zip(types, topics)]
    permitted = [True]

    def publish():
        link = ChassisLinkState()
        link.link_state = link.STATE_WIRE_REARM_READY
        link.protocol_compatible, link.wire_session_id = True, 7
        firmware = FirmwareControlState()
        firmware.wire_session_id = 7
        supervision = MotionSupervisionState()
        supervision.level, supervision.observation_session_id = supervision.LEVEL_NORMAL, 7
        compatibility = PlatformCompatibilityState()
        compatibility.permit_navigation = permitted[0]
        compatibility.observation_ready = True
        compatibility.wire_session_id = compatibility.observation_session_id = 7
        host = HostControlState()
        host.control_state = host.STATE_WAIT_FRESH_HOST_INTENT
        for publisher, message in zip(
            publishers, [link, firmware, supervision, compatibility, host]
        ):
            message.header.stamp = fixture.get_clock().now().to_msg()
            publisher.publish(message)

    timer = fixture.create_timer(0.02, publish)
    pose_publisher = fixture.create_publisher(PoseStamped, "goal_pose", 10)
    bypass_messages = []
    fixture.create_subscription(
        PoseStamped, "nav2/goal_pose", lambda message: bypass_messages.append(message), 10
    )
    for node in nodes:
        executor.add_node(node)
    spin_until(
        executor,
        lambda: guard.guard.ready
        and guard.pose_client.server_is_ready()
        and guard.backend.server_is_ready()
        and pose_publisher.get_subscription_count() > 0,
    )
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x, pose.pose.orientation.w = 1.25, 1.0
    pose_publisher.publish(pose)
    spin_until(executor, lambda: bool(handles) and guard.pose_send_future is None)
    backend_id = bytes(handles[0].goal_id.uuid)
    assert backend_id in guard.server._goal_handles, "RViz goal must traverse the public Action"
    assert guard.guard.active_uuid == backend_id.hex()
    assert handles[0].request.pose.pose.position.x == 1.25
    assert handles[0].request.pose.header.frame_id == "map"
    pose_publisher.publish(pose)
    spin_for(executor, 0.10)
    assert len(handles) == 1, "a busy RViz goal must be rejected, not queued"
    permitted[0] = False
    spin_until(executor, lambda: handles[0].is_cancel_requested)
    pose_publisher.publish(pose)
    spin_for(executor, 0.10)
    assert len(handles) == 1
    finish.set_result(True)
    spin_until(executor, lambda: guard.run is None)
    pose_publisher.publish(pose)
    spin_for(executor, 0.10)
    assert len(handles) == 1, "permission denial must not reach the backend"
    permitted[0] = True
    spin_until(executor, lambda: guard.guard.ready)
    spin_for(executor, 0.10)
    assert len(handles) == 1, "recovery must not replay rejected RViz poses"
    assert not bypass_messages
    pose_publisher.publish(pose)
    spin_until(executor, lambda: len(handles) == 2 and guard.run is None)
    assert bytes(handles[1].goal_id.uuid) in guard.server._goal_handles
    assert guard.guard.goal_generation == 2
    fixture.destroy_timer(timer)
    server.destroy()


def rearm_late_subscriber(executor, nodes):
    mux, supervisor, fixture = CmdVelMuxNode(), MotionSupervisorNode(), Node("fixture")
    nodes.extend([mux, supervisor, fixture, PlatformCompatibilityNode()])
    commands = []
    fixture.create_subscription(
        HostMotionCommand, "chassis/host_motion_command", lambda msg: commands.append(msg), 10
    )
    early_link = fixture.create_publisher(ChassisLinkState, "chassis/link_state", 10)

    def publish_unready():
        link = ChassisLinkState()
        link.link_state = link.STATE_WAIT_POST_CLEAR_DISABLE_ACK
        link.protocol_compatible, link.wire_session_id = True, 0xFA4E000000000001
        link.wire_rearm_required = True
        early_link.publish(link)

    early_timer = fixture.create_timer(0.02, publish_unready)
    for node in nodes:
        executor.add_node(node)
    spin_until(executor, lambda: bool(commands))
    first = commands[0]
    assert not first.enable and first.command_epoch
    spin_until(
        executor,
        lambda: mux.motion_state is not None
        and mux.motion_state.level == MotionSupervisionState.LEVEL_CRITICAL
        and mux.motion_state.reason_flags & MotionSupervisionState.REASON_OBSERVATION_STALE,
    )
    # The real provider's volatile subscription does not exist until after the
    # first release's 120ms lifespan. No explicit discovery wait is needed by
    # production: subsequent bounded retries must establish its disabled epoch.
    spin_for(executor, 0.25)
    assert supervisor.last_observation is None
    assert mux.motion_state.level == MotionSupervisionState.LEVEL_CRITICAL
    assert mux.motion_state.reason_flags & MotionSupervisionState.REASON_OBSERVATION_STALE
    assert mux.rearm_reason_flags & HostControlState.REARM_MOTION_CRITICAL
    assert mux.rearm_required and not mux.wire_ready and not mux.last_active
    assert mux.gate_state in (
        HostControlState.STATE_HOST_CLEARED,
        HostControlState.STATE_WAIT_SOURCE_QUIET,
        HostControlState.STATE_WAIT_WIRE_READY,
    ), "without wheel evidence the Host gate must remain unlicensed"
    assert all(not message.enable for message in commands)
    fixture.destroy_timer(early_timer)
    fake = FakeBaseNode()
    nodes.append(fake)
    executor.add_node(fake)
    spin_until(
        executor,
        lambda: not fake.rearm_required
        and mux.wire_ready
        and mux.gate_state == HostControlState.STATE_WAIT_FRESH_HOST_INTENT,
    )
    assert fake.command_epoch == first.command_epoch
    assert fake.command_sequence > first.sequence, "the late reader must miss the initial release"
    assert all(not message.enable for message in commands)
    assert all(message.command_epoch == first.command_epoch for message in commands)
    sequence = mux.sequence
    spin_for(executor, 0.20)
    assert mux.sequence == sequence, "wire readiness must stop the disable retry"
    assert mux.rearm_required and not mux.last_active and not fake.enabled
    assert supervisor.last_command is not None and not supervisor.last_command.enable


def compatibility_process_loss(executor, nodes):
    mux, supervisor, fake = CmdVelMuxNode(), MotionSupervisorNode(), FakeBaseNode()
    compatibility, fixture = PlatformCompatibilityNode(), Node("fixture")
    nodes.extend([mux, supervisor, fake, compatibility, fixture])
    for node in nodes:
        executor.add_node(node)
    spin_until(executor, lambda: mux.gate_state == HostControlState.STATE_WAIT_FRESH_HOST_INTENT)
    source = fixture.create_publisher(Twist, "cmd_vel/test", 10)
    keep_publishing = [True]

    def publish():
        if keep_publishing[0]:
            command = Twist()
            command.linear.x = 0.05
            source.publish(command)

    timer = fixture.create_timer(0.05, publish)
    spin_until(executor, lambda: mux.last_active and fake.enabled)
    lost_at = time.monotonic()
    executor.remove_node(compatibility)
    compatibility.destroy_node()
    nodes.remove(compatibility)
    spin_until(executor, lambda: mux.rearm_required and not mux.last_active and not fake.enabled)
    assert time.monotonic() - lost_at < 0.8
    assert mux.rearm_reason_flags & HostControlState.REARM_PROTOCOL_INCOMPATIBLE
    replacement = PlatformCompatibilityNode()
    nodes.append(replacement)
    executor.add_node(replacement)
    spin_for(executor, 0.4)
    assert (
        mux.rearm_required and not fake.enabled
    ), "node recovery must not replay an ongoing old source"
    keep_publishing[0] = False
    spin_until(executor, lambda: mux.gate_state == HostControlState.STATE_WAIT_FRESH_HOST_INTENT)
    assert not fake.enabled
    keep_publishing[0] = True
    spin_until(executor, lambda: mux.last_active and fake.enabled)
    fixture.destroy_timer(timer)


def operations(executor, nodes):
    ops, wire, fixture = ChassisOpsNode(), Node("wire"), Node("fixture")
    nodes.extend([ops, wire, fixture])
    queue_releases = []

    async def pending_clear(_request, response):
        queue_release = Future()
        queue_releases.append(queue_release)
        await queue_release
        response.success = True
        return response

    def queued(_request, response):
        response.success = True
        return response

    wire.create_service(Trigger, "stm32_bridge/wire_clear_fault", pending_clear)
    wire.create_service(SetBool, "stm32_bridge/wire_estop", queued)
    wire.create_service(Trigger, "stm32_bridge/wire_get_info", queued)
    links = fixture.create_publisher(ChassisLinkState, "chassis/link_state", 10)
    states = fixture.create_publisher(FirmwareControlState, "chassis/firmware_control_state", 10)
    infos = fixture.create_publisher(FirmwareInfo, "chassis/firmware_info", 10)
    session = [7]
    sequence = [100]
    estop = [False]

    def publish():
        link = ChassisLinkState()
        link.wire_session_id = session[0]
        links.publish(link)
        state = FirmwareControlState()
        sequence[0] += 1
        state.status_sequence, state.wire_session_id = sequence[0], session[0]
        state.estop_active = estop[0]
        states.publish(state)

    timer = fixture.create_timer(0.02, publish)
    client = ActionClient(fixture, ChassisOperation, "chassis/operation")
    for node in nodes:
        executor.add_node(node)
    spin_until(executor, lambda: client.server_is_ready() and ops.wire_session == 7)
    clear = ChassisOperation.Goal()
    clear.operation = clear.OP_CLEAR_FAULT
    sent = client.send_goal_async(clear)
    spin_until(executor, sent.done)
    result = sent.result().get_result_async()
    spin_for(executor, 0.05)
    assert not result.done(), "pending service must remain alive under the ROS executor"
    assert queue_releases
    queue_releases[0].set_result(True)
    spin_until(executor, result.done)
    assert result.result().result.status == ChassisOperation.Result.RESULT_QUEUED
    sent = client.send_goal_async(clear)
    spin_until(executor, sent.done)
    cancel_handle = sent.result()
    result = cancel_handle.get_result_async()
    spin_until(executor, lambda: len(queue_releases) == 2)
    cancel = cancel_handle.cancel_goal_async()
    spin_until(executor, lambda: cancel.done() and result.done())
    assert result.result().status == GoalStatus.STATUS_CANCELED
    queue_releases[1].set_result(True)
    goal = ChassisOperation.Goal()
    goal.operation, goal.enable = goal.OP_ESTOP_SET, True
    sent = client.send_goal_async(goal)
    spin_until(executor, sent.done)
    result = sent.result().get_result_async()
    spin_until(executor, lambda: bool(ops.operations))
    session[0], sequence[0], estop[0] = 8, 0, True
    spin_until(executor, result.done)
    assert result.result().result.status == ChassisOperation.Result.RESULT_REJECTED
    info_goal = ChassisOperation.Goal()
    info_goal.operation = info_goal.OP_GET_INFO
    sent = client.send_goal_async(info_goal)
    spin_until(executor, sent.done)
    result = sent.result().get_result_async()
    spin_until(executor, lambda: bool(ops.operations) and ops.operations[0].queued)
    info = FirmwareInfo()
    info.wire_session_id = 7
    infos.publish(info)
    spin_for(executor, 0.05)
    assert not result.done()
    info.wire_session_id = 8
    infos.publish(info)
    spin_until(executor, result.done)
    assert result.result().result.status == ChassisOperation.Result.RESULT_APPLIED
    fixture.destroy_timer(timer)
    client.destroy()


def estimation(executor, nodes):
    formal, supervisor, fixture = FormalOdometryNode(), MotionSupervisorNode(), Node("fixture")
    nodes.extend([formal, supervisor, fixture])
    compatibility_pub = fixture.create_publisher(
        PlatformCompatibilityState,
        "platform/compatibility_state",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    wheel_pub = fixture.create_publisher(WheelObservation, "wheel/observation", 10)
    odom_pub = fixture.create_publisher(Odometry, "odometry/filtered_internal", 10)
    gyro_pub = fixture.create_publisher(Imu, "imu/data", 10)
    observations = []
    fixture.create_subscription(Odometry, "odom", lambda msg: observations.append(msg), 10)
    keep_compatibility = [True]

    def publish():
        stamp = fixture.get_clock().now().to_msg()
        if keep_compatibility[0]:
            compatibility = PlatformCompatibilityState()
            compatibility.permit_formal_odometry = compatibility.observation_ready = True
            compatibility.observation_session_id = compatibility.wire_session_id = 7
            compatibility.header.stamp = stamp
            compatibility_pub.publish(compatibility)
        wheel = WheelObservation()
        wheel.schema_version, wheel.transport_session_id = wheel.SCHEMA_VERSION, 7
        wheel.header.stamp = stamp
        wheel_pub.publish(wheel)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, "odom", "base_link"
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)

    timer = fixture.create_timer(0.02, publish)
    for node in nodes:
        executor.add_node(node)
    spin_until(executor, lambda: len(observations) >= 2)
    keep_compatibility[0] = False
    spin_for(executor, 0.35)
    assert not formal.permitted
    before = len(observations)
    spin_for(executor, 0.10)
    assert len(observations) == before
    invalid = Imu()
    invalid.header.stamp = fixture.get_clock().now().to_msg()
    invalid.angular_velocity_covariance[0] = -1.0
    gyro_pub.publish(invalid)
    spin_for(executor, 0.05)
    assert supervisor.last_gyro_z is None
    valid = Imu()
    valid.header.stamp = fixture.get_clock().now().to_msg()
    valid.angular_velocity.z = 1.2
    valid.angular_velocity_covariance[0] = 0.001
    gyro_pub.publish(valid)
    spin_until(executor, lambda: supervisor.last_gyro_z is not None)
    assert supervisor.last_gyro_z == 1.2
    fixture.destroy_timer(timer)


if __name__ == "__main__":
    rclpy.init(args=["--ros-args", "-r", "__ns:=/adapter_test_" + uuid.uuid4().hex[:8]])
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        globals()[sys.argv[1]](executor, nodes)
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()
