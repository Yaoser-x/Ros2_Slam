#!/usr/bin/env python3
"""Read-only business readiness gate for the complete base-only launch chain."""

import argparse
import math
import time


class BaseReadiness:
    def __init__(self, base_mode="real", max_age=0.5):
        self.base_mode = base_mode
        self.max_age = max_age
        self.messages = {}
        self.received = {}
        self.odom_stamp = -1
        self.odom_samples = 0

    def observe(self, kind, message, now):
        if kind == "link":
            old = self.messages.get("link")
            if old is not None and int(old.wire_session_id) != int(message.wire_session_id):
                self.messages.clear()
                self.received.clear()
                self.odom_stamp = -1
                self.odom_samples = 0
        if kind == "odom":
            stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(
                message.header.stamp.nanosec
            )
            values = (
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w,
                message.twist.twist.linear.x,
                message.twist.twist.angular.z,
            )
            if stamp <= self.odom_stamp or not all(math.isfinite(value) for value in values):
                return
            self.odom_stamp = stamp
            self.odom_samples += 1
        self.messages[kind] = message
        self.received[kind] = now

    def reason(self, now):
        for kind in ("link", "compatibility", "supervision", "host", "odom"):
            if kind not in self.messages or not 0 <= now - self.received[kind] <= self.max_age:
                return f"waiting for fresh {kind}"
        link = self.messages["link"]
        session = int(link.wire_session_id)
        if (
            session == 0
            or link.link_state not in (6, 7)
            or not link.protocol_compatible
            or link.wire_rearm_required
            or link.status_age_ms > self.max_age * 1000
        ):
            return "link is not wire ready"
        compatibility = self.messages["compatibility"]
        expected = 2 if self.base_mode == "fake" else 1
        if (
            compatibility.state != expected
            or not compatibility.observation_ready
            or int(compatibility.wire_session_id) != session
            or int(compatibility.observation_session_id) != session
            or (self.base_mode == "real" and not compatibility.permit_formal_odometry)
        ):
            return "current-session compatibility has not permitted this provider"
        supervision = self.messages["supervision"]
        if (
            supervision.level not in (1, 2, 3)
            or supervision.release_host_candidate
            or not math.isfinite(supervision.command_scale)
            or supervision.command_scale <= 0
            or int(supervision.observation_session_id) != session
            or supervision.observation_age_ms > self.max_age * 1000
        ):
            return "motion supervision is not ready"
        host = self.messages["host"]
        # WAIT_FRESH_HOST_INTENT accepts the next fresh command, which clears rearm.
        # A read-only readiness observer must not wait for that command to arrive.
        if host.control_state != 3 and not (host.control_state == 4 and not host.rearm_required):
            return "Host control is not ready for fresh intent"
        if self.odom_samples < 2:
            return "waiting for successive odometry samples"
        return ""


def runtime_topics(params):
    def value(node, key, default):
        return params.get(node, {}).get("ros__parameters", {}).get(key, default)

    return {
        "link": value("platform_compatibility", "chassis_link_state_topic", "/chassis/link_state"),
        "compatibility": value(
            "platform_compatibility", "compatibility_state_topic", "/platform/compatibility_state"
        ),
        "supervision": value(
            "motion_supervisor", "motion_supervision_topic", "/motion/supervision_state"
        ),
        "host": value("cmd_vel_mux", "host_control_state_topic", "/chassis/host_control_state"),
        "odom": "/odom",
    }


def freshness_limit(params):
    value = params.get("stm32_bridge", {}).get("ros__parameters", {}).get("status_timeout")
    if value is None or not math.isfinite(float(value)) or float(value) <= 0:
        raise ValueError("compiled stm32_bridge.status_timeout is required and must be positive")
    return float(value)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-mode", choices=("real", "fake"), default="real")
    parser.add_argument("--effective-params", required=True)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("timeout must be positive and finite")
    import yaml
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from robot_interfaces.msg import (
        ChassisLinkState,
        HostControlState,
        MotionSupervisionState,
        PlatformCompatibilityState,
    )

    with open(args.effective_params, encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream) or {}
        topics = runtime_topics(parameters)
        max_age = freshness_limit(parameters)
    rclpy.init()
    node = rclpy.create_node("base_test_readiness")
    gate = BaseReadiness(args.base_mode, max_age)
    state_qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
    )
    # Volatile subscribers also receive reliable state publishers; no command is published.
    for kind, message_type in (
        ("link", ChassisLinkState),
        ("compatibility", PlatformCompatibilityState),
        ("supervision", MotionSupervisionState),
        ("host", HostControlState),
        ("odom", Odometry),
    ):
        qos = state_qos if kind in ("link", "compatibility") else 10
        node.create_subscription(
            message_type,
            topics[kind],
            lambda msg, name=kind: gate.observe(name, msg, time.monotonic()),
            qos,
        )
    deadline = time.monotonic() + args.timeout
    reason = "no business data"
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            reason = gate.reason(time.monotonic())
            if not reason:
                print(
                    f"READY base_mode={args.base_mode} wire_session={gate.messages['link'].wire_session_id} "
                    f"odom_samples={gate.odom_samples}",
                    flush=True,
                )
                return 0
        print(f"NOT_READY {reason}", flush=True)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
