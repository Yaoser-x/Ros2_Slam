#!/usr/bin/env python3
"""Fixed real-ROS motion stages; physical yaw calibration is deliberately excluded."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time

# Reuse the established base-only readiness and phase evidence helpers.
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "launch_scripts"))
from base_cmd_test_node import PhaseStats
from base_test_readiness import BaseReadiness, freshness_limit, runtime_topics
from suspended_wheel_evidence import (
    PHASES,
    direction_observed,
    opposite_feedback,
    three_sample_stop,
    wheel_guard,
)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)
    import yaml
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from robot_interfaces.msg import (
        ChassisLinkState,
        FirmwareControlState,
        FirmwareInfo,
        HostControlState,
        HostMotionCommand,
        ImuObservation,
        MotionSupervisionState,
        PlatformCompatibilityState,
        WheelObservation,
    )
    from rosidl_runtime_py.convert import message_to_ordereddict
    from std_srvs.srv import Trigger

    params_path = Path(os.environ["ROBOT_EFFECTIVE_PARAMS"])
    params = yaml.safe_load(params_path.read_text())
    topics = runtime_topics(params)
    gate = BaseReadiness("real", freshness_limit(params))
    rclpy.init()
    node = rclpy.create_node("suspended_wheel_acceptance")
    publisher = node.create_publisher(Twist, "/cmd_vel/test", 10)
    info_client = node.create_client(Trigger, "/stm32_bridge/wire_get_info")
    info_requests = []
    static = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
    )
    args.report.parent.mkdir(parents=True, exist_ok=True)
    events_path = args.report.with_suffix(".messages.jsonl")
    events = events_path.open("w", encoding="utf-8")
    latest, counts, wheel_samples, phases, failures = {}, {}, [], [], []
    active, current, phase = False, None, "startup"
    requested = (0.0, 0.0)
    session = None

    def receive(kind, message):
        nonlocal session
        now = time.monotonic()
        value = message_to_ordereddict(message)
        latest[kind] = (now, value)
        counts[kind] = counts.get(kind, 0) + 1
        events.write(json.dumps({"t": now, "phase": phase, "kind": kind, "message": value}) + "\n")
        if kind in ("link", "compatibility", "supervision", "host", "odom"):
            gate.observe(kind, message, now)
        if kind == "wheel":
            wheel_samples.append(value)
            if active:
                reason = wheel_guard(value)
                if reason:
                    raise RuntimeError(reason)
                if session is not None and value["transport_session_id"] != session:
                    raise RuntimeError("wire session changed during acceptance")
                if opposite_feedback(value, *requested):
                    raise RuntimeError("wheel feedback opposite to current motion request")
        if kind == "odom" and current is not None:
            import math

            pose, twist = message.pose.pose, message.twist.twist
            yaw = math.atan2(
                2 * pose.orientation.w * pose.orientation.z, 1 - 2 * pose.orientation.z**2
            )
            current.observe(pose.position.x, pose.position.y, yaw, twist.linear.x, twist.angular.z)

    for kind, message_type, topic, qos in (
        ("link", ChassisLinkState, topics["link"], static),
        ("compatibility", PlatformCompatibilityState, topics["compatibility"], static),
        ("supervision", MotionSupervisionState, topics["supervision"], 10),
        ("host", HostControlState, topics["host"], 10),
        ("odom", Odometry, topics["odom"], 20),
        ("wheel", WheelObservation, "wheel/observation", 20),
        ("imu", ImuObservation, "imu/observation", qos_profile_sensor_data),
        ("firmware", FirmwareInfo, "chassis/firmware_info", static),
        ("firmware_control", FirmwareControlState, "chassis/firmware_control_state", 10),
        ("command", HostMotionCommand, "chassis/host_motion_command", qos_profile_sensor_data),
    ):
        node.create_subscription(message_type, topic, lambda msg, key=kind: receive(key, msg), qos)

    def spin_command(vx, wz, duration, require_ready=True):
        nonlocal requested
        requested = (vx, wz)
        until, next_send = time.monotonic() + duration, 0.0
        began = time.monotonic()
        message = Twist()
        message.linear.x, message.angular.z = vx, wz
        while time.monotonic() < until:
            now = time.monotonic()
            if (
                phase == "constant"
                and len(info_requests) < 3
                and now - began >= (1, 5, 9)[len(info_requests)]
            ):
                if not info_client.service_is_ready():
                    raise RuntimeError("GET_INFO service unavailable during constant motion")
                info_requests.append(info_client.call_async(Trigger.Request()))
                events.write(
                    json.dumps({"t": now, "phase": phase, "kind": "get_info_request"}) + "\n"
                )
            if now >= next_send:
                publisher.publish(message)
                events.write(
                    json.dumps(
                        {"t": now, "phase": phase, "kind": "requested_twist", "vx": vx, "wz": wz}
                    )
                    + "\n"
                )
                next_send = now + 0.05
            rclpy.spin_once(node, timeout_sec=0.01)
            if require_ready:
                reason = gate.reason(time.monotonic())
                if reason:
                    raise RuntimeError(reason)
                if "wheel" not in latest or time.monotonic() - latest["wheel"][0] > 0.25:
                    raise RuntimeError("raw wheel observation exceeded 250ms freshness limit")

    def stop(label):
        nonlocal phase, current
        phase, current = label, None
        wheel_samples.clear()
        began = time.monotonic()
        while time.monotonic() - began < 0.8:
            spin_command(0.0, 0.0, 0.04, require_ready=False)
            if three_sample_stop(wheel_samples):
                phases.append(
                    {"label": label, "result": "STOPPED", "seconds": time.monotonic() - began}
                )
                return
        raise RuntimeError(
            "stop not confirmed by target/output and three stationary encoder samples within 0.8s"
        )

    try:
        until = time.monotonic() + 20.0
        while time.monotonic() < until and (gate.reason(time.monotonic()) or "wheel" not in latest):
            spin_command(0.0, 0.0, 0.05, require_ready=False)
        reason = gate.reason(time.monotonic())
        if reason or "wheel" not in latest:
            raise RuntimeError(reason or "no raw wheel observations")
        session = latest["wheel"][1]["transport_session_id"]
        active, phase = True, "stationary_30s"
        spin_command(0.0, 0.0, 30.0)
        stop("stop_before")
        for label, vx, wz, duration in PHASES:
            phase = label
            current = PhaseStats(label, vx, wz, duration)
            wheel_samples.clear()
            print(f"MOTION {label}: vx={vx}m/s wz={wz}rad/s duration={duration}s", flush=True)
            spin_command(vx, wz, duration)
            if label == "constant" and (
                len(info_requests) != 3
                or not all(item.done() and item.result().success for item in info_requests)
            ):
                raise RuntimeError("three GET_INFO requests were not queued successfully")
            result = current.execution_status()
            if result != "EXECUTED" or not direction_observed(wheel_samples, vx, wz):
                raise RuntimeError(
                    f"{label}: odometry or raw wheel direction not proven ({result})"
                )
            phases.append(
                {
                    "label": label,
                    "vx": vx,
                    "wz": wz,
                    "duration": duration,
                    "result": result,
                    "summary": current.summary_line(),
                }
            )
            print(current.summary_line(), flush=True)
            stop("stop_after_" + label)
    except Exception as exc:
        failures.append(f"{type(exc).__name__}: {exc}")
    finally:
        active = False
        try:
            stop("final_stop")
        except Exception as exc:
            failures.append(f"final stop: {type(exc).__name__}: {exc}")
        node.destroy_node()
        rclpy.try_shutdown()
        events.close()
    report = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "environment": "suspended_wheels",
        "result": "PASS" if not failures and len(phases) >= len(PHASES) * 2 else "FAIL",
        "failures": failures,
        "phases": phases,
        "message_counts": counts,
        "wire_session_id": session,
        "get_info_requests_queued": sum(
            item.done() and item.result().success for item in info_requests
        ),
        "firmware": latest.get("firmware"),
        "config_manifest": yaml.safe_load((params_path.parent / "manifest.yaml").read_text()),
        "messages": str(events_path),
        "physical_yaw_calibration": False,
        "formal_release_compatible": False,
        "additional_acceptance": {
            key: "NOT_RUN"
            for key in (
                "UART replay and lifetime",
                "process faults",
                "PS2 and IMU macro",
                "ten minute stability",
                "power cycle",
            )
        },
    }
    args.report.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print("SUSPENDED_REPORT", args.report, report["result"], flush=True)
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
