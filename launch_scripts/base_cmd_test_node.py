#!/usr/bin/env python3

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quaternion(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class PhaseStats:
    label: str
    cmd_vx: float
    cmd_wz: float
    duration: float
    sample_count: int = 0
    start_x: Optional[float] = None
    start_y: Optional[float] = None
    start_yaw: Optional[float] = None
    end_x: Optional[float] = None
    end_y: Optional[float] = None
    end_yaw: Optional[float] = None
    peak_abs_vx: float = 0.0
    peak_abs_wz: float = 0.0
    accumulated_yaw: float = 0.0
    end_vx: float = 0.0
    end_wz: float = 0.0

    def observe(self, x: float, y: float, yaw: float, vx: float, wz: float) -> None:
        if not all(math.isfinite(value) for value in (x, y, yaw, vx, wz)):
            return
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.start_yaw = yaw
        if self.end_yaw is not None:
            self.accumulated_yaw += wrap_angle(yaw - self.end_yaw)
        self.end_x = x
        self.end_y = y
        self.end_yaw = yaw
        self.sample_count += 1
        self.peak_abs_vx = max(self.peak_abs_vx, abs(vx))
        self.peak_abs_wz = max(self.peak_abs_wz, abs(wz))
        self.end_vx, self.end_wz = vx, wz

    def execution_status(self) -> str:
        if self.sample_count == 0:
            return "NO_ODOM"
        if self.sample_count < 2:
            return "INSUFFICIENT_ODOM"
        if abs(self.cmd_vx) > 1e-6:
            forward = (self.end_x - self.start_x) * math.cos(self.start_yaw) + (
                self.end_y - self.start_y
            ) * math.sin(self.start_yaw)
            if math.copysign(1.0, self.cmd_vx) * forward < 0.005:
                return "NO_MOTION"
        if abs(self.cmd_wz) > 1e-6 and math.copysign(
            1.0, self.cmd_wz
        ) * self.accumulated_yaw < math.radians(1.0):
            return "NO_MOTION"
        if abs(self.cmd_vx) <= 1e-6 and abs(self.cmd_wz) <= 1e-6:
            return (
                "STOPPED"
                if abs(self.end_vx) <= 0.02 and abs(self.end_wz) <= 0.05
                else "NOT_STOPPED"
            )
        return "EXECUTED"

    def summary_line(self) -> str:
        if self.sample_count == 0 or self.start_x is None or self.end_x is None:
            return (
                f"SUMMARY {self.label}: samples=0 cmd_vx={self.cmd_vx:.3f} "
                f"cmd_wz={self.cmd_wz:.3f} status=NO_ODOM"
            )
        dx = self.end_x - self.start_x
        dy = self.end_y - self.start_y
        dist = math.hypot(dx, dy)
        yaw_delta_deg = math.degrees(self.accumulated_yaw)
        return (
            f"SUMMARY {self.label}: samples={self.sample_count} "
            f"cmd_vx={self.cmd_vx:.3f} cmd_wz={self.cmd_wz:.3f} "
            f"dx={dx:.3f} dy={dy:.3f} dist={dist:.3f} "
            f"yaw_delta_deg={yaw_delta_deg:.1f} "
            f"peak_vx={self.peak_abs_vx:.3f} peak_wz_deg_s={math.degrees(self.peak_abs_wz):.1f} "
            f"status={self.execution_status()}"
        )

    def yaw_delta_deg(self) -> Optional[float]:
        if self.sample_count == 0 or self.start_yaw is None or self.end_yaw is None:
            return None
        return math.degrees(self.accumulated_yaw)

    def effective_wz_deg_s(self) -> Optional[float]:
        yaw_delta = self.yaw_delta_deg()
        if yaw_delta is None or self.duration <= 0.0:
            return None
        return yaw_delta / self.duration


def publish_for(node, publisher, vx, wz, duration, hz, label):
    period = 1.0 / max(hz, 1.0)
    deadline = time.monotonic() + max(duration, 0.0)
    msg = Twist()
    msg.linear.x = float(vx)
    msg.angular.z = float(wz)
    print(f"TEST {label}: vx={vx:.3f} wz={wz:.3f} duration={duration:.2f}s", flush=True)
    while rclpy.ok() and time.monotonic() < deadline:
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)


def publish_stop(node, publisher, hz, duration=0.8):
    publish_for(node, publisher, 0.0, 0.0, duration, hz, "stop")


def wait_for_subscriber(node, publisher, timeout):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        if publisher.get_subscription_count() > 0:
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return publisher.get_subscription_count() > 0


@dataclass
class OdomStartupGate:
    max_age: float = 0.5
    sample_count: int = 0
    last_stamp_ns: int = -1
    last_received_at: Optional[float] = None

    def observe(self, stamp_ns, values, ros_now, now):
        age = ros_now - stamp_ns * 1e-9
        if (
            stamp_ns <= self.last_stamp_ns
            or not all(math.isfinite(value) for value in (*values, age))
            or not -0.02 <= age <= self.max_age
        ):
            return False
        if self.last_received_at is not None and now - self.last_received_at > self.max_age:
            self.sample_count = 0
        self.last_stamp_ns = stamp_ns
        self.last_received_at = now
        self.sample_count += 1
        return True

    def ready(self, now):
        return (
            self.sample_count >= 2
            and self.last_received_at is not None
            and 0 <= now - self.last_received_at <= self.max_age
        )


def odom_freshness_limit():
    path = os.environ.get("ROBOT_EFFECTIVE_PARAMS")
    if not path:
        return 0.5
    import yaml

    with open(path, encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream) or {}
    value = parameters.get("stm32_bridge", {}).get("ros__parameters", {}).get("status_timeout")
    if value is None or not math.isfinite(float(value)) or float(value) <= 0:
        raise ValueError("compiled stm32_bridge.status_timeout is required and must be positive")
    return float(value)


def wait_for_odometry(node, gate, timeout):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        if gate.ready(time.monotonic()):
            return True
        rclpy.spin_once(node, timeout_sec=min(0.05, max(deadline - time.monotonic(), 0.0)))
    return rclpy.ok() and gate.ready(time.monotonic())


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Publish deterministic base /cmd_vel/test commands."
    )
    parser.add_argument("--mode", choices=("rotate", "linear", "angular_sweep"), default="rotate")
    parser.add_argument("--linear", type=float, default=0.08)
    parser.add_argument("--angular", type=float, default=0.35)
    parser.add_argument("--angular-series", type=str, default="0.35,0.7,1.0,1.5,2.0,3.0")
    parser.add_argument("--duration", type=float, default=1.5)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--wait-subscriber-sec", type=float, default=8.0)
    parser.add_argument("--odom-settle-sec", type=float, default=0.5)
    parser.add_argument("--odom-topic", default="/odom")
    args = parser.parse_args(argv)
    if (
        not all(
            math.isfinite(value)
            for value in (
                args.linear,
                args.angular,
                args.duration,
                args.hz,
                args.wait_subscriber_sec,
                args.odom_settle_sec,
            )
        )
        or args.duration <= 0.0
        or args.hz <= 0.0
        or args.wait_subscriber_sec <= 0.0
        or args.odom_settle_sec < 0.0
    ):
        parser.error(
            "commands/timing must be finite; duration/hz/wait positive and settle nonnegative"
        )
    try:
        odom_gate = OdomStartupGate(max_age=odom_freshness_limit())
    except (OSError, ValueError) as exc:
        parser.error(str(exc))

    rclpy.init()
    node = rclpy.create_node("base_cmd_test_publisher")
    publisher = node.create_publisher(Twist, "/cmd_vel/test", 10)
    current_phase: Optional[PhaseStats] = None
    phases: List[PhaseStats] = []

    def on_odom(msg: Odometry) -> None:
        nonlocal current_phase
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        pose = msg.pose.pose
        twist = msg.twist.twist
        values = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
            float(twist.linear.x),
            float(twist.angular.z),
        )
        if not odom_gate.observe(
            stamp_ns, values, node.get_clock().now().nanoseconds * 1e-9, time.monotonic()
        ):
            return
        if current_phase is None:
            return
        current_phase.observe(
            values[0], values[1], yaw_from_quaternion(values[2], values[3]), values[4], values[5]
        )

    node.create_subscription(Odometry, args.odom_topic, on_odom, 20)

    def run_phase(label: str, vx: float, wz: float, duration: float) -> None:
        nonlocal current_phase
        current_phase = PhaseStats(
            label=label, cmd_vx=float(vx), cmd_wz=float(wz), duration=float(duration)
        )
        publish_for(node, publisher, vx, wz, duration, args.hz, label)
        settle_deadline = time.monotonic() + max(args.odom_settle_sec, 0.0)
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        phases.append(current_phase)
        print(current_phase.summary_line(), flush=True)
        current_phase = None

    def print_calibration() -> None:
        for phase in phases:
            if phase.sample_count == 0 or abs(phase.cmd_wz) < 1e-6:
                continue
            effective = phase.effective_wz_deg_s()
            if effective is None:
                continue
            cmd_wz_deg_s = math.degrees(phase.cmd_wz)
            ratio = effective / cmd_wz_deg_s if abs(cmd_wz_deg_s) > 1e-6 else 0.0
            print(
                f"CALIBRATION {phase.label}: "
                f"cmd_wz_deg_s={cmd_wz_deg_s:.1f} effective_wz_deg_s={effective:.1f} "
                f"response_ratio={ratio:.3f}",
                flush=True,
            )

    if not wait_for_subscriber(node, publisher, args.wait_subscriber_sec):
        print("ERROR: no /cmd_vel/test subscriber found", file=sys.stderr, flush=True)
        publish_stop(node, publisher, args.hz, 0.5)
        node.destroy_node()
        rclpy.shutdown()
        return 2

    try:
        # DDS discovery for the command publisher does not qualify this odom reader.
        if not wait_for_odometry(node, odom_gate, args.wait_subscriber_sec):
            print(
                f"ERROR: no fresh advancing {args.odom_topic} received by this test node",
                file=sys.stderr,
                flush=True,
            )
            return 4
        run_phase("stop_before", 0.0, 0.0, 0.8)
        if args.mode == "angular_sweep":
            angular_values = []
            for part in args.angular_series.split(","):
                part = part.strip()
                if not part:
                    continue
                value = float(part)
                if not math.isfinite(value) or abs(value) <= 1e-6:
                    print(
                        "ERROR: angular-series values must be finite and nonzero",
                        file=sys.stderr,
                        flush=True,
                    )
                    return 3
                angular_values.append(value)
            if not angular_values:
                print(
                    "ERROR: angular_sweep requires at least one angular-series value",
                    file=sys.stderr,
                    flush=True,
                )
                return 3
            for value in angular_values:
                run_phase(f"rotate_left_{value:.2f}", 0.0, value, args.duration)
                run_phase(f"stop_after_left_{value:.2f}", 0.0, 0.0, 0.8)
                run_phase(f"rotate_right_{value:.2f}", 0.0, -value, args.duration)
                run_phase(f"stop_after_right_{value:.2f}", 0.0, 0.0, 0.8)
            print_calibration()
        else:
            run_phase("rotate_left", 0.0, args.angular, args.duration)
            run_phase("stop_mid", 0.0, 0.0, 0.8)
            run_phase("rotate_right", 0.0, -args.angular, args.duration)
            run_phase("stop_after_rotate", 0.0, 0.0, 0.8)
        if args.mode == "linear":
            run_phase("forward", args.linear, 0.0, args.duration)
            run_phase("stop_mid_linear", 0.0, 0.0, 0.8)
            run_phase("backward", -args.linear, 0.0, args.duration)
            run_phase("stop_end", 0.0, 0.0, 0.8)
    finally:
        publish_stop(node, publisher, args.hz, 0.8)
        node.destroy_node()
        rclpy.shutdown()

    failed = [
        phase.label for phase in phases if phase.execution_status() not in ("EXECUTED", "STOPPED")
    ]
    if failed or not phases:
        print(
            f"ERROR: execution evidence incomplete: {', '.join(failed) or 'no phases'}",
            file=sys.stderr,
            flush=True,
        )
        return 4
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
