"""Exercise real process signals with incoming DDS messages and no hardware port."""

import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import uuid

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from robot_interfaces.msg import FirmwareControlState, HostMotionCommand, ImuObservation, WheelObservation


def main():
    kind, signal_name, source = sys.argv[1:]
    namespace = "/signal_probe_" + uuid.uuid4().hex
    topic, message_type = {
        "mux": ("cmd_vel/test", Twist),
        "supervisor": ("wheel/observation", WheelObservation),
        "bridge": ("chassis/host_motion_command", HostMotionCommand),
        "compatibility": ("wheel/observation", WheelObservation),
        "operations": ("chassis/firmware_control_state", FirmwareControlState),
        "wheel": ("wheel/observation", WheelObservation),
        "imu": ("imu/observation", ImuObservation),
        "odometry": ("wheel/observation", WheelObservation),
    }[kind]
    rclpy.init()
    fixture = rclpy.create_node("signal_fixture", namespace=namespace)
    executor = SingleThreadedExecutor()
    executor.add_node(fixture)
    qos = QoSProfile(depth=1, deadline=Duration(seconds=0.05)) if kind == "bridge" else 10
    publisher = fixture.create_publisher(message_type, topic, qos)
    arguments = [sys.executable, source, "--ros-args", "-r", "__ns:=" + namespace]
    if kind == "bridge":
        port = "/dev/nonexistent_shutdown_probe_" + uuid.uuid4().hex
        assert not Path(port).exists()
        arguments += ["-p", "port:=" + port]
    sequence = 0

    def publish():
        nonlocal sequence
        sequence += 1
        message = message_type()
        if kind != "mux":
            message.header.stamp = fixture.get_clock().now().to_msg()
        if isinstance(message, WheelObservation):
            message.schema_version = WheelObservation.SCHEMA_VERSION
            message.transport_session_id = 7
            message.status_sequence = sequence
            message.motor_enabled_mask = message.speed_valid_mask = 6
        if kind == "imu":
            message.schema_version = ImuObservation.SCHEMA_VERSION
            message.transport_session_id = 7
            message.sample_sequence = sequence
            message.mcu_sample_time_ms = sequence * 10
        if kind == "operations":
            message.wire_session_id = 7
            message.status_sequence = sequence
        if kind == "bridge":
            message.command_epoch, message.sequence = 7, sequence
            message.enable = False
            message.host_subsource = HostMotionCommand.HOST_SUBSOURCE_TEST
        publisher.publish(message)
        executor.spin_once(timeout_sec=0.005)

    try:
        with tempfile.TemporaryFile(mode="w+") as log:
            process = subprocess.Popen(arguments, stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 18.0
                while not publisher.get_subscription_count() and time.monotonic() < deadline:
                    assert process.poll() is None, "node exited before subscriber discovery"
                    publish()
                assert publisher.get_subscription_count(), "no live DDS subscriber"
                until = time.monotonic() + 0.8
                while time.monotonic() < until:
                    publish()
                process.send_signal(getattr(signal, signal_name))
                process.wait(timeout=8.0)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=5.0)
            log.seek(0)
            output = log.read()
        assert process.returncode == 0, output
        assert "Traceback" not in output and "Unable to convert call argument" not in output, output
        if kind == "bridge":
            assert "SERIAL_SHUTDOWN" in output and "disable_written=False" in output, output
        print(json.dumps({"case": kind, "signal": signal_name, "exit_code": process.returncode,
                          "result": "PASS", "hardware_access": False}))
    finally:
        executor.shutdown()
        fixture.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
