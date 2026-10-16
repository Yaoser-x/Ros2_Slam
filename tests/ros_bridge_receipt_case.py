"""Real DDS receipt while the bridge state callback group is deliberately busy."""

import threading
import time
import uuid

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from robot_interfaces.msg import HostMotionCommand
from stm32_robot_bridge.bridge_node_v3 import STM32BridgeV3


def main():
    namespace = "/receipt_" + uuid.uuid4().hex
    rclpy.init(args=["--ros-args", "-r", "__ns:=" + namespace,
                     "-p", "port:=/dev/nonexistent_receipt_probe_" + uuid.uuid4().hex])
    bridge = STM32BridgeV3()
    fixture = rclpy.create_node("receipt_fixture", namespace=namespace)
    publisher = fixture.create_publisher(
        HostMotionCommand, "chassis/host_motion_command",
        QoSProfile(depth=1, deadline=Duration(seconds=0.05)),
    )
    entered, release = threading.Event(), threading.Event()

    def busy():
        timer.cancel()
        entered.set()
        assert release.wait(3), "test did not release the default callback group"

    executor = MultiThreadedExecutor(num_threads=2, context=bridge.context)
    executor.add_node(bridge)
    executor.add_node(fixture)
    worker = threading.Thread(target=executor.spin)
    worker.start()
    try:
        until = time.monotonic() + 10
        while not publisher.get_subscription_count() and time.monotonic() < until:
            time.sleep(0.01)
        assert publisher.get_subscription_count()
        # A test timer occupies the same mutually exclusive group as control,
        # services and telemetry; the receipt callback must still make progress.
        timer = bridge.create_timer(0.01, busy)
        assert entered.wait(3)
        sent_at = time.monotonic()
        message = HostMotionCommand()
        message.header.stamp = fixture.get_clock().now().to_msg()
        message.command_epoch, message.sequence = 7, 1
        publisher.publish(message)
        received = None
        until = time.monotonic() + 0.5
        while received is None and time.monotonic() < until:
            overflow, received = bridge.command_inbox.take()
            assert not overflow
            time.sleep(0.001)
        assert received is not None, "DDS receipt was starved by the state callback group"
        assert not release.is_set()
        assert sent_at <= received.received_monotonic <= time.monotonic()
        assert received.session_id == 7 and received.sequence == 1
        assert bridge.core.snapshot.command_session_id == 0
        print("PASS: receipt captured during blocked state callback; no concurrent core mutation")
    finally:
        release.set()
        bridge.command_inbox.stop()
        executor.shutdown()
        worker.join(timeout=3)
        assert not worker.is_alive()
        bridge.shutdown()
        fixture.destroy_node()
        bridge.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
