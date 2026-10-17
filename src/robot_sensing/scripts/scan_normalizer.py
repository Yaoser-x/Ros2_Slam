#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanNormalizer(Node):
    def __init__(self) -> None:
        super().__init__('scan_normalizer')

        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('output_size', 425)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('status_log_interval_sec', 10.0)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.output_size = max(2, int(self.get_parameter('output_size').value))
        self.angle_min = float(self.get_parameter('angle_min').value)
        self.angle_max = float(self.get_parameter('angle_max').value)
        self.status_log_interval_sec = max(0.0, float(self.get_parameter('status_log_interval_sec').value))
        self.angle_increment = (self.angle_max - self.angle_min) / float(self.output_size - 1)
        self.last_log_time = self.get_clock().now()
        self.message_count = 0

        self.pub = self.create_publisher(LaserScan, self.output_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(LaserScan, self.input_topic, self.on_scan, qos_profile_sensor_data)
        self.get_logger().warn(
            f'scan_normalizer active: {self.input_topic} -> {self.output_topic}, bins={self.output_size}'
        )

    def maybe_log_status(self, input_size: int) -> None:
        if self.status_log_interval_sec <= 0.0:
            return
        now = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds * 1e-9
        if elapsed < self.status_log_interval_sec:
            return
        self.last_log_time = now
        self.get_logger().info(
            f'scan_normalizer_state count={self.message_count} input_size={input_size} '
            f'output_size={self.output_size}'
        )

    @staticmethod
    def valid_range(value: float, range_min: float, range_max: float) -> bool:
        return math.isfinite(value) and range_min <= value <= range_max

    def on_scan(self, msg: LaserScan) -> None:
        if not (
            math.isfinite(msg.scan_time)
            and msg.scan_time > 0.0
            and math.isfinite(msg.time_increment)
            and 0.0 <= msg.time_increment <= msg.scan_time
        ):
            self.get_logger().warn('Skipping scan: invalid scan_time or time_increment')
            return

        out = LaserScan()
        out.header = msg.header
        out.angle_min = self.angle_min
        out.angle_max = self.angle_max
        out.angle_increment = self.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = [math.inf] * self.output_size
        out.intensities = [0.0] * self.output_size

        source_increment = msg.angle_increment
        if abs(source_increment) <= 1e-9:
            source_increment = (msg.angle_max - msg.angle_min) / max(float(len(msg.ranges) - 1), 1.0)

        for source_index, source_range in enumerate(msg.ranges):
            if not self.valid_range(source_range, msg.range_min, msg.range_max):
                continue
            source_angle = msg.angle_min + float(source_index) * source_increment
            target_index = int(round((source_angle - self.angle_min) / self.angle_increment))
            if target_index < 0 or target_index >= self.output_size:
                continue
            if source_range < out.ranges[target_index]:
                out.ranges[target_index] = float(source_range)
                if source_index < len(msg.intensities):
                    out.intensities[target_index] = float(msg.intensities[source_index])

        self.message_count += 1
        self.pub.publish(out)
        self.maybe_log_status(len(msg.ranges))


def main() -> int:
    rclpy.init()
    node: Optional[ScanNormalizer] = None
    try:
        node = ScanNormalizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
