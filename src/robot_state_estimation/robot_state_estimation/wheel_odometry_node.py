#!/usr/bin/env python3
"""Convert raw wheel observations into formal wheel odometry."""

import math
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from builtin_interfaces.msg import Time as TimeMessage
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import PlatformCompatibilityState, WheelObservation

from robot_state_estimation.time_mapper import (
    McuClockMapper,
    SampleDisposition,
    SampleOrderTracker,
)
from robot_state_estimation.wheel_odometry import EncoderOdometry, covariance_multiplier
from robot_chassis_model.wheel_layout import WheelLayout, WheelLayoutError


class WheelOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("wheel_odometry")
        defaults = {
            "config_sha256": "development-uncompiled",
            "observation_topic": "wheel/observation",
            "odom_topic": "wheel/odom",
            "frame_id": "odom",
            "child_frame_id": "base_link",
            "wheel_radius": 0.035,
            "wheel_track_width": 0.176,
            "encoder_counts_per_revolution": 2464.0,
            "odom_linear_scale": 1.0,
            "odom_angular_scale": 1.0,
            "odom_angular_sign": 1.0,
            "max_dt_sec": 0.25,
            "hard_max_wheel_peripheral_speed_mps": 0.70,
            "compatibility_state_topic": "platform/compatibility_state",
            "left_variance_floor_m2": 0.000025,
            "left_variance_per_meter": 0.0025,
            "right_variance_floor_m2": 0.000025,
            "right_variance_per_meter": 0.0025,
            "left_right_correlation": 0.0,
            "turn_noise_gain": 0.5,
            "slip_noise_gain": 1.0,
            "degraded_single_wheel_multiplier": 4.0,
            "unobserved_pose_variance": 1000000.0,
            "unobserved_twist_variance": 1000000.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.hard_max_wheel_speed = float(
            self.get_parameter("hard_max_wheel_peripheral_speed_mps").value
        )
        self.odometry = EncoderOdometry(
            wheel_radius_m=float(self.get_parameter("wheel_radius").value),
            track_width_m=float(self.get_parameter("wheel_track_width").value),
            counts_per_revolution=float(self.get_parameter("encoder_counts_per_revolution").value),
            max_dt_sec=float(self.get_parameter("max_dt_sec").value),
            linear_scale=float(self.get_parameter("odom_linear_scale").value),
            angular_scale=float(self.get_parameter("odom_angular_scale").value),
            angular_sign=float(self.get_parameter("odom_angular_sign").value),
            left_variance_floor_m2=float(self.get_parameter("left_variance_floor_m2").value),
            left_variance_per_meter=float(self.get_parameter("left_variance_per_meter").value),
            right_variance_floor_m2=float(self.get_parameter("right_variance_floor_m2").value),
            right_variance_per_meter=float(self.get_parameter("right_variance_per_meter").value),
            left_right_correlation=float(self.get_parameter("left_right_correlation").value),
            turn_noise_gain=float(self.get_parameter("turn_noise_gain").value),
            slip_noise_gain=float(self.get_parameter("slip_noise_gain").value),
            degraded_single_wheel_multiplier=float(
                self.get_parameter("degraded_single_wheel_multiplier").value
            ),
        )
        self.order = SampleOrderTracker()
        self.unobserved_pose_variance = float(self.get_parameter("unobserved_pose_variance").value)
        self.unobserved_twist_variance = float(
            self.get_parameter("unobserved_twist_variance").value
        )
        self.clock_mapper = McuClockMapper()
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), qos
        )
        self.subscription = self.create_subscription(
            WheelObservation,
            str(self.get_parameter("observation_topic").value),
            self._on_observation,
            qos,
        )
        self.duplicate_count = 0
        self.out_of_order_count = 0
        self.rejected_count = 0
        self.compatibility_permits_odometry = False
        self.create_subscription(
            PlatformCompatibilityState,
            str(self.get_parameter("compatibility_state_topic").value),
            self._on_compatibility,
            10,
        )

    def _on_compatibility(self, msg: PlatformCompatibilityState) -> None:
        permitted = bool(msg.permit_formal_odometry)
        if permitted != self.compatibility_permits_odometry:
            self.odometry.reset_sample_baseline()
        self.compatibility_permits_odometry = permitted

    def _on_observation(self, msg: WheelObservation) -> None:
        if not self.compatibility_permits_odometry:
            self.odometry.reset_sample_baseline()
            self.rejected_count += 1
            return
        if int(msg.schema_version) != WheelObservation.SCHEMA_VERSION:
            self.rejected_count += 1
            return
        disposition = self.order.update(msg.transport_session_id, msg.status_sequence)
        if disposition is SampleDisposition.DUPLICATE:
            self.duplicate_count += 1
            return
        if disposition is SampleDisposition.OUT_OF_ORDER:
            self.out_of_order_count += 1
            return
        if disposition is SampleDisposition.FIRST:
            self.clock_mapper.reset()
            self.odometry.reset_sample_baseline()
        receive_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        timing = self.clock_mapper.update(msg.mcu_sample_time_ms, receive_sec)
        if timing.reset:
            self.odometry.reset_sample_baseline()
        layout = WheelLayout(msg.motor_enabled_mask, msg.speed_valid_mask, msg.encoder_anomaly_mask)
        try:
            left_speed, right_speed = layout.aggregate(msg.wheel_speed_mps)
        except WheelLayoutError:
            # Upper-v3 speed quality does not qualify accumulated counts.
            wheel_wz = 0.0
        else:
            wheel_wz = (right_speed - left_speed) / max(
                float(self.get_parameter("wheel_track_width").value), 1e-6
            )
        multiplier = covariance_multiplier(
            wheel_speeds=msg.wheel_speed_mps,
            enabled_mask=msg.motor_enabled_mask,
            speed_valid_mask=msg.speed_valid_mask,
            anomaly_mask=msg.encoder_anomaly_mask,
            sample_age_sec=max(receive_sec - timing.sample_ros_sec, 0.0),
            turn_rate=wheel_wz,
            quality_flags=(
                int(msg.side_consistency_flags) | int(msg.comm_health_flags) | int(msg.status_flags)
            ),
        )
        update = self.odometry.update(
            tuple(msg.encoder_count),
            sample_time_sec=timing.sensor_time_sec,
            enabled_mask=msg.motor_enabled_mask,
            speed_valid_mask=msg.speed_valid_mask,
            anomaly_mask=msg.encoder_anomaly_mask,
            transport_session_id=msg.transport_session_id,
            reset_generation=msg.reset_generation,
            covariance_multiplier=multiplier,
            hard_max_wheel_peripheral_speed_mps=self.hard_max_wheel_speed,
        )
        if not update.integrated or not update.trusted:
            return
        odom = Odometry()
        odom.header.stamp = self._time_message(timing.sample_ros_sec)
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = update.pose.x
        odom.pose.pose.position.y = update.pose.y
        odom.pose.pose.orientation.z = math.sin(update.pose.yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(update.pose.yaw * 0.5)
        odom.twist.twist.linear.x = update.vx
        odom.twist.twist.angular.z = update.wz
        p = update.pose_covariance
        odom.pose.covariance[0], odom.pose.covariance[1], odom.pose.covariance[5] = (
            p[0],
            p[1],
            p[2],
        )
        odom.pose.covariance[6], odom.pose.covariance[7], odom.pose.covariance[11] = (
            p[3],
            p[4],
            p[5],
        )
        odom.pose.covariance[30], odom.pose.covariance[31], odom.pose.covariance[35] = (
            p[6],
            p[7],
            p[8],
        )
        for index in (14, 21, 28):
            odom.pose.covariance[index] = self.unobserved_pose_variance
        t = update.twist_covariance
        odom.twist.covariance[0] = max(t[0], 1e-6)
        odom.twist.covariance[5] = t[1]
        odom.twist.covariance[30] = t[2]
        odom.twist.covariance[35] = max(t[3], 1e-6)
        for index in (7, 14, 21, 28):
            odom.twist.covariance[index] = self.unobserved_twist_variance
        self.publisher.publish(odom)

    @staticmethod
    def _time_message(seconds: float) -> TimeMessage:
        sec = math.floor(seconds)
        nanosec = int(round((seconds - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec, nanosec = sec + 1, nanosec - 1_000_000_000
        return TimeMessage(sec=int(sec), nanosec=nanosec)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = WheelOdometryNode()
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
