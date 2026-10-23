#!/usr/bin/env python3
"""Map native Go2 sensor timestamps to NX wall time for local DimOS consumers."""

import copy
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from unitree_go.msg import SportModeState

from robot_clock_sync import (
    ClockCalibrator,
    SourceStampGuard,
    map_stamp,
    read_snapshot,
    validate_positive_finite,
)


_DEFAULT_SNAPSHOT = "/tmp/dimos_nx_local_adapter_clock.json"


class NxLocalSensorAdapter(Node):
    """Read native sensor topics and publish timestamp-corrected copies only."""

    def __init__(self) -> None:
        super().__init__("nx_local_sensor_adapter")
        self.declare_parameter("sport_state_topic", "/sportmodestate")
        self.declare_parameter("cloud_topic", "/utlidar/cloud_deskewed")
        self.declare_parameter("pose_topic", "/utlidar/robot_pose")
        self.declare_parameter("output_cloud_topic", "/dimos/registered_scan")
        self.declare_parameter("output_pose_topic", "/dimos/odom")
        self.declare_parameter("clock_snapshot_path", _DEFAULT_SNAPSHOT)
        self.declare_parameter("max_source_age_sec", 0.5)

        self._snapshot_path = str(self.get_parameter("clock_snapshot_path").value)
        if not self._snapshot_path:
            raise ValueError("clock_snapshot_path must not be empty")
        self._max_source_age_sec = validate_positive_finite(
            self.get_parameter("max_source_age_sec").value, "max_source_age_sec"
        )

        # This private snapshot belongs only to this local adapter process.
        self._calibrator = ClockCalibrator(self._snapshot_path)
        self._cloud_guard = SourceStampGuard()
        self._pose_guard = SourceStampGuard()
        self._last_drop_log = 0.0

        cloud_input_qos = QoSProfile(depth=5)
        cloud_input_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        cloud_input_qos.durability = DurabilityPolicy.VOLATILE
        cloud_output_qos = QoSProfile(depth=5)
        cloud_output_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        cloud_output_qos.durability = DurabilityPolicy.VOLATILE
        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(
            SportModeState,
            str(self.get_parameter("sport_state_topic").value),
            self._on_sport_state,
            10,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("cloud_topic").value),
            self._on_cloud,
            cloud_input_qos,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self._on_pose,
            pose_qos,
        )
        self._cloud_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("output_cloud_topic").value),
            cloud_output_qos,
        )
        self._pose_publisher = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("output_pose_topic").value),
            pose_qos,
        )

    def _on_sport_state(self, message: SportModeState) -> None:
        try:
            self._calibrator.observe(int(message.stamp.sec), int(message.stamp.nanosec))
        except (AttributeError, TypeError, ValueError) as error:
            self._warn_drop(f"ignored invalid SportModeState clock sample: {error}")

    def _mapped_stamp(self, message, guard: SourceStampGuard) -> Time:
        sec = int(message.header.stamp.sec)
        nanosec = int(message.header.stamp.nanosec)
        snapshot = read_snapshot(self._snapshot_path)
        mapped_sec, mapped_nanosec = map_stamp(
            sec,
            nanosec,
            snapshot,
            max_age_sec=self._max_source_age_sec,
        )
        guard.accept(
            sec,
            nanosec,
            snapshot["session"],
            mapped_sec,
            mapped_nanosec,
        )
        return Time(sec=mapped_sec, nanosec=mapped_nanosec)

    def _on_cloud(self, message: PointCloud2) -> None:
        try:
            stamp = self._mapped_stamp(message, self._cloud_guard)
        except (AttributeError, KeyError, OSError, TypeError, ValueError) as error:
            self._warn_drop(f"dropped cloud with unmappable/reordered stamp: {error}")
            return
        output = copy.deepcopy(message)
        output.header.stamp = stamp
        self._cloud_publisher.publish(output)

    def _on_pose(self, message: PoseStamped) -> None:
        try:
            stamp = self._mapped_stamp(message, self._pose_guard)
        except (AttributeError, KeyError, OSError, TypeError, ValueError) as error:
            self._warn_drop(f"dropped pose with unmappable/reordered stamp: {error}")
            return
        output = copy.deepcopy(message)
        output.header.stamp = stamp
        self._pose_publisher.publish(output)

    def _warn_drop(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_drop_log >= 1.0:
            self._last_drop_log = now
            self.get_logger().warning(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NxLocalSensorAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
