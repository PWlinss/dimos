#!/usr/bin/env python3
"""Bridge PointCloud2 between two local ROS DDS participants over localhost TCP."""

import argparse
import math
import queue
import socket
import struct
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2


_LENGTH = struct.Struct("!I")
_DEFAULT_PORT = 39102
_DEFAULT_MAX_MESSAGE_BYTES = 16 * 1024 * 1024
_DEFAULT_CONNECT_TIMEOUT_SEC = 0.2
_DEFAULT_RETRY_BACKOFF_INITIAL_SEC = 0.1
_DEFAULT_RETRY_BACKOFF_MAX_SEC = 2.0
_MIN_RETRY_BACKOFF_SEC = 0.01


def _validate_positive_finite(value, name: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be a finite positive number") from error
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise ValueError(f"{name} must be a finite positive number")
    return parsed


def _validate_positive_int(value, name: str) -> int:
    parsed = _validate_positive_finite(value, name)
    if not parsed.is_integer():
        raise ValueError(f"{name} must be a positive integer")
    return int(parsed)


def _validate_nonnegative_finite(value, name: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be a finite non-negative number") from error
    if not math.isfinite(parsed) or parsed < 0.0:
        raise ValueError(f"{name} must be a finite non-negative number")
    return parsed


def _retry_delay(value, name: str) -> float:
    return max(_validate_nonnegative_finite(value, name), _MIN_RETRY_BACKOFF_SEC)


def _reliability_policy(value: str) -> ReliabilityPolicy:
    normalized = str(value).strip().lower()
    if normalized == "reliable":
        return ReliabilityPolicy.RELIABLE
    if normalized == "best_effort":
        return ReliabilityPolicy.BEST_EFFORT
    raise ValueError("input_reliability must be 'reliable' or 'best_effort'")


def _recv_exact(connection: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = connection.recv(size - len(chunks))
        if not chunk:
            raise ConnectionError("TCP peer disconnected")
        chunks.extend(chunk)
    return bytes(chunks)


class PointCloudTcpSender(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_tcp_sender")
        self.declare_parameter("input_topic", "/utlidar/cloud_deskewed")
        self.declare_parameter("tcp_host", "127.0.0.1")
        self.declare_parameter("tcp_port", _DEFAULT_PORT)
        self.declare_parameter("max_rate_hz", 10.0)
        self.declare_parameter("connect_timeout_sec", _DEFAULT_CONNECT_TIMEOUT_SEC)
        self.declare_parameter(
            "retry_backoff_initial_sec", _DEFAULT_RETRY_BACKOFF_INITIAL_SEC
        )
        self.declare_parameter("retry_backoff_max_sec", _DEFAULT_RETRY_BACKOFF_MAX_SEC)
        self.declare_parameter("input_reliability", "reliable")
        self.declare_parameter("stats_interval_sec", 2.0)

        self._host = str(self.get_parameter("tcp_host").value)
        self._port = int(self.get_parameter("tcp_port").value)
        self._max_rate_hz = _validate_positive_finite(
            self.get_parameter("max_rate_hz").value, "max_rate_hz"
        )
        self._minimum_period = 1.0 / self._max_rate_hz
        self._connect_timeout_sec = _validate_positive_finite(
            self.get_parameter("connect_timeout_sec").value, "connect_timeout_sec"
        )
        self._retry_backoff_initial_sec = _retry_delay(
            self.get_parameter("retry_backoff_initial_sec").value,
            "retry_backoff_initial_sec",
        )
        self._retry_backoff_max_sec = max(
            _retry_delay(
                self.get_parameter("retry_backoff_max_sec").value,
                "retry_backoff_max_sec",
            ),
            self._retry_backoff_initial_sec,
        )
        self._retry_backoff_sec = self._retry_backoff_initial_sec
        self._next_connect_time = 0.0
        self._last_sent = 0.0
        self._connection = None
        self._received = 0
        self._sent = 0
        self._rate_drops = 0
        self._send_failures = 0

        qos = QoSProfile(depth=5)
        qos.reliability = _reliability_policy(
            self.get_parameter("input_reliability").value
        )
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("input_topic").value),
            self._on_cloud,
            qos,
        )
        self.create_timer(
            _validate_positive_finite(
                self.get_parameter("stats_interval_sec").value, "stats_interval_sec"
            ),
            self._log_stats,
        )

    def _connect(self) -> bool:
        if self._connection is not None:
            return True
        now = time.monotonic()
        if now < self._next_connect_time:
            return False

        connection = None
        try:
            connection = socket.create_connection(
                (self._host, self._port), timeout=self._connect_timeout_sec
            )
            connection.settimeout(0.5)
            self._connection = connection
            self._retry_backoff_sec = self._retry_backoff_initial_sec
            self._next_connect_time = 0.0
            return True
        except OSError:
            if connection is not None:
                try:
                    connection.close()
                except OSError:
                    pass
            self._connection = None
            self._schedule_retry(time.monotonic())
            return False

    def _schedule_retry(self, now: float) -> None:
        self._next_connect_time = now + self._retry_backoff_sec
        self._retry_backoff_sec = min(
            self._retry_backoff_sec * 2.0, self._retry_backoff_max_sec
        )

    def _disconnect(self) -> None:
        if self._connection is not None:
            try:
                self._connection.close()
            except OSError:
                pass
        self._connection = None

    def _on_cloud(self, message: PointCloud2) -> None:
        self._received += 1
        now = time.monotonic()
        if now - self._last_sent < self._minimum_period:
            self._rate_drops += 1
            return
        if not self._connect():
            self._send_failures += 1
            return

        payload = serialize_message(message)
        try:
            self._connection.sendall(_LENGTH.pack(len(payload)))
            self._connection.sendall(payload)
        except OSError:
            self._send_failures += 1
            self._disconnect()
            self._schedule_retry(time.monotonic())
            return

        self._last_sent = now
        self._sent += 1

    def _log_stats(self) -> None:
        self.get_logger().info(
            "pointcloud TCP sender stats: "
            f"received={self._received} sent={self._sent} "
            f"rate_drops={self._rate_drops} send_failures={self._send_failures}"
        )

    def destroy_node(self) -> bool:
        self._disconnect()
        return super().destroy_node()


class PointCloudTcpReceiver(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_tcp_receiver")
        self.declare_parameter("output_topic", "/dimos/registered_scan")
        self.declare_parameter("bind_host", "127.0.0.1")
        self.declare_parameter("tcp_port", _DEFAULT_PORT)
        self.declare_parameter("max_message_bytes", _DEFAULT_MAX_MESSAGE_BYTES)
        self.declare_parameter("stats_interval_sec", 2.0)

        self._host = str(self.get_parameter("bind_host").value)
        self._port = int(self.get_parameter("tcp_port").value)
        self._max_message_bytes = _validate_positive_int(
            self.get_parameter("max_message_bytes").value, "max_message_bytes"
        )
        self._latest = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._server = None
        self._server_ready = threading.Event()
        self._server_error = None
        self._server_failure_reported = False
        self._received = 0
        self._published = 0
        self._queue_drops = 0
        self._receive_failures = 0

        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self._publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter("output_topic").value), qos
        )
        self.create_timer(0.005, self._publish_latest)
        self.create_timer(
            _validate_positive_finite(
                self.get_parameter("stats_interval_sec").value, "stats_interval_sec"
            ),
            self._log_stats,
        )
        self.create_timer(0.1, self._check_server_error)
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _check_server_error(self) -> None:
        if self._server_error is None or self._server_failure_reported:
            return
        self._server_failure_reported = True
        self.get_logger().error(
            "pointcloud TCP receiver shutting down after socket failure: "
            f"{self._server_error}"
        )
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass

    def _serve(self) -> None:
        server = None
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._server = server
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self._host, self._port))
            server.listen(1)
            server.settimeout(0.2)
        except OSError as error:
            self._server_error = error
            self._stop.set()
            self._server_ready.set()
            self.get_logger().error(
                "pointcloud TCP receiver failed to bind/listen on "
                f"{self._host}:{self._port}: {error}"
            )
            if server is not None:
                try:
                    server.close()
                except OSError:
                    pass
            return

        self._server_ready.set()

        while not self._stop.is_set():
            try:
                connection, _ = server.accept()
            except socket.timeout:
                continue
            except OSError as error:
                if not self._stop.is_set():
                    self._server_error = error
                    self._stop.set()
                    self.get_logger().error(
                        f"pointcloud TCP receiver accept failed: {error}"
                    )
                break

            with connection:
                connection.settimeout(1.0)
                while not self._stop.is_set():
                    try:
                        size = _LENGTH.unpack(_recv_exact(connection, _LENGTH.size))[0]
                        if size <= 0 or size > self._max_message_bytes:
                            raise ValueError(f"invalid PointCloud2 payload size: {size}")
                        payload = _recv_exact(connection, size)
                    except (ConnectionError, OSError, ValueError):
                        self._receive_failures += 1
                        break

                    if self._latest.full():
                        try:
                            self._latest.get_nowait()
                            self._queue_drops += 1
                        except queue.Empty:
                            pass
                    self._latest.put_nowait(payload)
                    self._received += 1

        try:
            server.close()
        except OSError:
            pass

    def _publish_latest(self) -> None:
        try:
            payload = self._latest.get_nowait()
        except queue.Empty:
            return
        try:
            message = deserialize_message(payload, PointCloud2)
        except Exception as error:
            self._receive_failures += 1
            self.get_logger().warning(f"discarded invalid PointCloud2 payload: {error}")
            return
        self._publisher.publish(message)
        self._published += 1

    def _log_stats(self) -> None:
        self.get_logger().info(
            "pointcloud TCP receiver stats: "
            f"received={self._received} published={self._published} "
            f"queue_drops={self._queue_drops} receive_failures={self._receive_failures}"
        )

    def destroy_node(self) -> bool:
        self._stop.set()
        if self._server is not None:
            try:
                self._server.close()
            except OSError:
                pass
        self._thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--role", choices=("sender", "receiver"), required=True)
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = PointCloudTcpSender() if parsed.role == "sender" else PointCloudTcpReceiver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
