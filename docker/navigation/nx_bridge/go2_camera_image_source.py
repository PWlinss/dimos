#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Publish Go2 H264 multicast video as ROS CompressedImage."""

from __future__ import annotations

import time

import gi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402


class Go2CameraImageSource(Node):
    def __init__(self) -> None:
        super().__init__("go2_camera_image_source")
        self.declare_parameter("udp_address", "230.1.1.1")
        self.declare_parameter("udp_port", 1720)
        self.declare_parameter("interface", "enp109s0")
        self.declare_parameter("target_fps", 5)
        self.declare_parameter("image_topic", "/camera/image_raw/compressed")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 360)
        self.declare_parameter("jpeg_quality", 60)
        self.declare_parameter("frame_id", "camera_optical_frame")

        self.udp_address = self.get_parameter("udp_address").value
        self.udp_port = int(self.get_parameter("udp_port").value)
        self.interface = self.get_parameter("interface").value
        self.target_fps = int(self.get_parameter("target_fps").value)
        self.image_topic = self.get_parameter("image_topic").value
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.frame_id = self.get_parameter("frame_id").value

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(CompressedImage, self.image_topic, qos)
        self.frame_count = 0
        self.last_report_time = time.time()
        self.last_publish_time = 0.0

        Gst.init(None)
        self.pipeline = None
        self.appsink = None
        self._init_gstreamer()

    def _init_gstreamer(self) -> None:
        pipeline = (
            f"udpsrc address={self.udp_address} port={self.udp_port} "
            f"multicast-iface={self.interface} ! "
            "queue ! "
            "application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! "
            "videoconvert ! videoscale ! "
            f"video/x-raw,width={self.width},height={self.height} ! "
            f"jpegenc quality={self.jpeg_quality} ! "
            "appsink name=sink emit-signals=True drop=True sync=false"
        )
        self.get_logger().info(f"GStreamer pipeline: {pipeline}")
        self.pipeline = Gst.parse_launch(pipeline)
        self.appsink = self.pipeline.get_by_name("sink")
        self.appsink.connect("new-sample", self._on_frame, None)
        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info(
            f"Go2 camera source started: topic={self.image_topic} "
            f"udp={self.udp_address}:{self.udp_port} interface={self.interface}"
        )

    def _on_frame(self, sink, _) -> Gst.FlowReturn:
        sample = sink.emit("pull-sample")
        if not sample:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        if not buf:
            return Gst.FlowReturn.OK

        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            self.get_logger().error("Failed to map camera buffer")
            return Gst.FlowReturn.ERROR

        try:
            now = time.time()
            if self.target_fps > 0 and now - self.last_publish_time < 1.0 / self.target_fps:
                return Gst.FlowReturn.OK

            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.format = "jpeg"
            msg.data = bytes(map_info.data)
            self.publisher.publish(msg)
            self.last_publish_time = now

            self.frame_count += 1
            if now - self.last_report_time >= 5.0:
                fps = self.frame_count / (now - self.last_report_time)
                self.get_logger().info(f"Published camera frames at {fps:.2f} FPS")
                self.frame_count = 0
                self.last_report_time = now
        finally:
            buf.unmap(map_info)

        return Gst.FlowReturn.OK

    def destroy_node(self) -> bool:
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Go2CameraImageSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
