#!/usr/bin/env python3
"""Publishes the camera pose in the tag-defined world and relative to a settable anchor."""

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from rwrumi_pose.tag_localizer import (
    TagLocalizer,
    invert_transform,
    load_tag_map,
    rotation_to_quaternion,
)

LATEST_FRAME_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT
)


def image_to_bgr(msg: Image) -> np.ndarray:
    channels = {"bgr8": 3, "rgb8": 3, "mono8": 1}.get(msg.encoding)
    if channels is None:
        raise ValueError(f"Unsupported image encoding {msg.encoding}")
    row_bytes = msg.width * channels
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :row_bytes]
    image = raw.reshape(msg.height, msg.width, channels)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if msg.encoding == "mono8":
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return image.copy()


def pose_message(T: np.ndarray, header_stamp, frame_id: str) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = header_stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in T[:3, 3])
    x, y, z, w = (float(v) for v in rotation_to_quaternion(T[:3, :3]))
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = x, y, z, w
    return msg


class TagLocalizerNode(Node):
    def __init__(self):
        super().__init__("tag_localizer")
        tag_map_path = str(self.declare_parameter("tag_map", "").value)
        if not tag_map_path:
            raise ValueError("Parameter 'tag_map' must point to a tag map YAML file")
        self.tags = load_tag_map(tag_map_path)
        self.world_frame = str(self.declare_parameter("world_frame", "world").value)
        self.camera_frame = str(self.declare_parameter("camera_frame", "umi_camera_optical_frame").value)
        self.max_reprojection_px = float(self.declare_parameter("max_reprojection_px", 3.0).value)
        self.max_ambiguity = float(self.declare_parameter("max_ambiguity", 0.6).value)
        self.prior_timeout_s = float(self.declare_parameter("prior_timeout_s", 0.5).value)

        self.localizer = None
        self.camera_info_key = None
        self.last_pose = None
        self.last_pose_time = None
        self.T_world_anchor = None

        self.pose_pub = self.create_publisher(PoseStamped, "/umi/pose/camera", 10)
        self.relative_pub = self.create_publisher(PoseStamped, "/umi/pose/relative", 10)
        self.status_pub = self.create_publisher(DiagnosticStatus, "/umi/pose/status", 10)
        self.debug_pub = self.create_publisher(Image, "/umi/pose/debug_image", qos_profile_sensor_data)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_service(Trigger, "/umi/pose/set_anchor", self.set_anchor)
        self.create_subscription(CameraInfo, "/umi/camera/color/camera_info", self.on_camera_info, LATEST_FRAME_QOS)
        self.create_subscription(Image, "/umi/camera/color/image_raw", self.on_image, LATEST_FRAME_QOS)
        self.get_logger().info(f"Loaded {len(self.tags)} tag(s): {sorted(self.tags)}")

    def on_camera_info(self, msg: CameraInfo) -> None:
        key = (tuple(msg.k), tuple(msg.d))
        if key == self.camera_info_key:
            return
        self.camera_info_key = key
        self.localizer = TagLocalizer(
            self.tags, np.array(msg.k), np.array(msg.d), self.max_reprojection_px, self.max_ambiguity
        )

    def set_anchor(self, _request, response):
        if self.last_pose is None or not self._prior_is_fresh():
            response.success = False
            response.message = "No recent pose: keep a known tag in view and try again"
            return response
        self.T_world_anchor = self.last_pose.copy()
        response.success = True
        response.message = "Anchor set to the current camera pose"
        return response

    def _prior_is_fresh(self) -> bool:
        if self.last_pose_time is None:
            return False
        return (self.get_clock().now() - self.last_pose_time).nanoseconds * 1e-9 <= self.prior_timeout_s

    def on_image(self, msg: Image) -> None:
        if self.localizer is None:
            return
        bgr = image_to_bgr(msg)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        prior = self.last_pose if self._prior_is_fresh() else None
        result = self.localizer.localize(gray, prior)

        if result.T_world_camera is not None:
            self.last_pose = result.T_world_camera
            self.last_pose_time = self.get_clock().now()
            if self.T_world_anchor is None:
                self.T_world_anchor = result.T_world_camera.copy()
            self.publish_pose(result.T_world_camera, msg.header.stamp)

        self.publish_status(result, msg.header.stamp)
        if self.debug_pub.get_subscription_count() > 0:
            self.publish_debug(bgr, result, msg)

    def publish_pose(self, T_world_camera: np.ndarray, stamp) -> None:
        pose = pose_message(T_world_camera, stamp, self.world_frame)
        self.pose_pub.publish(pose)

        T_anchor_camera = invert_transform(self.T_world_anchor) @ T_world_camera
        self.relative_pub.publish(pose_message(T_anchor_camera, stamp, "umi_anchor"))

        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = self.camera_frame
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def publish_status(self, result, stamp) -> None:
        status = DiagnosticStatus()
        status.name = "umi_pose"
        status.hardware_id = "oakd"
        status.level = DiagnosticStatus.OK if result.status == "ok" else DiagnosticStatus.WARN
        status.message = result.status
        status.values = [
            KeyValue(key="stamp", value=f"{stamp.sec}.{stamp.nanosec:09d}"),
            KeyValue(key="visible_ids", value=str(result.visible_ids)),
            KeyValue(key="used_ids", value=str(result.used_ids)),
            KeyValue(key="reprojection_px", value=f"{result.reprojection_px:.3f}"),
            KeyValue(key="ambiguity", value=f"{result.ambiguity:.3f}"),
        ]
        self.status_pub.publish(status)

    def publish_debug(self, bgr: np.ndarray, result, source: Image) -> None:
        if result.corners:
            cv2.aruco.drawDetectedMarkers(bgr, result.corners, result.corner_ids)
        color = (0, 180, 0) if result.status == "ok" else (0, 0, 220)
        label = f"{result.status}  tags {result.used_ids}  err {result.reprojection_px:.2f} px"
        cv2.putText(bgr, label, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
        debug = Image()
        debug.header = source.header
        debug.height, debug.width = bgr.shape[:2]
        debug.encoding = "bgr8"
        debug.step = int(bgr.strides[0])
        debug.data.frombytes(bgr.tobytes())
        self.debug_pub.publish(debug)


def main():
    rclpy.init()
    node = TagLocalizerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
