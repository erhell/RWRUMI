#!/usr/bin/env python3
"""OAK-D Pro driver: publishes synced color and aligned depth, calibration and raw IMU."""

from datetime import timedelta

import depthai as dai
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu

WIDTH = 960
HEIGHT = 540
IMU_RATE_HZ = 200


def build_pipeline(fps: float, lens_position: int, enable_imu: bool) -> dai.Pipeline:
    pipeline = dai.Pipeline()

    color = pipeline.create(dai.node.ColorCamera)
    color.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    color.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    color.setIspScale(1, 2)
    color.setFps(fps)
    # Autofocus would change the intrinsics while recording.
    color.initialControl.setManualFocus(lens_position)

    left = pipeline.create(dai.node.MonoCamera)
    right = pipeline.create(dai.node.MonoCamera)
    for camera, socket in ((left, dai.CameraBoardSocket.CAM_B), (right, dai.CameraBoardSocket.CAM_C)):
        camera.setBoardSocket(socket)
        camera.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        camera.setFps(fps)

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)
    stereo.setExtendedDisparity(True)  # a handheld device works close to objects
    # The median filter cannot handle the disparity range of extended + subpixel.
    stereo.initialConfig.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(WIDTH, HEIGHT)
    left.out.link(stereo.left)
    right.out.link(stereo.right)

    sync = pipeline.create(dai.node.Sync)
    sync.setSyncThreshold(timedelta(seconds=0.5 / fps))
    color.isp.link(sync.inputs["color"])
    stereo.depth.link(sync.inputs["depth"])
    frames_out = pipeline.create(dai.node.XLinkOut)
    frames_out.setStreamName("frames")
    sync.out.link(frames_out.input)

    if enable_imu:
        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], IMU_RATE_HZ)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        imu_out = pipeline.create(dai.node.XLinkOut)
        imu_out.setStreamName("imu")
        imu.out.link(imu_out.input)

    return pipeline


def make_camera_info(calibration, frame_id: str) -> CameraInfo:
    socket = dai.CameraBoardSocket.CAM_A
    K = np.array(calibration.getCameraIntrinsics(socket, WIDTH, HEIGHT), dtype=float)
    D = np.array(calibration.getDistortionCoefficients(socket), dtype=float)
    if D.size > 8 and np.all(np.abs(D[8:]) < 1e-9):
        D = D[:8]

    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width = WIDTH
    info.height = HEIGHT
    info.distortion_model = "rational_polynomial"
    info.d = D.tolist()
    info.k = K.flatten().tolist()
    info.r = np.eye(3).flatten().tolist()
    info.p = np.hstack((K, np.zeros((3, 1)))).flatten().tolist()
    return info


def make_image(array: np.ndarray, encoding: str, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = array.shape[:2]
    msg.encoding = encoding
    msg.step = int(array.strides[0])
    msg.data.frombytes(array.tobytes())
    return msg


class OakdNode(Node):
    def __init__(self):
        super().__init__("oakd")
        self.fps = float(self.declare_parameter("fps", 15.0).value)
        self.lens_position = int(self.declare_parameter("lens_position", 130).value)
        self.ir_dot_intensity = float(self.declare_parameter("ir_dot_intensity", 0.0).value)
        self.enable_imu = bool(self.declare_parameter("enable_imu", True).value)
        self.mxid = str(self.declare_parameter("mxid", "").value)
        self.frame_id = str(self.declare_parameter("frame_id", "umi_camera_optical_frame").value)
        self.imu_frame_id = str(self.declare_parameter("imu_frame_id", "umi_camera_imu_frame").value)

        qos = qos_profile_sensor_data
        self.color_pub = self.create_publisher(Image, "/umi/camera/color/image_raw", qos)
        self.color_info_pub = self.create_publisher(CameraInfo, "/umi/camera/color/camera_info", qos)
        self.depth_pub = self.create_publisher(Image, "/umi/camera/depth/image_raw", qos)
        self.depth_info_pub = self.create_publisher(CameraInfo, "/umi/camera/depth/camera_info", qos)
        self.imu_pub = self.create_publisher(Imu, "/umi/camera/imu", 200) if self.enable_imu else None

    def to_ros_time(self, device_timestamp):
        """Convert a host-synced DepthAI timestamp to ROS time, keeping the capture time."""
        age = dai.Clock.now() - device_timestamp
        return (self.get_clock().now() - Duration(nanoseconds=int(age.total_seconds() * 1e9))).to_msg()

    def publish_frames(self, group, camera_info: CameraInfo) -> None:
        color = group["color"]
        depth = group["depth"]
        camera_info.header.stamp = self.to_ros_time(color.getTimestamp())
        header = camera_info.header
        self.color_pub.publish(make_image(color.getCvFrame(), "bgr8", header))
        self.depth_pub.publish(make_image(depth.getFrame(), "16UC1", header))
        self.color_info_pub.publish(camera_info)
        self.depth_info_pub.publish(camera_info)

    def publish_imu(self, imu_data) -> None:
        for packet in imu_data.packets:
            accel = packet.acceleroMeter
            gyro = packet.gyroscope
            msg = Imu()
            msg.header.frame_id = self.imu_frame_id
            msg.header.stamp = self.to_ros_time(gyro.getTimestamp())
            msg.orientation_covariance[0] = -1.0  # raw IMU, no orientation estimate
            msg.linear_acceleration.x = float(accel.x)
            msg.linear_acceleration.y = float(accel.y)
            msg.linear_acceleration.z = float(accel.z)
            msg.angular_velocity.x = float(gyro.x)
            msg.angular_velocity.y = float(gyro.y)
            msg.angular_velocity.z = float(gyro.z)
            self.imu_pub.publish(msg)

    def run(self) -> None:
        pipeline = build_pipeline(self.fps, self.lens_position, self.enable_imu)
        device_info = dai.DeviceInfo(self.mxid) if self.mxid else dai.DeviceInfo()
        with dai.Device(pipeline, device_info) as device:
            speed = device.getUsbSpeed()
            self.get_logger().info(
                f"Connected {device.getDeviceName()} ({device.getMxId()}), USB {speed.name}, {self.fps:g} fps"
            )
            if speed not in (dai.UsbSpeed.SUPER, dai.UsbSpeed.SUPER_PLUS):
                self.get_logger().warn("Camera is on USB 2. Use a USB 3 cable and port for more than about 15 fps.")
            if self.ir_dot_intensity > 0.0:
                device.setIrLaserDotProjectorIntensity(self.ir_dot_intensity)

            camera_info = make_camera_info(device.readCalibration(), self.frame_id)
            frames = device.getOutputQueue("frames", maxSize=2, blocking=False)
            imu = device.getOutputQueue("imu", maxSize=50, blocking=False) if self.enable_imu else None

            while rclpy.ok():
                group = frames.tryGet()
                imu_data = imu.tryGet() if imu is not None else None
                if group is not None:
                    self.publish_frames(group, camera_info)
                if imu_data is not None:
                    self.publish_imu(imu_data)
                rclpy.spin_once(self, timeout_sec=0.0 if group or imu_data else 0.001)


def main():
    rclpy.init()
    node = OakdNode()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
