#!/usr/bin/env python3

import math
from pathlib import Path

import depthai as dai
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class MegaDepthNode(Node):
    """Run MegaDepth on the OAK-1 Myriad X and publish relative depth."""

    def __init__(self):
        super().__init__('oak1_megadepth')

        self.declare_parameter(
            'blob_path',
            '/models/megadepth_192x256_openvino_2021.4_6shave.blob')
        self.declare_parameter('device_ip', 'auto')
        self.declare_parameter('image_topic', '/oak1/image_raw')
        self.declare_parameter('depth_topic', '/oak1/relative_depth')
        self.declare_parameter('camera_info_topic', '/oak1/camera_info')
        self.declare_parameter('frame_id', 'oak1_optical_frame')
        self.declare_parameter('fps', 20.0)

        self.width = 256
        self.height = 192
        self.frame_id = self.get_parameter('frame_id').value
        blob_path = Path(self.get_parameter('blob_path').value)
        if not blob_path.is_file():
            raise FileNotFoundError(f'MegaDepth blob not found: {blob_path}')
        if blob_path.stat().st_size < 1_000_000:
            raise RuntimeError(
                f'MegaDepth model is a Git LFS pointer, not a blob: {blob_path}. '
                'Run git lfs pull before building.')

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(
            Image, self.get_parameter('image_topic').value, 10)
        self.depth_pub = self.create_publisher(
            Image, self.get_parameter('depth_topic').value, 10)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, self.get_parameter('camera_info_topic').value, 10)

        pipeline = dai.Pipeline()
        pipeline.setOpenVINOVersion(dai.OpenVINO.VERSION_2021_4)

        camera = pipeline.create(dai.node.ColorCamera)
        camera.setPreviewSize(self.width, self.height)
        camera.setInterleaved(False)
        camera.setFps(float(self.get_parameter('fps').value))
        camera.setResolution(
            dai.ColorCameraProperties.SensorResolution.THE_1080_P)

        network = pipeline.create(dai.node.NeuralNetwork)
        network.setBlobPath(str(blob_path))
        network.setNumPoolFrames(4)
        network.input.setBlocking(False)
        network.setNumInferenceThreads(2)

        camera_out = pipeline.create(dai.node.XLinkOut)
        camera_out.setStreamName('camera')
        depth_out = pipeline.create(dai.node.XLinkOut)
        depth_out.setStreamName('depth')

        camera.preview.link(network.input)
        network.passthrough.link(camera_out.input)
        network.out.link(depth_out.input)

        device_ip = self.get_parameter('device_ip').value
        if device_ip and device_ip != 'auto':
            self.device = dai.Device(pipeline, dai.DeviceInfo(device_ip))
        else:
            self.device = dai.Device(pipeline)

        self.camera_queue = self.device.getOutputQueue(
            'camera', maxSize=8, blocking=False)
        self.depth_queue = self.device.getOutputQueue(
            'depth', maxSize=8, blocking=False)
        self.camera_packets = {}
        self.depth_packets = {}
        self.camera_info = self._read_camera_info()
        self.timer = self.create_timer(0.005, self._poll)

        self.get_logger().info(
            f'OAK-1 MegaDepth ready: {blob_path.name} at '
            f'{self.width}x{self.height}')

    def _read_camera_info(self):
        info = CameraInfo()
        info.width = self.width
        info.height = self.height
        try:
            calibration = self.device.readCalibration()
            socket = getattr(dai.CameraBoardSocket, 'CAM_A', None)
            if socket is None:
                socket = dai.CameraBoardSocket.RGB
            intrinsic = calibration.getCameraIntrinsics(
                socket, self.width, self.height)
            distortion = calibration.getDistortionCoefficients(socket)
            info.k = [
                intrinsic[0][0], intrinsic[0][1], intrinsic[0][2],
                intrinsic[1][0], intrinsic[1][1], intrinsic[1][2],
                intrinsic[2][0], intrinsic[2][1], intrinsic[2][2],
            ]
            info.d = list(distortion)
            info.distortion_model = 'plumb_bob'
        except Exception as exc:
            horizontal_fov = math.radians(69.0)
            focal = self.width / (2.0 * math.tan(horizontal_fov / 2.0))
            info.k = [
                focal, 0.0, self.width / 2.0,
                0.0, focal, self.height / 2.0,
                0.0, 0.0, 1.0,
            ]
            self.get_logger().warning(
                f'Using approximate OAK-1 intrinsics: {exc}')
        return info

    @staticmethod
    def _store_packet(queue, packets):
        while True:
            packet = queue.tryGet()
            if packet is None:
                break
            packets[packet.getSequenceNum()] = packet
        while len(packets) > 8:
            packets.pop(min(packets))

    def _poll(self):
        self._store_packet(self.camera_queue, self.camera_packets)
        self._store_packet(self.depth_queue, self.depth_packets)
        matching = sorted(
            set(self.camera_packets).intersection(self.depth_packets))
        if not matching:
            return

        sequence = matching[-1]
        camera_packet = self.camera_packets.pop(sequence)
        depth_packet = self.depth_packets.pop(sequence)
        for old_sequence in [s for s in self.camera_packets if s < sequence]:
            self.camera_packets.pop(old_sequence)
        for old_sequence in [s for s in self.depth_packets if s < sequence]:
            self.depth_packets.pop(old_sequence)

        frame = camera_packet.getCvFrame()
        relative_depth = np.asarray(
            depth_packet.getFirstLayerFp16(),
            dtype=np.float32).reshape(self.height, self.width)
        relative_depth[~np.isfinite(relative_depth)] = 0.0

        stamp = self.get_clock().now().to_msg()
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id

        depth_msg = self.bridge.cv2_to_imgmsg(
            relative_depth, encoding='32FC1')
        depth_msg.header = image_msg.header

        self.camera_info.header = image_msg.header
        self.image_pub.publish(image_msg)
        self.depth_pub.publish(depth_msg)
        self.camera_info_pub.publish(self.camera_info)

    def destroy_node(self):
        self.device.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MegaDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
