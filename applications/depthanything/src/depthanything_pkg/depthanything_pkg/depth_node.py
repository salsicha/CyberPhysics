#!/usr/bin/env python3
from typing import Optional

import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header

from depthanything_pkg import metric_depth


class DepthAnythingNode(Node):
    def __init__(self):
        super().__init__('depth_anything_v2_node')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_topic', '/depth_anything/depth/image')
        self.declare_parameter('pointcloud_topic', '/depth_anything/points')
        self.declare_parameter('encoder', 'vits')
        self.declare_parameter('metric_dataset', 'vkitti')
        self.declare_parameter('model_path', '')
        self.declare_parameter('max_depth', 80.0)
        self.declare_parameter('min_depth', 0.05)
        self.declare_parameter('publish_pointcloud', True)
        self.declare_parameter('pointcloud_stride', 4)
        # Optional per-frame scale alignment (README.md, "Scale alignment"):
        # a metric depth image registered to the colour camera; empty: off.
        self.declare_parameter('scale_reference_topic', '')
        self.declare_parameter('scale_reference_min_m', 0.3)
        self.declare_parameter('scale_reference_max_m', 4.0)
        self.declare_parameter('scale_reference_stride', 8)
        self.declare_parameter('scale_reference_min_samples', 100)
        self.declare_parameter('scale_reference_max_age_s', 0.05)
        self.declare_parameter('scale_hold_s', 1.0)

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.encoder = self.get_parameter('encoder').value
        self.metric_dataset = self.get_parameter('metric_dataset').value.lower()
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.publish_pointcloud = bool(self.get_parameter('publish_pointcloud').value)
        self.pointcloud_stride = max(1, int(self.get_parameter('pointcloud_stride').value))
        self.camera_info: Optional[CameraInfo] = None
        scale_reference_topic = self.get_parameter('scale_reference_topic').value
        self.aligner = None
        if scale_reference_topic:
            self.aligner = metric_depth.ScaleAligner(
                self.get_parameter('scale_reference_min_m').value,
                self.get_parameter('scale_reference_max_m').value,
                self.get_parameter('scale_reference_stride').value,
                self.get_parameter('scale_reference_min_samples').value,
                self.get_parameter('scale_reference_max_age_s').value,
                self.get_parameter('scale_hold_s').value)

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Using device: {self.device}')

        # The model's output is its sigmoid head times max_depth, so
        # max_depth must be the checkpoint's training value (80 vkitti, 20
        # hypersim) or every depth is scaled by the difference.
        trained = metric_depth.TRAINED_MAX_DEPTH.get(self.metric_dataset)
        if trained is not None and abs(trained - self.max_depth) > 1e-6:
            self.get_logger().warn(
                f'max_depth {self.max_depth:g} differs from the {self.metric_dataset} '
                f'checkpoint\'s {trained:g}: depths are scaled by {self.max_depth / trained:.3g}')
        self.model, model_path = metric_depth.load_model(
            self.encoder, self.metric_dataset, self.max_depth,
            self.get_parameter('model_path').value, self.device)
        self.get_logger().info(
            f'Loaded Depth Anything V2 metric {self.metric_dataset}/{self.encoder} from {model_path}'
        )

        self.bridge = CvBridge()
        self.depth_pub = self.create_publisher(Image, depth_topic, 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, pointcloud_topic, 10)
        # Keep only the newest camera frame: inference is slower than the
        # camera, and a deeper queue makes every depth image several frames
        # old (median 0.31 s with sensor-data depth 5 on RACECAR, 0.13 s
        # while DDS was dropping most frames).
        latest_frame_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=qos_profile_sensor_data.reliability,
            durability=qos_profile_sensor_data.durability)
        self.create_subscription(
            Image, image_topic, self.image_callback, latest_frame_qos)
        self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback,
            qos_profile_sensor_data)
        if self.aligner is not None:
            self.create_subscription(
                Image, scale_reference_topic, self.scale_reference_callback,
                QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                           reliability=qos_profile_sensor_data.reliability,
                           durability=qos_profile_sensor_data.durability))
            self.get_logger().info(
                f'Scaling each depth to {scale_reference_topic} (median ratio over '
                f'{self.aligner.min_m:g}-{self.aligner.max_m:g} m)')

        self.get_logger().info(
            f'Subscribed to {image_topic} and {camera_info_topic}; publishing depth to {depth_topic}'
        )
        if self.publish_pointcloud:
            self.get_logger().info(f'Publishing point clouds to {pointcloud_topic}')

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    @staticmethod
    def stamp_s(header: Header) -> float:
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def scale_reference_callback(self, msg: Image) -> None:
        try:
            pixels = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.aligner.add_reference(
                self.stamp_s(msg.header), metric_depth.depth_image_to_m(pixels, msg.encoding))
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f'Unusable scale reference: {exc}', throttle_duration_sec=10.0)

    def image_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        with torch.no_grad():
            depth = self.model.infer_image(cv_image)

        depth = metric_depth.mask_range(depth, self.min_depth, self.max_depth)
        if self.aligner is not None:
            scale = self.aligner.scale_for(self.stamp_s(msg.header), depth)
            if scale is None:
                self.get_logger().warn('No scale reference for this frame: not published',
                                       throttle_duration_sec=5.0)
                return
            depth = metric_depth.mask_range(depth * np.float32(scale), self.min_depth, self.max_depth)
            self.get_logger().info(f'Depth scale {scale:.3f}', throttle_duration_sec=10.0)

        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        depth_msg.header = msg.header
        self.depth_pub.publish(depth_msg)

        if self.publish_pointcloud and self.camera_info is not None:
            self.pointcloud_pub.publish(self.depth_to_pointcloud(depth, msg.header, self.camera_info))

    def depth_to_pointcloud(self, depth: np.ndarray, header: Header, info: CameraInfo) -> PointCloud2:
        fx, fy = float(info.k[0]), float(info.k[4])
        cx, cy = float(info.k[2]), float(info.k[5])
        if fx == 0.0 or fy == 0.0:
            self.get_logger().warn('CameraInfo has zero focal length; skipping point cloud projection')
            points = np.empty((0, 3), dtype=np.float32)
        else:
            ys, xs = np.mgrid[0:depth.shape[0]:self.pointcloud_stride, 0:depth.shape[1]:self.pointcloud_stride]
            zs = depth[ys, xs]
            # image_callback already NaN-masked out-of-range depths.
            valid = np.isfinite(zs)
            xs = xs[valid].astype(np.float32)
            ys = ys[valid].astype(np.float32)
            zs = zs[valid].astype(np.float32)
            points = np.column_stack(((xs - cx) * zs / fx, (ys - cy) * zs / fy, zs))

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = points.shape[0]
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True  # invalid points are filtered out above
        cloud.data = points.astype(np.float32, copy=False).tobytes()
        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = DepthAnythingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
