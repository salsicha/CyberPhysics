#!/usr/bin/env python3
import math
import os
import sys
from typing import Optional

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header

# Put metric_depth first so DepthAnythingV2 resolves to the metric model implementation.
sys.path.insert(0, '/opt/Depth-Anything-V2/metric_depth')
sys.path.append('/opt/Depth-Anything-V2')

try:
    from depth_anything_v2.dpt import DepthAnythingV2
except ImportError as exc:
    print(f'Error importing DepthAnythingV2: {exc}')
    DepthAnythingV2 = None


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

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Using device: {self.device}')

        model_configs = {
            'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
            'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
            'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
            'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]},
        }
        if self.encoder not in model_configs:
            raise ValueError(f'Unsupported encoder {self.encoder}; expected one of {sorted(model_configs)}')
        if DepthAnythingV2 is None:
            raise RuntimeError('DepthAnythingV2 class not found. Is /opt/Depth-Anything-V2 installed?')

        config = dict(model_configs[self.encoder])
        config['max_depth'] = self.max_depth
        self.model = DepthAnythingV2(**config)

        model_path = self.get_parameter('model_path').value
        if not model_path:
            model_path = (
                f'/opt/Depth-Anything-V2/checkpoints/'
                f'depth_anything_v2_metric_{self.metric_dataset}_{self.encoder}.pth'
            )
        if not os.path.exists(model_path):
            raise FileNotFoundError(f'Metric Depth Anything weights not found at {model_path}')

        state_dict = torch.load(model_path, map_location='cpu')
        self.model.load_state_dict(state_dict)
        self.model = self.model.to(self.device).eval()
        self.get_logger().info(
            f'Loaded Depth Anything V2 metric {self.metric_dataset}/{self.encoder} from {model_path}'
        )

        self.bridge = CvBridge()
        self.depth_pub = self.create_publisher(Image, depth_topic, 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, pointcloud_topic, 10)
        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback,
            qos_profile_sensor_data)

        self.get_logger().info(
            f'Subscribed to {image_topic} and {camera_info_topic}; publishing depth to {depth_topic}'
        )
        if self.publish_pointcloud:
            self.get_logger().info(f'Publishing point clouds to {pointcloud_topic}')

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def image_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        with torch.no_grad():
            depth = self.model.infer_image(cv_image)

        depth = np.asarray(depth, dtype=np.float32)
        depth[
            (~np.isfinite(depth)) |
            (depth < self.min_depth) |
            (depth > self.max_depth)] = np.nan

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
