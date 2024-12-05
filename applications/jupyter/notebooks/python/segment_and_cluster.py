#!/usr/bin/env python

from geometry_msgs.msg import Pose, Point, Vector3
import ros_numpy
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import open3d as o3d
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

INPUT_TOPIC = "/m02/camera2/depth/color/points"
OUTPUT_TOPIC = "/m02/markers/tracking"

TARGET_FRAME_ID = "m02/workspace"
CAMERA_FRAME_ID = "m02/camera2_color_optical_frame"


def segment_cld(pc_masked, size):
    """Segment plane in cloud"""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_masked)

    # RANSAC: Random sample consensus
    plane_model, inliers = pcd.segment_plane(distance_threshold=size, ransac_n=3, num_iterations=500)
    inlier_cld = pcd.select_by_index(inliers)
    outlier_cld = pcd.select_by_index(inliers, invert=True)

    return inlier_cld, outlier_cld


def extract_cloud(msg, bounds):
    """Extract cloud from message"""
    pointclouds_rgb = ros_numpy.point_cloud2.pointcloud2_to_array(msg)

    pointclouds = np.zeros(pointclouds_rgb.shape + (3,), dtype=np.float32)
    pointclouds[..., 0] = pointclouds_rgb['x']
    pointclouds[..., 1] = pointclouds_rgb['y']
    pointclouds[..., 2] = pointclouds_rgb['z']

    pc_rgb = ros_numpy.point_cloud2.split_rgb_field(pointclouds_rgb)

    rgb = np.zeros(pc_rgb.shape + (3,), dtype=np.uint8)
    rgb[..., 0] = pc_rgb['r']
    rgb[..., 1] = pc_rgb['g']
    rgb[..., 2] = pc_rgb['b']

    if bounds:
        x_min_b = (pointclouds[:, 0] > bounds[0][0])
        y_min_b = (pointclouds[:, 1] > bounds[0][1])
        z_min_b = (pointclouds[:, 2] > bounds[0][2])
        x_max_b = (pointclouds[:, 0] < bounds[1][0])
        y_max_b = (pointclouds[:, 1] < bounds[1][1])
        z_max_b = (pointclouds[:, 2] < bounds[1][2])
        pointclouds = pointclouds[x_min_b & y_min_b & z_min_b & x_max_b & y_max_b & z_max_b]
    
    return pointclouds, rgb

    
def get_bounds(pts):
    """Get bounds of cloud"""
    bounds = [[0, 0, 0], [0, 0, 0]]
    xyz_load = np.asarray(pts.points)
    bounds[0][0] = xyz_load[:, 0].min()
    bounds[0][1] = xyz_load[:, 1].min()
    bounds[0][2] = xyz_load[:, 2].min()
    bounds[1][0] = xyz_load[:, 0].max()            
    bounds[1][1] = xyz_load[:, 1].max()            
    bounds[1][2] = xyz_load[:, 2].max()            
    return bounds


def lagest_labeled_cloud(in_cld, size, min_pts, index):
    """Get largest cloud cluster"""
    labels = np.array(in_cld.cluster_dbscan(eps=size, min_points=min_pts))
    label_mask = (labels == index)
    mask_indices = (label_mask == True).nonzero()[0]
    masked_cld = in_cld.select_by_index(mask_indices)
    return masked_cld


def get_markers(timestamp, points):
    """Markers"""
    markers = []
    for pt in points:
        print(pt)
        marker = Marker(
            header=Header(
                stamp = timestamp,
                frame_id = CAMERA_FRAME_ID,
            ),
            type = Marker.SPHERE,
            action = Marker.ADD,
            pose = Pose(position=Point(x=pt[0], y=pt[1], z=pt[2])),
            color = ColorRGBA(r=1, a=1),
            scale = Vector3(0.05, 0.05, 0.05),
        )
        markers.append(marker)
    return MarkerArray(markers=markers)


class ClusterFinder:
    def __init__(self):
        self.init_listener = True

        # Setup
        self.feature_size = 0.01
        self.label_index = 0
        self.min_pts = 50
        self.color_min = 100                
        self.size_outlier = 0.005
        self.min_bowl_pts = 1000

        self.bounds = [[], []]
        self.median = [0.0, 0.0, 0.0]

        self.sub = rospy.Subscriber(INPUT_TOPIC, PointCloud2, self.pc2_callback)
        self.pub = rospy.Publisher('test_markers', MarkerArray, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


    def transform_between_frames(self, msg):
        """Example transformation between m02/workspace and m02/camera2_color_optical_frame"""
        try:
            trans = self.tf_buffer.lookup_transform(TARGET_FRAME_ID, msg.header.frame_id, msg.header.stamp, rospy.Duration(1.0))
            cloud_out = do_transform_cloud(msg, trans)
            print("Transformed cloud: ", cloud_out.header)
        except Exception as e:
            print("E: ", str(e))

    def pc2_callback(self, msg):
        """Point cloud callback"""
        # Transform example
        self.transform_between_frames(msg)

        # Filter messages to 1 Hz
        if msg.header.seq % 5 == 0:
            if self.init_listener:
                self.init_listener = False

                # Convert message to cloud
                pointclouds, rgb = extract_cloud(msg, None)
                
                # Mask on white conveyor belt
                color_mask = (rgb[:, 0] > self.color_min) & (rgb[:, 1] > self.color_min) & (rgb[:, 2] > self.color_min)
                pc_color_masked = pointclouds[color_mask]
                
                # Segment cloud
                inlier_cloud, outlier_cld = segment_cld(pc_color_masked, self.feature_size)

                # Cluster cloud by label
                masked_cld = lagest_labeled_cloud(inlier_cloud, self.feature_size, self.min_pts, self.label_index)

                # Get bounds
                self.bounds = get_bounds(masked_cld)
                print("bounds: ", self.bounds)

            else:
                # Convert message to cloud
                pointclouds, rgb = extract_cloud(msg, self.bounds)

                # Segment cloud
                inlier_cld, outlier_cld = segment_cld(pointclouds, self.size_outlier)

                # Cluster cloud by label
                outlier_cluster = lagest_labeled_cloud(outlier_cld, self.feature_size, self.min_pts, self.label_index)

                # Make centroids for labeled clouds
                if len(outlier_cluster.points) > self.min_bowl_pts:
                    xyz_pts = np.asarray(outlier_cluster.points)
                    self.median = [np.median(xyz_pts[:, 0]), np.median(xyz_pts[:, 1]), np.median(xyz_pts[:, 2])]                

            # Publish markers
            tracking_markers = get_markers(msg.header.stamp, [self.median])
            self.pub.publish(tracking_markers)


def main():
    rospy.init_node('take_home', anonymous=True)
    take_home_test = ClusterFinder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()

