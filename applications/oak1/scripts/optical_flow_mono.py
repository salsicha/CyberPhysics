#!/usr/bin/env python3

import numpy as np
import cv2
from collections import deque
import depthai as dai
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


"""
Optical Flow

Harris corners are detected in frame
Lucas Kanade optical flow tracks features between frames
Translation between frames is the mean of the flow vectors
Rotation between frames is homography estimation

Translation and Rotation vectors are published with ROS

"""


class CameraMotionEstimator:

    def __init__(self, filter_weight=0.5, motion_threshold=0.01, rotation_threshold=0.05):
        self.last_avg_flow = np.array([0.0, 0.0])
        self.filter_weight = filter_weight
        self.motion_threshold = motion_threshold
        self.rotation_threshold = rotation_threshold

    def estimate_motion(self, feature_paths):
        most_prominent_motion = "Camera Staying Still"
        max_magnitude = 0.0
        avg_flow = np.array([0.0, 0.0])
        total_rotation = 0.0
        vanishing_point = np.array([0.0, 0.0])
        num_features = len(feature_paths)

        print(f"Number of features: {num_features}")

        if num_features == 0:
            return most_prominent_motion, vanishing_point, avg_flow

        src_hom = []
        dst_hom = []

        for path in feature_paths.values():
            if len(path) >= 2:
                src_hom.append([path[-2].x, path[-2].y])
                dst_hom.append([path[-1].x, path[-1].y])
                src = np.array([path[-2].x, path[-2].y])
                dst = np.array([path[-1].x, path[-1].y])
                avg_flow += dst - src
                motion_vector = dst + (dst - src)
                vanishing_point += motion_vector
                rotation = np.arctan2(dst[1] - src[1], dst[0] - src[0])
                total_rotation += rotation
        

        ## Using findHomography to estimate translation, rotation
        translation = [0.0, 0.0, 0.0]
        if len(src_hom) >= 4:
            src_pts = np.array(src_hom).reshape(-1, 1, 2)
            dst_pts = np.array(dst_hom).reshape(-1, 1, 2)
            H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            # print("Homography matrix:\n", H)
            scale_x = np.sqrt(np.linalg.det(H[:3, :3]))
            rotation_matrix = H[:3, :3] / scale_x
            translation_vector = H[:3, 2]
            translation = translation_vector.tolist()
            print(f"Translation: {translation_vector}")
            print(f"Rotation: {rotation_matrix}")



        avg_flow /= num_features
        avg_rotation = total_rotation / num_features
        vanishing_point /= num_features


        print(f"Average Flow: {avg_flow}")
        print(f"Average Rotation: {avg_rotation}")


        avg_flow = (self.filter_weight * self.last_avg_flow +
                    (1 - self.filter_weight) * avg_flow)
        self.last_avg_flow = avg_flow

        flow_magnitude = np.linalg.norm(avg_flow)
        rotation_magnitude = abs(avg_rotation)

        if flow_magnitude > max_magnitude and flow_magnitude > self.motion_threshold:
            if abs(avg_flow[0]) > abs(avg_flow[1]):
                most_prominent_motion = 'Right' if avg_flow[0] < 0 else 'Left'
            else:
                most_prominent_motion = 'Down' if avg_flow[1] < 0 else 'Up'
            max_magnitude = flow_magnitude

        if rotation_magnitude > max_magnitude and rotation_magnitude > self.rotation_threshold:
            most_prominent_motion = 'Rotating'

        return most_prominent_motion, vanishing_point, avg_flow, translation


class FeatureTrackerDrawer:
    lineColor = (200, 0, 200)
    pointColor = (0, 0, 255)
    vanishingPointColor = (255, 0, 255)  # Violet color for vanishing point
    circleRadius = 2
    maxTrackedFeaturesPathLength = 30
    trackedFeaturesPathLength = 10
    trackedIDs = None
    trackedFeaturesPath = None

    def trackFeaturePath(self, features):
        newTrackedIDs = set()
        for currentFeature in features:
            currentID = currentFeature.id
            newTrackedIDs.add(currentID)

            if currentID not in self.trackedFeaturesPath:
                self.trackedFeaturesPath[currentID] = deque()

            path = self.trackedFeaturesPath[currentID]
            path.append(currentFeature.position)

            while(len(path) > max(1, FeatureTrackerDrawer.trackedFeaturesPathLength)):
                path.popleft()

            self.trackedFeaturesPath[currentID] = path

        featuresToRemove = set()

        for oldId in self.trackedIDs:
            if oldId not in newTrackedIDs:
                featuresToRemove.add(oldId)

        for id in featuresToRemove:
            self.trackedFeaturesPath.pop(id)

        self.trackedIDs = newTrackedIDs


    def drawVanishingPoint(self, img, vanishing_point):
        cv2.circle(img, (int(vanishing_point[0]), int(vanishing_point[1])), self.circleRadius, self.vanishingPointColor, -1, cv2.LINE_AA, 0)


    # Define color mapping for directions
    direction_colors = {
        "Up": (0, 255, 255),  # Yellow
        "Down": (0, 255, 0),  # Green
        "Left": (255, 0, 0),  # Blue
        "Right": (0, 0, 255), # Red
    }


    def drawFeatures(self, img, vanishing_point=None, prominent_motion=None):
        # Get the appropriate point color based on the prominent motion
        if prominent_motion in self.direction_colors:
            point_color = self.direction_colors[prominent_motion]
        else:
            point_color = self.pointColor

        for featurePath in self.trackedFeaturesPath.values():
            path = featurePath

            for j in range(len(path) - 1):
                src = (int(path[j].x), int(path[j].y))
                dst = (int(path[j + 1].x), int(path[j + 1].y))
                cv2.line(img, src, dst, point_color, 1, cv2.LINE_AA, 0)

            j = len(path) - 1
            cv2.circle(img, (int(path[j].x), int(path[j].y)), self.circleRadius, point_color, -1, cv2.LINE_AA, 0)

        # Draw the direction text on the image

        if prominent_motion:
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1
            font_thickness = 2
            text_size = cv2.getTextSize(prominent_motion, font, font_scale, font_thickness)[0]
            text_x = (img.shape[1] - text_size[0]) // 2
            text_y = text_size[1] + 20  # 20 pixels from the top

            # Get the appropriate color based on the prominent motion
            text_color = self.direction_colors.get(prominent_motion, (255, 255, 255))  # Default to white

            # Draw the text
            cv2.putText(img, prominent_motion, (text_x, text_y), font, font_scale, text_color, font_thickness, cv2.LINE_AA)

        # Draw vanishing point if provided
        if vanishing_point is not None:
            self.drawVanishingPoint(img, vanishing_point)

    def __init__(self, windowName):
        self.windowName = windowName
        cv2.namedWindow(windowName)
        self.trackedIDs = set()
        self.trackedFeaturesPath = dict()

def create_pipeline():
    pipeline = dai.Pipeline()

    # Create a MonoCamera node and set its properties
    mono_left = pipeline.create(dai.node.ColorCamera)
    # mono_left.setCamera("left")
    # mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    # mono_left.setFps(15)

    # Create a FeatureTracker node
    feature_tracker_left = pipeline.create(dai.node.FeatureTracker)

    # Create XLinkOut nodes for output streams
    xout_tracked_features_left = pipeline.create(dai.node.XLinkOut)
    xout_passthrough_left = pipeline.create(dai.node.XLinkOut)

    # Set stream names
    xout_tracked_features_left.setStreamName("trackedFeaturesLeft")
    xout_passthrough_left.setStreamName("passthroughLeft")

    # Allocate resources for improved performance
    num_shaves = 2
    num_memory_slices = 2
    feature_tracker_left.setHardwareResources(num_shaves, num_memory_slices)

    # Link the nodes
    mono_left.video.link(feature_tracker_left.inputImage)
    feature_tracker_left.passthroughInputImage.link(xout_passthrough_left.input)
    feature_tracker_left.outputFeatures.link(xout_tracked_features_left.input)

    return pipeline


if __name__ == '__main__':

    rclpy.init()
    node = rclpy.create_node('publisher')
    flow_publisher = node.create_publisher(Float32MultiArray, "flow_pub", 10)

    pipeline = create_pipeline()

    with dai.Device(pipeline) as device:
        output_features_left_queue = device.getOutputQueue(
            "trackedFeaturesLeft", maxSize=4, blocking=False)
        passthrough_image_left_queue = device.getOutputQueue(
            "passthroughLeft", maxSize=4, blocking=False)

        left_window_name = "Left"
        left_feature_drawer = FeatureTrackerDrawer(left_window_name)
        camera_estimator_left = CameraMotionEstimator(
            filter_weight=0.5, motion_threshold=0.3, rotation_threshold=0.5)

        while True:
            in_passthrough_frame_left = passthrough_image_left_queue.get()
            passthrough_frame_left = in_passthrough_frame_left.getFrame()
            left_frame = cv2.cvtColor(passthrough_frame_left, cv2.COLOR_GRAY2BGR)

            tracked_features_left = output_features_left_queue.get().trackedFeatures

            motions_left, vanishing_pt_left, avg_flow, translation = camera_estimator_left.estimate_motion(
                left_feature_drawer.trackedFeaturesPath)

            left_feature_drawer.trackFeaturePath(tracked_features_left)

            # left_feature_drawer.drawFeatures(left_frame, vanishing_pt_left, motions_left)

            # print("Motions:", motions_left)
            # cv2.imshow(left_window_name, left_frame)

            # ROS publisher
            msg = Float32MultiArray()
            msg.data = [translation[0], translation[1], translation[2], avg_flow[0], avg_flow[1]]
            flow_publisher.publish(msg)

            if cv2.waitKey(1) == ord('q'):
                break

    node.destroy_node()
    rclpy.shutdown()
