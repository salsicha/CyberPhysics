#!/usr/bin/env python3

import cv2
import depthai as dai
from collections import deque

import time


show_window = True


class FeatureTrackerDrawer:

    lineColor = (200, 0, 200)
    pointColor = (0, 0, 255)
    circleRadius = 2
    maxTrackedFeaturesPathLength = 30
    # for how many frames the feature is tracked
    trackedFeaturesPathLength = 10

    trackedIDs = None
    trackedFeaturesPath = None

    def onTrackBar(self, val):
        FeatureTrackerDrawer.trackedFeaturesPathLength = val
        pass

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

    def drawFeatures(self, img):

        cv2.setTrackbarPos(self.trackbarName, self.windowName, FeatureTrackerDrawer.trackedFeaturesPathLength)

        for featurePath in self.trackedFeaturesPath.values():
            path = featurePath

            for j in range(len(path) - 1):
                src = (int(path[j].x), int(path[j].y))
                dst = (int(path[j + 1].x), int(path[j + 1].y))
                cv2.line(img, src, dst, self.lineColor, 1, cv2.LINE_AA, 0)
            j = len(path) - 1
            cv2.circle(img, (int(path[j].x), int(path[j].y)), self.circleRadius, self.pointColor, -1, cv2.LINE_AA, 0)

    def __init__(self, trackbarName, windowName):
        self.trackbarName = trackbarName
        self.windowName = windowName

        if show_window:
            cv2.namedWindow(windowName)
            cv2.createTrackbar(trackbarName, windowName, FeatureTrackerDrawer.trackedFeaturesPathLength, FeatureTrackerDrawer.maxTrackedFeaturesPathLength, self.onTrackBar)

        self.trackedIDs = set()
        self.trackedFeaturesPath = dict()


# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
colorCam = pipeline.create(dai.node.ColorCamera)
# colorCam = pipeline.create(dai.node.MonoCamera)
featureTrackerColor = pipeline.create(dai.node.FeatureTracker)


## Script
script_node = pipeline.create(dai.node.Script)
script_node.setScript("""
    import marshal

    input_data = node.io['input'].get()
                      
    num = 123
    node.warn(f"Number {num}")

    output_data = [1, "hello", {"foo": "bar"}]
    out_serial = marshal.dumps(output_data)
    buf = Buffer(len(out_serial))
    while True:
        node.io['output'].send(b)
""")
input_node = pipeline.create(dai.node.XLinkIn)
input_node.setStreamName("input")
output_node = pipeline.create(dai.node.XLinkOut)
output_node.setStreamName("output")
input_node.out.link(script_node.inputs["input"])
script_node.outputs["output"].link(output_node.input)



xoutPassthroughFrameColor = pipeline.create(dai.node.XLinkOut)
xoutTrackedFeaturesColor = pipeline.create(dai.node.XLinkOut)
xinTrackedFeaturesConfig = pipeline.create(dai.node.XLinkIn)

xoutPassthroughFrameColor.setStreamName("passthroughFrameColor")
xoutTrackedFeaturesColor.setStreamName("trackedFeaturesColor")
xinTrackedFeaturesConfig.setStreamName("trackedFeaturesConfig")

# Properties
colorCam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

if 1:
    colorCam.setIspScale(2,3)
    colorCam.video.link(featureTrackerColor.inputImage)
else:
    colorCam.isp.link(featureTrackerColor.inputImage)

# Linking
featureTrackerColor.passthroughInputImage.link(xoutPassthroughFrameColor.input)
featureTrackerColor.outputFeatures.link(xoutTrackedFeaturesColor.input)
xinTrackedFeaturesConfig.out.link(featureTrackerColor.inputConfig)

# By default the least mount of resources are allocated
# increasing it improves performance
numShaves = 2
numMemorySlices = 2
featureTrackerColor.setHardwareResources(numShaves, numMemorySlices)
featureTrackerConfig = featureTrackerColor.initialConfig.get()

# print("Press 's' to switch between Lucas-Kanade optical flow and hardware accelerated motion estimation!")

# Connect to device and start pipeline
with dai.Device(pipeline) as device:


    ## Script
    input_queue = device.getInputQueue("input")
    output_queue = device.getOutputQueue("output", 8, False)


    # Output queues used to receive the results
    passthroughImageColorQueue = device.getOutputQueue("passthroughFrameColor", 8, False)
    outputFeaturesColorQueue = device.getOutputQueue("trackedFeaturesColor", 8, False)

    inputFeatureTrackerConfigQueue = device.getInputQueue("trackedFeaturesConfig")

    colorWindowName = "color"
    colorFeatureDrawer = FeatureTrackerDrawer("Feature tracking duration (frames)", colorWindowName)

    featureTrackerConfig.motionEstimator.type = dai.FeatureTrackerConfig.MotionEstimator.Type.HW_MOTION_ESTIMATION
    cfg = dai.FeatureTrackerConfig()
    cfg.set(featureTrackerConfig)
    inputFeatureTrackerConfigQueue.send(cfg)


    while True:


        ## Script
        # input_data = 1
        # input_queue.send(input_data)
        # output_data = output_queue.get()
        # print(f"output data: {output_data}")




        inPassthroughFrameColor = passthroughImageColorQueue.get()
        passthroughFrameColor = inPassthroughFrameColor.getCvFrame()

        trackedFeaturesColor = outputFeaturesColorQueue.get().trackedFeatures
        colorFeatureDrawer.trackFeaturePath(trackedFeaturesColor)



        ## TODO: calculate velocity and publish

        try:
            keys = list(colorFeatureDrawer.trackedFeaturesPath.keys())
            # print(f"keys: {keys}")
            feature_queue = colorFeatureDrawer.trackedFeaturesPath[keys[0]]

            # first_feature = feature_queue[-2]
            # print(f"Features: {first_feature.x}, {first_feature.y} \n")

            # first_feature = feature_queue[-1]
            # print(f"Features: {first_feature.x}, {first_feature.y} \n")

            frame_one = []
            frame_two = []
            for key in keys:
                feature_queue = colorFeatureDrawer.trackedFeaturesPath[key]
                

        except:
            pass

        ## TODO: Calculate optical flow
        ## prev_gray and gray are just the grayscale image
        # new_pts, status, errors = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, None, **lk_params)
        ## Select good points
        # good_new = new_pts[status == 1]
        # good_old = prev_pts[status == 1]
        # H, _ = cv2.findHomography(good_old, good_new, cv2.RANSAC, 5.0)
        ## Get translation out of homography "H"
        # rotation_matrix, translation_vector, camera_normal = cv2.decomposeHomographyMat(H)

        # print(f"Translation: {translation_vector}"")


        # TODO: Publish

        # outputFeaturesColorQueue.get().trackedFeatures
        # TrackedFeatures message. Carries position (X, Y) of tracked features and their ID.
        # from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
        # from rospy.numpy_msg import numpy_msg
        # Which msg type??


        if show_window:
            colorFrame = passthroughFrameColor

            colorFeatureDrawer.drawFeatures(colorFrame)

            # Show the frame
            cv2.imshow(colorWindowName, colorFrame)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break

            # elif key == ord('s'):
            #     if featureTrackerConfig.motionEstimator.type == dai.FeatureTrackerConfig.MotionEstimator.Type.LUCAS_KANADE_OPTICAL_FLOW:
            #         featureTrackerConfig.motionEstimator.type = dai.FeatureTrackerConfig.MotionEstimator.Type.HW_MOTION_ESTIMATION
            #         print("Switching to hardware accelerated motion estimation")
            #     else:
            #         featureTrackerConfig.motionEstimator.type = dai.FeatureTrackerConfig.MotionEstimator.Type.LUCAS_KANADE_OPTICAL_FLOW
            #         print("Switching to Lucas-Kanade optical flow")
            #     cfg = dai.FeatureTrackerConfig()
            #     cfg.set(featureTrackerConfig)
            #     inputFeatureTrackerConfigQueue.send(cfg)

        else:
            time.sleep(0.032)
