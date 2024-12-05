#!/usr/bin/env python3

"""
This demo reads from a rosbag and plots the angular velocity from the Blackfly camera and Novatel INS.
To collect data for the demo mount the camera on top of the INS.
Rotate both units left and right for a few seconds in a sinusoidal motion and save to a rosbag.

The following topics need to be present in the bag file:
    - /novatel/oem7/mark2time (recorded at the start of a camera capture)
    - /imu/data_raw
    - /blackfly_front/image_raw
    - /blackfly_front/camera_info (needs to have valid calibration data)
"""

import argparse
import cv2
import rosbag
import scipy
import matplotlib.pyplot as plt
import numpy as np

from tqdm import tqdm

def find_rotation_between_images(prev_gray, cur_gray, K, D, is_fisheye):
    """
    Find rotation between two images using epipolar geometry

    Parameters
    ----------
        prev_gray: grayscale image
        cur_gray: grayscale image
        K: 3x3 camera matrix
        D: distortion coefficients
        is_fisheye: flag to indicate camera model is a fisheye or not

    Returns:
        R: 3x3 rotation matrix
        good_prev: inlier points
        good_cur: inliter points
    """

    p0 = cv2.goodFeaturesToTrack(prev_gray, maxCorners=1000, qualityLevel=0.01, minDistance=7, blockSize=7)

    p1, st, err = cv2.calcOpticalFlowPyrLK(
        prev_gray,
        cur_gray,
        p0,
        None,
        winSize=(15, 15),
        maxLevel=2,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    # Select good points
    if p1 is not None:
        good_cur = p1[st==1]
        good_prev = p0[st==1]
    else:
        return None

    if len(good_prev) == 0:
        return None

    if is_fisheye:
        good_prev_norm = cv2.fisheye.undistortPoints(np.expand_dims(good_prev, axis=1), K, D)
        good_cur_norm = cv2.fisheye.undistortPoints(np.expand_dims(good_cur, axis=1), K, D)
    else:
        good_prev_norm = cv2.undistortPoints(np.expand_dims(good_prev, axis=1), K, D)
        good_cur_norm = cv2.undistortPoints(np.expand_dims(good_cur, axis=1), K, D)

    # find essential matrix
    E, mask = cv2.findFundamentalMat(good_prev_norm, good_cur_norm, cv2.FM_RANSAC, 0.01)
    idx, _ = np.where(mask == 1)

    good_cur = good_cur[idx, :]
    good_prev = good_prev[idx, :]

    num_inliers, R, _, _ = cv2.recoverPose(E, good_prev, good_cur, np.eye(3, 3))

    if num_inliers < 10:
        return None

    return R, good_prev, good_cur


def debayer_img(msg):
    """
    Debayer the raw image to RGB

    Parameters
    ----------
        msg: ROS sensor_msgs/Image message

    Returns:
        buf: RGB image
    """

    buf = np.frombuffer(msg.data, dtype=np.uint8)
    buf = buf.reshape(msg.height, msg.width, 1)
    buf = cv2.cvtColor(buf, cv2.COLOR_BAYER_RG2RGB)

    return buf


def find_quadratic_peak(y0, y1, y2):
    """
    Find peak of 3 points by fitting a quadratic.

    The x values are implicitly [-1, 0, 1]

    Parameters
    ----------
        y0: scalar
        y1: scalar
        y2: scalar

    Returns:
        x: peak location, scalar between [-1, 1]
    """

    # make sure input is quadratic looking
    assert abs(y1) > abs(y0)
    assert abs(y1) > abs(y2)

    # solve for ax^2 + bx + c = y
    M = np.array([
        [1, -1, 1],
        [0, 0, 1],
        [1, 1, 1]])

    coeff = np.linalg.inv(M) @ np.array([y0, y1, y2])

    a = coeff[0]
    b = coeff[1]
    c = coeff[2]

    # find peak x
    x = -b / (2*a)

    return x


def find_shift(A, B):
    """
    Find the time shift between two signals A and B using cross correlation

    Parameters
    ----------
        A: 1D signal
        B: 1D signal

    Returns:
        shift: shift between A and B in array index units
    """

    assert len(A) == len(B)

    # normalize data
    a = A - A.mean()
    a /= A.std()

    b = B - B.mean()
    b /= B.std()

    xcorr = scipy.signal.correlate(a, b)

    x0 = xcorr.argmax() - 1
    x1 = xcorr.argmax()
    x2 = xcorr.argmax() + 1

    shift = 0

    if x0 >= 0 and x2 < len(xcorr):
        shift = -len(A) + x1

        # refine the time shift estimate
        y0 = xcorr[x0]
        y1 = xcorr[x1]
        y2 = xcorr[x2]

        delta = find_quadratic_peak(y0, y1, y2)
        shift += delta

    return shift


def display_results(img_t, img_omega, imu_t, imu_omega, time_shift, message_count, total_message, dbg_img):
    plt.clf()
    plt.subplot(121)
    plt.plot(img_t, img_omega, label="Camera Y-axis")
    plt.plot(imu_t, imu_omega, label="IMU Z-axis")
    plt.legend(loc="upper left")
    plt.xlabel("Time (seconds)")
    plt.ylabel("$\\omega$ (radians/seconds)")

    title = "Camera and IMU sync demo\nComparing angular velocities\n"
    if time_shift is not None:
        title += f"Average time shift: {time_shift*1000:.2f} ms"

    per = float(message_count) / total_message
    progress = f"\nROS messages read: {message_count}/{total_message} ({per*100:.2f}%)"

    plt.title(title + progress)
    if dbg_img is not None:
        plt.subplot(122)
        plt.imshow(dbg_img)
    plt.pause(0.000001)


def run_demo(bagfile):
    # limit image size to speed up processing
    MAX_IMAGE_WIDTH = 1024

    bag = rosbag.Bag(bagfile)
    assert bag

    mark2time_history = []
    ref_time = None
    prev_gray = None
    prev_img_t = None
    cur_mark2time_imu = None
    cur_imu = None
    dbg_img = None
    time_shift = None

    # running average of time delta between mark2time
    # this should be 1/frame rate of the camera
    avg_mark2time_timestep = None
    prev_mark2time_time = None

    # angular velocity from IMU
    imu_t = []
    imu_omega = []

    # angular velocity from camera
    img_t = []
    img_omega = []

    # camera model
    K = None # camera matrix
    D = None # camera distortion
    is_fisheye = False
    K_rescaled = False

    for count, (topic, msg, t) in tqdm(enumerate(bag.read_messages()), total=bag.get_message_count()):
        if ref_time is None:
            ref_time = t.to_sec()

        t = t.to_sec() - ref_time

        # we got data from topics arriving at different ROS time
        # the "sync" strategy is very simple
        #   - each image_raw is matched to the previous mark2time
        #   - each mark2time is matched to the previous imu data

        if topic == "/novatel/oem7/mark2time":
            mark2time_history.append(t)

            if prev_mark2time_time is not None:
                if avg_mark2time_timestep is not None:
                    avg_mark2time_timestep = 0.7*avg_mark2time_timestep + 0.3*(t - prev_mark2time_time)
                else:
                    avg_mark2time_timestep = t - prev_mark2time_time

            prev_mark2time_time = t

            if cur_imu is not None:
                cur_mark2time_imu = cur_imu
        elif topic == "/imu/data_raw":
            cur_imu = (t, msg)
        elif topic == "/blackfly_front/camera_info":
            if K is None:
                K = np.array(msg.K).reshape(3, 3)
                assert K[0, 0] > 0, "No camera calibration found in ROS bag"

                D = np.array(msg.D)
                is_fisheye = msg.distortion_model == "equidistant"
        elif topic == "/blackfly_front/image_raw":
            buf = debayer_img(msg)

            h, w, _ = buf.shape
            if w > MAX_IMAGE_WIDTH:
                scale = float(MAX_IMAGE_WIDTH) / w

                buf = cv2.resize(buf, (int(w*scale), int(h*scale)))

                if not K_rescaled:
                    K[0, 0] *= scale
                    K[1, 1] *= scale
                    K[0, 2] *= scale
                    K[1, 2] *= scale
                    K_rescaled = True

            cur_gray = cv2.cvtColor(buf, cv2.COLOR_RGB2GRAY)

            if len(mark2time_history) >= 1:
                cur_img_t = mark2time_history[0]
                mark2time_history.pop(0)
            else:
                cur_img_t = None

            # wait for at least 2 images before processing
            if all(item is not None for item in [prev_img_t, cur_img_t, prev_gray, cur_mark2time_imu]):
                ret = find_rotation_between_images(prev_gray, cur_gray, K, D, is_fisheye)

                if ret is not None:
                    R, good_old, good_new = ret

                    # derive angular velocities from rotation using linear approximation
                    skew = scipy.linalg.logm(R)
                    wx = skew[2, 1] / avg_mark2time_timestep
                    wy = skew[0, 2] / avg_mark2time_timestep
                    wz = skew[1, 0] / avg_mark2time_timestep

                    img_t.append(cur_img_t)
                    img_omega.append(-wy)

                    imu_t.append(cur_mark2time_imu[0])
                    imu_omega.append(cur_mark2time_imu[1].angular_velocity.z)

                    dbg_img = cv2.cvtColor(cur_gray, cv2.COLOR_GRAY2RGB)

                    # draw the tracks
                    for new, old in zip(good_new, good_old):
                        a, b = new.ravel()
                        c, d = old.ravel()
                        dbg_img = cv2.line(dbg_img, (int(a), int(b)), (int(c), int(d)), (255, 0, 0), 1)
                        dbg_img = cv2.circle(dbg_img, (int(a), int(b)), 3, (255, 0, 0), -1)
                else:
                    print("Can't find rotation between images!")

            prev_gray = cur_gray
            prev_img_t = cur_img_t

            if len(img_omega) >= 2:
                shift = find_shift(np.array(img_omega), np.array(imu_omega))
                time_shift = shift * avg_mark2time_timestep

            display_results(img_t, img_omega, imu_t, imu_omega, time_shift, count + 1, bag.get_message_count(), dbg_img)

    display_results(img_t, img_omega, imu_t, imu_omega, time_shift, bag.get_message_count(), bag.get_message_count(), dbg_img)

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("bagfile", help="ROS bag file containing topics [/novatel/oem7/mark2time, /imu/data_raw, blackfly_front/image_raw]", type=str)

    args = parser.parse_args()

    run_demo(args.bagfile)