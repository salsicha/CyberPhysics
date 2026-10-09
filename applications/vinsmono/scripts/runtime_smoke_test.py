#!/usr/bin/env python3
"""Run in the VINS image: sensor QoS, feature flow, clean worker shutdown, and
estimator output from the container's launch file on synthetic VIO data."""
import argparse
import math
import signal
import subprocess
import tempfile
import time
from contextlib import contextmanager
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu, PointCloud


@contextmanager
def running_node(package, config, stop_signal=signal.SIGINT, parameters=()):
    executable = Path(get_package_prefix(package)) / 'lib' / package / package
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen([
            str(executable), '--ros-args', '-r', f'__ns:=/{package}',
            '-p', f'config_file:={config}',
            *[arg for value in parameters for arg in ('-p', value)],
        ], stdin=subprocess.PIPE, stdout=log, stderr=log)
        try:
            yield process
        finally:
            if process.poll() is None:
                process.send_signal(stop_signal)
            try:
                code = process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                raise AssertionError(f'{package} did not stop within 10 seconds')
            finally:
                process.stdin.close()
            if code != 0:
                log.seek(0)
                raise AssertionError(f'{package} exited {code}:\n{log.read()}')


@contextmanager
def running_launch(config, *arguments):
    """The container CMD: ros2 launch vins_estimator vins_mono.launch.py."""
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'vins_estimator', 'vins_mono.launch.py',
             f'config_file:={config}', *arguments],
            stdin=subprocess.PIPE, stdout=log, stderr=log)
        try:
            yield process, log
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                code = process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                raise AssertionError('vins_mono.launch.py did not stop within 30 seconds')
            finally:
                process.stdin.close()
            log.seek(0)
            output = log.read()
            if code != 0 or 'process has died' in output:
                raise AssertionError(f'vins_mono.launch.py exited {code}:\n{output}')


def read_matrix(config, name, shape):
    storage = cv2.FileStorage(str(config), cv2.FILE_STORAGE_READ)
    try:
        node = storage.getNode(name)
        return np.asarray(node.mat(), dtype=float).reshape(shape)
    finally:
        storage.release()


class SyntheticVio:
    """Camera/IMU rig moving through textured patches at 3.5-9 m.

    Ground truth is analytic: world z up, IMU pose R(t) = R0 Exp(u theta(t)),
    p(t) sinusoidal, so the accelerometer reads R^T (a + g) and the gyro u theta'.
    The camera sits at the configured camera-to-IMU extrinsic.
    """

    def __init__(self, config, seed=7):
        storage = cv2.FileStorage(str(config), cv2.FILE_STORAGE_READ)
        projection = storage.getNode('projection_parameters')
        self.fx, self.fy = projection.getNode('fx').real(), projection.getNode('fy').real()
        self.cx, self.cy = projection.getNode('cx').real(), projection.getNode('cy').real()
        self.width = int(storage.getNode('image_width').real())
        self.height = int(storage.getNode('image_height').real())
        self.gravity = storage.getNode('g_norm').real()
        storage.release()
        self.r_ic = read_matrix(config, 'extrinsicRotation', (3, 3))
        self.t_ic = read_matrix(config, 'extrinsicTranslation', (3,))
        # Camera looks along world +x: optical z = +x, image right = -y, down = -z.
        r_wc = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
        self.r0 = r_wc @ self.r_ic.T
        self.axis = np.array([0.3, 1.0, 0.2]) / np.linalg.norm([0.3, 1.0, 0.2])
        self.amplitude = np.array([0.5, 0.3, 0.4])
        self.omega = 2.0 * math.pi * np.array([0.4, 0.55, 0.3])
        self.rot_amplitude, self.rot_omega = 0.12, 2.0 * math.pi * 0.35
        rng = np.random.default_rng(seed)
        count = 400
        centers = np.column_stack([rng.uniform(3.5, 9.0, count), rng.uniform(-5.0, 5.0, count),
                                   rng.uniform(-3.5, 3.5, count)])
        half = 0.07
        offsets = np.array([[0, -half, -half], [0, half, -half], [0, half, half], [0, -half, half]])
        self.patches = centers[:, None, :] + offsets[None, :, :]
        shades = rng.integers(0, 2, count)
        self.colors = np.where(shades, rng.integers(170, 256, count), rng.integers(0, 70, count))

    def _rotation(self, theta):
        k = np.array([[0.0, -self.axis[2], self.axis[1]], [self.axis[2], 0.0, -self.axis[0]],
                      [-self.axis[1], self.axis[0], 0.0]])
        return self.r0 @ (np.eye(3) + math.sin(theta) * k + (1 - math.cos(theta)) * k @ k)

    def pose(self, t):
        theta = self.rot_amplitude * math.sin(self.rot_omega * t)
        return self._rotation(theta), self.amplitude * np.sin(self.omega * t)

    def imu(self, t):
        r_wi, _ = self.pose(t)
        accel = -self.amplitude * self.omega ** 2 * np.sin(self.omega * t)
        specific_force = r_wi.T @ (accel + np.array([0.0, 0.0, self.gravity]))
        rate = self.axis * self.rot_amplitude * self.rot_omega * math.cos(self.rot_omega * t)
        return specific_force, rate

    def image(self, t):
        r_wi, p_wi = self.pose(t)
        r_wc = r_wi @ self.r_ic
        p_wc = p_wi + r_wi @ self.t_ic
        points = (self.patches - p_wc) @ r_wc  # world -> camera, per corner
        frame = np.full((self.height, self.width), 110, dtype=np.uint8)
        depth = points[:, :, 2]
        order = np.argsort(-depth.mean(axis=1))
        shift = 4
        for index in order:
            if depth[index].min() < 0.5:
                continue
            u = self.fx * points[index, :, 0] / depth[index] + self.cx
            v = self.fy * points[index, :, 1] / depth[index] + self.cy
            if u.max() < 0 or u.min() >= self.width or v.max() < 0 or v.min() >= self.height:
                continue
            corners = np.round(np.column_stack([u, v]) * (1 << shift)).astype(np.int32)
            cv2.fillConvexPoly(frame, corners, int(self.colors[index]), cv2.LINE_AA, shift)
        return frame


def check_estimator_output(node, config, image_pub, imu_pub):
    """Run the container's launch file on synthetic VIO and require odometry."""
    rig = SyntheticVio(config)
    odometry, graph_paths = [], []
    subscriptions = [
        node.create_subscription(Odometry, '/vins_estimator/odometry', odometry.append, 100),
        node.create_subscription(PathMsg, '/pose_graph/pose_graph_path', graph_paths.append, 10),
    ]
    image_stamps = set()
    result_csv = Path('/data/vinsmono/vins_result_no_loop.csv')
    try:
        with running_launch(config, 'enable_pose_graph:=true') as (launch, log):
            wait_until(node, lambda: (imu_pub.get_subscription_count() > 0
                                      and image_pub.get_subscription_count() >= 2),
                       [launch], timeout=30)
            start = node.get_clock().now().nanoseconds
            wall_start = time.monotonic()
            imu_period, image_period = 1.0 / 200.0, 1.0 / 20.0
            imu_index = image_index = 0
            deadline = wall_start + 45.0
            while time.monotonic() < deadline:
                assert launch.poll() is None, 'launch exited during sensor playback'
                if len(odometry) >= 40 and graph_paths and graph_paths[-1].poses:
                    break
                elapsed = time.monotonic() - wall_start
                while imu_index * imu_period <= elapsed:
                    t = imu_index * imu_period
                    force, rate = rig.imu(t)
                    imu = Imu()
                    imu.header.stamp = Time(nanoseconds=start + round(t * 1e9)).to_msg()
                    imu.linear_acceleration.x, imu.linear_acceleration.y, \
                        imu.linear_acceleration.z = (float(v) for v in force)
                    imu.angular_velocity.x, imu.angular_velocity.y, \
                        imu.angular_velocity.z = (float(v) for v in rate)
                    imu_pub.publish(imu)
                    imu_index += 1
                if image_index * image_period <= elapsed:
                    t = image_index * image_period
                    frame = rig.image(t)
                    stamp_ns = start + round(t * 1e9)
                    image = Image(height=frame.shape[0], width=frame.shape[1],
                                  encoding='mono8', step=frame.shape[1], data=frame.tobytes())
                    image.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
                    image.header.frame_id = 'camera'
                    image_pub.publish(image)
                    image_stamps.add(stamp_ns)
                    image_index += 1
                rclpy.spin_once(node, timeout_sec=0.001)

            def stamp_ns(stamp):
                return stamp.sec * 1_000_000_000 + stamp.nanosec

            assert len(odometry) >= 40, f'estimator published {len(odometry)} odometry messages'
            for msg in odometry:
                assert stamp_ns(msg.header.stamp) in image_stamps, 'odometry stamp is not an image stamp'
                pose = msg.pose.pose
                values = (pose.position.x, pose.position.y, pose.position.z, pose.orientation.w,
                          pose.orientation.x, pose.orientation.y, pose.orientation.z)
                assert all(math.isfinite(v) for v in values), f'non-finite odometry {values}'
                assert math.dist((0, 0, 0), values[:3]) < 50.0, f'odometry diverged: {values[:3]}'
            print(f'PASS estimator odometry ({len(odometry)} messages)', flush=True)

            assert graph_paths and graph_paths[-1].poses, 'pose graph published no keyframe path'
            ordered = sorted(image_stamps)
            for pose in graph_paths[-1].poses:
                # Keyframe times are stored as double seconds (~0.2 us resolution).
                nearest = min(ordered, key=lambda s: abs(s - stamp_ns(pose.header.stamp)))
                assert abs(nearest - stamp_ns(pose.header.stamp)) < 1000, (
                    f'pose graph stamp {pose.header.stamp} is not an image stamp')
            print(f'PASS pose graph path stamps ({len(graph_paths[-1].poses)} keyframes)', flush=True)

            rows = [line.split(',') for line in result_csv.read_text().splitlines() if line]
            assert rows, f'{result_csv} is empty'
            assert all(int(row[0]) in image_stamps for row in rows), (
                'trajectory CSV stamps are not image stamps in integer nanoseconds')
            print(f'PASS trajectory CSV ({len(rows)} rows, nanosecond stamps)', flush=True)
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)


def wait_until(node, predicate, processes, timeout=15):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert all(process.poll() is None for process in processes), 'VINS exited before readiness'
        assert time.monotonic() < deadline, 'Timed out waiting for VINS sensor connections'
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('vins_runtime_smoke_test')
    image_pub = node.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
    imu_pub = node.create_publisher(Imu, '/camera/imu', qos_profile_sensor_data)
    features = []
    subscription = node.create_subscription(
        PointCloud, '/feature_tracker/feature', lambda msg: features.append(msg), 10)
    try:
        for stop_signal in (signal.SIGINT, signal.SIGTERM):
            with running_node('vins_estimator', args.config, stop_signal) as estimator:
                wait_until(node, lambda: imu_pub.get_subscription_count() > 0, [estimator])
            wait_until(node, lambda: imu_pub.get_subscription_count() == 0, [])
            print(f'PASS idle estimator shutdown on {stop_signal.name}', flush=True)

        for enabled in (False, True):
            for stop_signal in (signal.SIGINT, signal.SIGTERM):
                parameters = (
                    f'enable_loop_closure:={str(enabled).lower()}',
                    'support_file:=/workspace/install/share/config_pkg/support_files',
                )
                with running_node('pose_graph', args.config, stop_signal, parameters) as graph:
                    wait_until(node, lambda: node.count_publishers('/pose_graph/match_points') > 0,
                               [graph], timeout=30)
                wait_until(node, lambda: node.count_publishers('/pose_graph/match_points') == 0, [])
                print(f'PASS pose graph loop_closure={enabled} shutdown on {stop_signal.name}', flush=True)

        with running_node('vins_estimator', args.config) as estimator:
            with running_node('feature_tracker', args.config) as tracker:
                wait_until(node, lambda: (imu_pub.get_subscription_count() > 0
                                         and image_pub.get_subscription_count() > 0),
                           [estimator, tracker])
                print('PASS best-effort camera and IMU connections', flush=True)
                image = Image(height=480, width=640, encoding='mono8', step=640)
                image.data = bytes(255 if ((x // 20) + (y // 20)) % 2 else 0
                                   for y in range(480) for x in range(640))
                deadline = time.monotonic() + 5
                next_image = 0.0
                while time.monotonic() < deadline:
                    stamp = node.get_clock().now().to_msg()
                    imu = Imu()
                    imu.header.stamp = stamp
                    imu.linear_acceleration.z = 9.81
                    imu_pub.publish(imu)
                    if time.monotonic() >= next_image:
                        image.header.stamp = stamp
                        image_pub.publish(image)
                        next_image = time.monotonic() + 0.05
                    rclpy.spin_once(node, timeout_sec=0.005)
                assert any(msg.points for msg in features), 'No tracked features received'
                assert estimator.poll() is None and tracker.poll() is None
        print('PASS feature flow and estimator shutdown after sensor processing', flush=True)

        check_estimator_output(node, args.config, image_pub, imu_pub)
        print('PASS launch file shutdown after estimator output', flush=True)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
