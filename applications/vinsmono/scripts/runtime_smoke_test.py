#!/usr/bin/env python3
"""Run in the VINS image: sensor QoS, feature flow, and clean worker shutdown."""
import argparse
import signal
import subprocess
import tempfile
import time
from contextlib import contextmanager
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud


@contextmanager
def running_node(package, config, stop_signal=signal.SIGINT):
    executable = Path(get_package_prefix(package)) / 'lib' / package / package
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen([
            str(executable), '--ros-args', '-r', f'__ns:=/{package}',
            '-p', f'config_file:={config}',
        ], stdout=log, stderr=log)
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
            if code != 0:
                log.seek(0)
                raise AssertionError(f'{package} exited {code}:\n{log.read()}')


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
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
