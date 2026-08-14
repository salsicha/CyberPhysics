#!/usr/bin/env python3
"""Genesis terrain, drone physics, rendered sensors, and ArduPilot backend."""

from __future__ import annotations

import argparse
import faulthandler
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, "/workspace/applications/ardupilot_sitl/scripts")
from ardupilot_json import (  # noqa: E402
    ArduPilotJSON,
    pwm_to_rotor_speed,
    quaternion_to_matrix,
    state_from_enu,
)


def env_float(name, default):
    return float(os.environ.get(name, default))


def env_int(name, default):
    return int(os.environ.get(name, default))


def to_numpy(value):
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value, dtype=np.float64)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", default=os.environ.get("GENESIS_BACKEND", "cpu"),
                        choices=["cpu", "gpu"])
    parser.add_argument("--lat", type=float, default=env_float("INITIAL_LAT", 37.9234))
    parser.add_argument("--lon", type=float, default=env_float("INITIAL_LON", -122.5967))
    parser.add_argument("--origin-alt", type=float, default=env_float("ORIGIN_ALT", 781.0))
    parser.add_argument("--assets", default="/data/navsim_assets")
    parser.add_argument("--rate-hz", type=float, default=env_float("GENESIS_RATE_HZ", 1200.0))
    parser.add_argument("--camera-rate-hz", type=float,
                        default=env_float("GENESIS_CAMERA_RATE_HZ", 10.0))
    parser.add_argument("--duration-s", type=float, default=env_float("GENESIS_DURATION_S", 0.0))
    parser.add_argument("--width", type=int, default=env_int("GENESIS_CAMERA_WIDTH", 640))
    parser.add_argument("--height", type=int, default=env_int("GENESIS_CAMERA_HEIGHT", 480))
    parser.add_argument("--fov-deg", type=float, default=env_float("FOV_DEG", 60.0))
    parser.add_argument("--rgb-topic", default=os.environ.get(
        "WILDNAV_IMAGE_TOPIC", "/aerodrone/sensor_measurements/downward_rgb/image_raw"))
    parser.add_argument("--rgb-info-topic", default=os.environ.get(
        "WILDNAV_CAMERA_INFO_TOPIC", "/aerodrone/sensor_measurements/downward_rgb/camera_info"))
    parser.add_argument("--depth-topic", default=os.environ.get(
        "DEPTH_TOPIC", "/aerodrone/sensor_measurements/downward_rgbd/depth"))
    parser.add_argument("--depth-info-topic", default=os.environ.get(
        "CAMERA_INFO_TOPIC", "/aerodrone/sensor_measurements/downward_rgbd/depth/camera_info"))
    return parser.parse_args()


class GenesisSensorPublisher(Node):
    def __init__(self, args):
        super().__init__("genesis_sim_sensors")
        self.args = args
        self.truth_pub = self.create_publisher(
            Odometry, "/aerodrone/simulation/ground_truth/odom", 10)
        self.rgb_pub = self.create_publisher(Image, args.rgb_topic, 10)
        self.rgb_info_pub = self.create_publisher(CameraInfo, args.rgb_info_topic, 10)
        self.depth_pub = self.create_publisher(Image, args.depth_topic, 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, args.depth_info_topic, 10)

    def camera_info(self, stamp, frame):
        width, height = self.args.width, self.args.height
        fx = width / (2.0 * math.tan(math.radians(self.args.fov_deg) / 2.0))
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = frame
        msg.width, msg.height = width, height
        msg.distortion_model = "plumb_bob"
        msg.k = [fx, 0.0, width / 2.0, 0.0, fx, height / 2.0, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, width / 2.0, 0.0, 0.0, fx, height / 2.0,
                 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def publish_truth(self, position, quaternion, velocity, angular):
        stamp = self.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "earth"
        msg.child_frame_id = "aerodrone/base_link"
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = position
        msg.pose.pose.orientation.w, msg.pose.pose.orientation.x = quaternion[:2]
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z = quaternion[2:]
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = velocity
        msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = angular
        self.truth_pub.publish(msg)

    def publish_camera(self, rgb, depth):
        stamp = self.get_clock().now().to_msg()
        rgb = np.asarray(rgb)
        if rgb.shape[-1] == 4:
            rgb = rgb[..., :3]
        if rgb.dtype != np.uint8:
            scale = 255.0 if float(np.nanmax(rgb)) <= 1.0 else 1.0
            rgb = np.clip(rgb * scale, 0, 255).astype(np.uint8)
        rgb = np.ascontiguousarray(rgb)
        depth = np.ascontiguousarray(np.asarray(depth, dtype=np.float32))
        frame = "downward_camera_optical_frame"
        rgb_msg = Image()
        rgb_msg.header.stamp, rgb_msg.header.frame_id = stamp, frame
        rgb_msg.height, rgb_msg.width = rgb.shape[:2]
        rgb_msg.encoding, rgb_msg.step = "rgb8", rgb.shape[1] * 3
        rgb_msg.data = rgb.tobytes()
        self.rgb_pub.publish(rgb_msg)
        self.rgb_info_pub.publish(self.camera_info(stamp, frame))
        depth_msg = Image()
        depth_msg.header.stamp, depth_msg.header.frame_id = stamp, frame
        depth_msg.height, depth_msg.width = depth.shape[:2]
        depth_msg.encoding, depth_msg.step = "32FC1", depth.shape[1] * 4
        depth_msg.data = depth.tobytes()
        self.depth_pub.publish(depth_msg)
        self.depth_info_pub.publish(self.camera_info(stamp, frame))


def main():
    args = parse_args()
    import genesis as gs

    if os.environ.get("GENESIS_TRACE_STARTUP", "0") == "1":
        faulthandler.enable()
        faulthandler.dump_traceback_later(60, repeat=True)

    assets = Path(args.assets)
    manifest = json.loads((assets / "manifest.json").read_text(encoding="utf-8"))
    home = np.asarray(manifest["home_enu_m"], dtype=np.float64)
    backend = gs.cpu if args.backend == "cpu" else gs.gpu
    gs.init(
        backend=backend,
        logging_level=os.environ.get("GENESIS_LOG_LEVEL", "info"))
    scene = gs.Scene(
        # A sub-millisecond rigid step matches ArduPilot's 1200 Hz JSON loop.
        # Genesis 1.3.3's multi-substep CUDA path can stall on this terrain,
        # so lock-step exchanges one stable physics step per actuator frame.
        sim_options=gs.options.SimOptions(
            dt=1.0 / args.rate_hz, gravity=(0, 0, -9.80665)),
        show_viewer=False,
    )
    mesh_data = np.load(assets / "terrain_mesh.npz")
    mesh_rows = int(manifest["terrain"]["mesh_rows"])
    mesh_cols = int(manifest["terrain"]["mesh_cols"])
    mesh_heights = mesh_data["vertices"][:, 2].reshape(mesh_rows, mesh_cols)
    # OBJ rows run north-to-south and columns west-to-east. Genesis Terrain
    # indexes x then y from its origin, so transpose and reverse north.
    # The 80 m native heightfield matches Gazebo collision resolution.
    height_field = mesh_heights[::-2, ::2].T
    half_m = float(manifest["terrain"]["area_km"]) * 500.0
    horizontal_scale = 2.0 * half_m / (height_field.shape[0] - 1)
    terrain_surface = gs.surfaces.Default(
        diffuse_texture=gs.textures.ImageTexture(
            image_path=str(assets / "orthophoto.jpg")))
    scene.add_entity(gs.morphs.Terrain(
        height_field=height_field, horizontal_scale=horizontal_scale,
        vertical_scale=1.0, pos=(-half_m, -half_m, 0.0),
        visualization=True, collision=True), surface=terrain_surface)
    # The common home is the top of Gazebo's conservative DEM collision cell.
    # Put a small real collision pad at that same surveyed elevation so the
    # lightweight drone starts at rest instead of falling onto a steep,
    # interpolated heightfield triangle while ArduPilot initializes.
    scene.add_entity(gs.morphs.Box(
        pos=(float(home[0]), float(home[1]), float(home[2] - 0.12)),
        size=(3.0, 3.0, 0.2), fixed=True))
    drone = scene.add_entity(gs.morphs.Drone(
        file="urdf/drones/cf2x.urdf", pos=tuple(home.tolist())))
    camera = scene.add_camera(
        res=(args.width, args.height), pos=tuple(home.tolist()),
        lookat=(home[0], home[1], home[2] - 10.0), up=(0.0, 1.0, 0.0),
        fov=args.fov_deg, near=0.3, far=300.0, GUI=False)
    print("Genesis scene build starting", flush=True)
    scene.build()
    print("Genesis scene build complete", flush=True)
    faulthandler.cancel_dump_traceback_later()

    rclpy.init()
    publisher = GenesisSensorPublisher(args)
    transport = ArduPilotJSON(port=int(os.environ.get("ARDUPILOT_JSON_PORT", "9002")))
    Path("/tmp/genesis_sim_ready").touch()
    dt = 1.0 / args.rate_hz
    camera_stride = max(1, int(round(args.rate_hz / args.camera_rate_hz)))
    maximum_rotor_rpm = env_float("GENESIS_MAX_ROTOR_RPM", 21700.0)
    previous_velocity = np.zeros(3)
    tick = 0
    start = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(publisher, timeout_sec=0.0)
            frame = transport.receive()
            if frame is None:
                time.sleep(dt)
                continue
            rpm = pwm_to_rotor_speed(frame.pwm[:4], maximum_speed=maximum_rotor_rpm)
            if hasattr(drone, "set_propellers_rpm"):
                drone.set_propellers_rpm(rpm.tolist())
            else:
                drone.set_propellels_rpm(rpm.tolist())
            scene.step()

            position = to_numpy(drone.get_pos())[:3]
            quaternion = to_numpy(drone.get_quat())[:4]
            velocity = to_numpy(drone.get_vel())[:3]
            angular = to_numpy(drone.get_ang())[:3]
            acceleration_world = (velocity - previous_velocity) / dt
            previous_velocity = velocity.copy()
            specific_force_flu = quaternion_to_matrix(quaternion).T @ (
                acceleration_world - np.array([0.0, 0.0, -9.80665]))
            transport.send(state_from_enu(
                position, velocity, quaternion, angular, specific_force_flu,
                args.lat, args.lon, args.origin_alt, timestamp=tick * dt,
                reference_enu=home))

            if tick % camera_stride == 0:
                camera.set_pose(
                    pos=position.tolist(),
                    lookat=(position + np.array([0.0, 0.0, -10.0])).tolist(),
                    up=(0.0, 1.0, 0.0))
                rendered = camera.render(rgb=True, depth=True)
                publisher.publish_camera(rendered[0], rendered[1])
                publisher.publish_truth(position.tolist(), quaternion.tolist(),
                                        velocity.tolist(), angular.tolist())
            if tick % max(1, int(args.rate_hz * 5.0)) == 0:
                print(
                    "Genesis physics "
                    f"frame_rate={frame.frame_rate_hz}Hz "
                    f"pwm={list(frame.pwm[:4])} "
                    f"position={position.round(3).tolist()}",
                    flush=True,
                )
            tick += 1
            if args.duration_s > 0 and time.monotonic() - start >= args.duration_s:
                break
    except KeyboardInterrupt:
        pass
    finally:
        transport.close()
        publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
