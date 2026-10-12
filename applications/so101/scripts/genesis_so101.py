#!/usr/bin/env python3
"""Run the SO-101 in Genesis with the common ROS 2 command interface.

With a scenario (the default, systems/so101/scenarios/picking_table.json) the
scene is built from it, as in Isaac and Gazebo: the arm at robot_spawn, the
table, riser, bins, occluder and calibration board as fixed boxes, the
objects as free bodies resting on the table, and the scenario camera
rendered to the common camera topics.
"""

import argparse
import array
import json
import math
import os
from pathlib import Path
import time

import genesis as gs
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float64MultiArray

from so101_common import (
    CAMERA_FRAME_ID,
    DEPTH_CAMERA_INFO_TOPIC,
    DEPTH_FRAME_ID,
    DEPTH_TOPIC,
    JOINT_NAMES,
    RGB_CAMERA_INFO_TOPIC,
    RGB_TOPIC,
    camera_info_values,
    quaternion_from_euler,
    rpy_to_matrix,
    scenario_camera,
    scene_color,
    scene_support_errors,
    validated_joint_command,
)

DEFAULT_SCENARIO = "/workspace/systems/so101/scenarios/picking_table.json"


def to_numpy(values):
    """Convert Genesis DOF readings to a host float64 array.

    On the gpu backend Genesis returns CUDA torch tensors, which np.asarray
    cannot convert, so copy tensors to the host first.
    """
    if hasattr(values, "detach"):
        values = values.detach().cpu().numpy()
    return np.asarray(values, dtype=np.float64).reshape(-1)


def genesis_quat(rpy):
    """Genesis (w, x, y, z) quaternion of a URDF roll, pitch, yaw."""
    qx, qy, qz, qw = quaternion_from_euler(*(float(value) for value in rpy[:3]))
    return (qw, qx, qy, qz)


def vertical_fov_deg(horizontal_fov_deg, width, height):
    """Genesis cameras take the vertical field of view; pixels are square."""
    half = math.tan(math.radians(float(horizontal_fov_deg)) / 2.0) * float(height) / float(width)
    return math.degrees(2.0 * math.atan(half))


def camera_look(camera):
    """(pos, lookat, up) of a scenario camera: groot_camera_link looks along
    its +X with +Z up."""
    rotation = rpy_to_matrix(camera["rpy"])
    position = np.asarray(camera["xyz"][:3], dtype=np.float64)
    return (tuple(position.tolist()), tuple((position + rotation[:, 0]).tolist()),
            tuple(rotation[:, 2].tolist()))


def add_scenario(scene, scenario):
    """Add the scenario's assets (fixed) and objects (free); {id: entity} of the objects."""
    for asset in scenario.get("static_assets", []):
        scene.add_entity(
            gs.morphs.Box(pos=tuple(asset["xyz"][:3]), quat=genesis_quat(asset.get("rpy", [0.0, 0.0, 0.0])),
                          size=tuple(asset["size"][:3]), fixed=True),
            surface=gs.surfaces.Default(color=scene_color(asset)),
            name=asset["name"],
        )
    objects = {}
    for entry in scenario.get("objects", []):
        pose = dict(pos=tuple(entry["xyz"][:3]), quat=genesis_quat(entry.get("rpy", [0.0, 0.0, 0.0])))
        if entry.get("shape") == "cylinder":
            radius, height = float(entry["radius_m"]), float(entry["height_m"])
            morph = gs.morphs.Cylinder(radius=radius, height=height, **pose)
            volume = math.pi * radius * radius * height
        else:
            morph = gs.morphs.Box(size=tuple(entry["size"][:3]), **pose)
            volume = float(np.prod(entry["size"][:3]))
        material = gs.materials.Rigid(rho=float(entry.get("mass_kg", 0.04)) / volume,
                                      friction=float(entry.get("friction", 0.7)))
        objects[entry["id"]] = scene.add_entity(
            morph, material=material, surface=gs.surfaces.Default(color=scene_color(entry)), name=entry["id"])
    return objects


class GenesisBridge(Node):
    def __init__(self, camera_config=None):
        super().__init__("so101_genesis_bridge")
        self.target = np.zeros(len(JOINT_NAMES), dtype=np.float32)
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.subscription = self.create_subscription(
            Float64MultiArray, "/so101/joint_commands", self._command, 10
        )
        self.camera_config = camera_config
        if camera_config is not None:
            self.image_pub = self.create_publisher(Image, RGB_TOPIC, 5)
            self.camera_info_pub = self.create_publisher(CameraInfo, RGB_CAMERA_INFO_TOPIC, 5)
            self.depth_pub = self.create_publisher(Image, DEPTH_TOPIC, 5)
            self.depth_info_pub = self.create_publisher(CameraInfo, DEPTH_CAMERA_INFO_TOPIC, 5)

    def _command(self, msg):
        try:
            self.target = validated_joint_command(msg.data).astype(np.float32)
        except ValueError as exc:
            self.get_logger().warning(f"Ignoring SO-101 joint command: {exc}")

    def publish_state(self, positions, velocities):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = to_numpy(positions).tolist()
        msg.velocity = to_numpy(velocities).tolist()
        self.publisher.publish(msg)

    def _camera_info(self, stamp, frame_id):
        width, height = (int(value) for value in self.camera_config["resolution"])
        values = camera_info_values(width, height, float(self.camera_config["horizontal_fov_deg"]))
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.width = values["width"]
        info.height = values["height"]
        info.distortion_model = values["distortion_model"]
        info.d = values["d"]
        info.k = values["k"]
        info.r = values["r"]
        info.p = values["p"]
        return info

    def publish_camera(self, rgb, depth):
        stamp = self.get_clock().now().to_msg()
        rgb = np.ascontiguousarray(np.asarray(rgb)[:, :, :3], dtype=np.uint8)
        height, width = rgb.shape[:2]
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = CAMERA_FRAME_ID
        image.height, image.width = height, width
        image.encoding = "rgb8"
        image.step = width * 3
        # An array("B") skips the per-byte validation Humble's rclpy runs on
        # other sequences (about 40 ms for one 640x360 frame).
        image.data = array.array("B", rgb.tobytes())
        self.image_pub.publish(image)
        self.camera_info_pub.publish(self._camera_info(stamp, CAMERA_FRAME_ID))
        if depth is not None:
            depth = np.ascontiguousarray(np.asarray(depth, dtype=np.float32).reshape(height, width))
            depth_msg = Image()
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = DEPTH_FRAME_ID
            depth_msg.height, depth_msg.width = height, width
            depth_msg.encoding = "32FC1"
            depth_msg.step = width * 4
            depth_msg.data = array.array("B", depth.tobytes())
            self.depth_pub.publish(depth_msg)
            self.depth_info_pub.publish(self._camera_info(stamp, DEPTH_FRAME_ID))


def load_scenario(path):
    if not path:
        return {}
    if not os.path.isfile(path):
        raise FileNotFoundError(path)
    return json.loads(Path(path).read_text())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", default="/workspace/systems/so101/urdf/so101.urdf")
    parser.add_argument("--scenario", default=DEFAULT_SCENARIO,
                        help="scene to build ('' for the arm on the ground plane only)")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--backend", choices=("gpu", "cpu"), default="gpu")
    parser.add_argument("--no-camera", action="store_true", help="do not render the scenario camera")
    parser.add_argument("--heartbeat-s", type=float, default=float(os.environ.get("SO101_LOG_HEARTBEAT_SECONDS", "30")),
                        help="print the objects' positions this often (0: never)")
    args = parser.parse_args()

    if not os.path.isfile(args.urdf):
        raise FileNotFoundError(args.urdf)
    scenario = load_scenario(args.scenario)
    for error in scene_support_errors(scenario) if scenario else ():
        print(f"SO-101 scenario inconsistency: {error}", flush=True)
    camera_config = scenario_camera(scenario) if scenario and not args.no_camera else None

    rclpy.init()
    bridge = GenesisBridge(camera_config)
    gs.init(backend=getattr(gs, args.backend))
    scene = gs.Scene(show_viewer=not args.headless)
    scene.add_entity(gs.morphs.Plane())
    spawn = scenario.get("robot_spawn", {})
    robot = scene.add_entity(gs.morphs.URDF(
        file=args.urdf, fixed=True, pos=tuple(spawn.get("xyz", [0.0, 0.0, 0.0])),
        quat=genesis_quat(spawn.get("rpy", [0.0, 0.0, 0.0]))))
    objects = add_scenario(scene, scenario) if scenario else {}
    camera = None
    if camera_config is not None:
        width, height = (int(value) for value in camera_config["resolution"])
        pos, lookat, up = camera_look(camera_config)
        near, far = (float(value) for value in camera_config.get("clipping_range_m", [0.01, 100.0]))
        camera = scene.add_camera(
            res=(width, height), pos=pos, lookat=lookat, up=up,
            fov=vertical_fov_deg(camera_config["horizontal_fov_deg"], width, height),
            near=near, far=min(far, 10.0), GUI=False)
    scene.build()

    dofs = [robot.get_joint(name).dof_idx_local for name in JOINT_NAMES]
    robot.set_dofs_kp(np.array([80, 80, 70, 45, 30, 20]), dofs_idx_local=dofs)
    robot.set_dofs_kv(np.array([8, 8, 7, 5, 4, 2]), dofs_idx_local=dofs)

    dt = float(scene.sim_options.dt) if hasattr(scene, "sim_options") else 0.01
    camera_period = 1.0 / float(camera_config.get("frame_rate_hz", 30.0)) if camera_config else math.inf
    start = time.monotonic()
    next_camera = start
    last_heartbeat = start - args.heartbeat_s if args.heartbeat_s > 0 else math.inf
    steps = 0
    print(f"SO-101 Genesis loop running (scenario={args.scenario or 'none'}, "
          f"camera={'on' if camera is not None else 'off'})", flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(bridge, timeout_sec=0.0)
            robot.control_dofs_position(bridge.target, dofs_idx_local=dofs)
            scene.step()
            steps += 1
            bridge.publish_state(
                robot.get_dofs_position(dofs_idx_local=dofs),
                robot.get_dofs_velocity(dofs_idx_local=dofs),
            )
            now = time.monotonic()
            if camera is not None and now >= next_camera:
                rgb, depth, _, _ = camera.render(rgb=True, depth=True)
                bridge.publish_camera(rgb, depth)
                next_camera = max(next_camera + camera_period, now)
            if objects and now - last_heartbeat >= args.heartbeat_s:
                poses = {name: [round(v, 4) for v in to_numpy(entity.get_pos()).tolist()]
                         for name, entity in objects.items()}
                rtf = steps * dt / max(now - start, 1e-9)
                print(f"SO-101 Genesis t={steps * dt:.2f}s (real-time factor {rtf:.2f}) objects: "
                      f"{json.dumps(poses)}", flush=True)
                last_heartbeat = now
            # Keep simulated time at wall-clock rate: the policies' command
            # rates and the camera's frame rate are real-time rates.
            delay = start + steps * dt - time.monotonic()
            time.sleep(max(delay, 0.0005))
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
