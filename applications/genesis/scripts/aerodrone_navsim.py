#!/usr/bin/env python3
"""Genesis terrain, drone physics, rendered sensors, and ArduPilot backend."""

from __future__ import annotations

import argparse
import array
import faulthandler
import json
import math
import os
from pathlib import Path
import select
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

sys.path.insert(0, "/workspace/applications/ardupilot_sitl/scripts")
# tracking_targets.py: mounted there by the composition, in the repository
# next to this application.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "demnav" / "scripts"))
sys.path.insert(0, "/workspace/applications/demnav/scripts")
from ardupilot_json import (  # noqa: E402
    ArduPilotJSON,
    quaternion_to_matrix,
    state_from_enu,
)
from tracking_targets import HIDE_TOPIC, TRUTH_TOPIC, Tracks, truth_json  # noqa: E402

# Genesis' urdf/drones/cf2x.urdf: mass, thrust (N/rpm^2) and reaction
# moment (N m/rpm^2) coefficients.
CF2X_MASS_KG = 0.027
CF2X_KF = 3.16e-10
CF2X_KM = 7.94e-12
# Propeller hubs (the prop*_link inertial origins), body FLU metres. The
# URDF attaches those links with no joint offset, so Genesis' own
# set_propellers_rpm() applies every thrust at the body origin: it can
# yaw and climb but never roll or pitch. rotor_wrench() uses these arms.
CF2X_ROTOR_POSITIONS_FLU = np.array([
    [0.028, -0.028],   # prop0 front-right
    [-0.028, -0.028],  # prop1 rear-right
    [-0.028, 0.028],   # prop2 rear-left
    [0.028, 0.028],    # prop3 front-left
])
# ArduPilot's quad X (FRAME_CLASS 1, FRAME_TYPE 1; AP_MotorsMatrix) outputs
# motor 1 front-right CCW, 2 rear-left CCW, 3 front-left CW, 4 rear-right
# CW: cf2x propeller i is ArduPilot output MOTOR_ORDER[i].
MOTOR_ORDER = (0, 3, 1, 2)
# Reaction torque about body +z, opposite the propeller's spin: CCW
# propellers (motors 1, 2) turn the body clockwise seen from above.
ROTOR_YAW_SIGN = np.array([-1.0, 1.0, -1.0, 1.0])
# ArduPilot's motor model (AP_Motors_Thrust_Linearization), pinned in
# config/aerodrone_sitl.params: no thrust below MOT_SPIN_MIN of the PWM
# range, full thrust at MOT_SPIN_MAX, (1 - MOT_THST_EXPO) r + MOT_THST_EXPO
# r^2 in between.
PWM_MIN, PWM_MAX = 1000.0, 2000.0
MOT_SPIN_MIN, MOT_SPIN_MAX, MOT_THST_EXPO = 0.15, 0.95, 0.65
GRAVITY = 9.80665


def rotor_thrust_fraction(pwm):
    """Thrust of each ArduPilot output, as a fraction of full thrust."""
    actuator = (np.asarray(pwm, dtype=np.float64) - PWM_MIN) / (PWM_MAX - PWM_MIN)
    ratio = np.clip((actuator - MOT_SPIN_MIN) / (MOT_SPIN_MAX - MOT_SPIN_MIN), 0.0, 1.0)
    return (1.0 - MOT_THST_EXPO) * ratio + MOT_THST_EXPO * ratio * ratio


def air_density_ratio(altitude_m):
    """ISA density over sea level's. ArduPilot divides its motor output by
    the barometric estimate of it, so propeller thrust must scale with it."""
    return (1.0 - 2.25577e-5 * altitude_m) ** 4.25588


def rotor_thrusts(pwm, maximum_thrust_n, density_ratio=1.0):
    """Thrust (N) of cf2x propellers 0-3 from ArduPilot's motor PWM."""
    pwm = np.asarray(pwm, dtype=np.float64)[list(MOTOR_ORDER)]
    return maximum_thrust_n * density_ratio * rotor_thrust_fraction(pwm)


def rotor_wrench(thrusts, moment_ratio=CF2X_KM / CF2X_KF):
    """Body FLU force and torque of the propeller thrusts (cf2x order):
    thrust along +z at each hub plus each propeller's reaction moment."""
    thrusts = np.asarray(thrusts, dtype=np.float64)
    force = np.array([0.0, 0.0, thrusts.sum()])
    torque = np.array([
        CF2X_ROTOR_POSITIONS_FLU[:, 1] @ thrusts,
        -CF2X_ROTOR_POSITIONS_FLU[:, 0] @ thrusts,
        moment_ratio * (ROTOR_YAW_SIGN @ thrusts),
    ])
    return force, torque


def env_float(name, default):
    return float(os.environ.get(name, default))


def env_int(name, default):
    return int(os.environ.get(name, default))


def vertical_fov_deg(horizontal_fov_deg, width, height):
    """Genesis cameras take the vertical field of view; FOV_DEG (like the
    camera_info published here) is the horizontal one, square pixels."""
    half = math.tan(math.radians(horizontal_fov_deg) / 2.0) * height / width
    return math.degrees(2.0 * math.atan(half))


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
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.targets_pub = self.create_publisher(String, TRUTH_TOPIC, 10)

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

    def publish_clock(self, seconds):
        """Simulation time (ArduPilot's lock-step time) on /clock: the
        navigation fusion ages its dead reckoning by it, not by wall time."""
        nanoseconds = round(seconds * 1e9)
        msg = Clock()
        msg.clock.sec, msg.clock.nanosec = divmod(nanoseconds, 1_000_000_000)
        self.clock_pub.publish(msg)

    def publish_targets(self, text):
        """Tracking-target ground truth (systems/aerodrone/tracking.md)."""
        msg = String()
        msg.data = text
        self.targets_pub.publish(msg)

    def subscribe_hide(self, tracks):
        """Hide the targets named on HIDE_TOPIC (tracking_targets.py)."""
        self.create_subscription(String, HIDE_TOPIC, lambda msg: tracks.hide(msg.data), 10)

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
        # array('B') is the message setter's fast path: bytes are checked
        # element by element, which took a third of the lock-step time.
        rgb_msg.data = array.array("B", rgb.tobytes())
        self.rgb_pub.publish(rgb_msg)
        self.rgb_info_pub.publish(self.camera_info(stamp, frame))
        depth_msg = Image()
        depth_msg.header.stamp, depth_msg.header.frame_id = stamp, frame
        depth_msg.height, depth_msg.width = depth.shape[:2]
        depth_msg.encoding, depth_msg.step = "32FC1", depth.shape[1] * 4
        depth_msg.data = array.array("B", depth.tobytes())
        self.depth_pub.publish(depth_msg)
        self.depth_info_pub.publish(self.camera_info(stamp, frame))


def add_tracking_targets(gs, scene, assets):
    """The tracking targets of assets/tracking_targets.json (cars, people:
    build_georeferenced_terrain.py --targets), as visual-only fixed meshes
    with their baked texture atlas, moved by place_tracking_targets.
    Returns (Tracks, entities), or (None, []) for terrain without targets."""
    path = Path(assets) / "tracking_targets.json"
    if not path.is_file():
        return None, []
    tracks = Tracks.load(path)
    entities = []
    for track, pose in zip(tracks.tracks, tracks.poses(0.0)):
        entities.append(scene.add_entity(
            gs.morphs.Mesh(
                file=str(tracks.model_file(track, "mesh")), fixed=True, collision=False,
                pos=(pose.east, pose.north, pose.up), quat=pose.quaternion_wxyz()),
            surface=gs.surfaces.Default(diffuse_texture=gs.textures.ImageTexture(
                image_path=str(tracks.model_file(track, "texture"))))))
    print(f"Genesis tracking targets: {', '.join(t.name for t in tracks.tracks)}", flush=True)
    return tracks, entities


def place_tracking_targets(entities, poses):
    """Move each target to its pose (terrain ENU) before a render."""
    for entity, pose in zip(entities, poses):
        entity.set_pos((pose.east, pose.north, pose.up))
        entity.set_quat(pose.quaternion_wxyz())


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
    scene.add_entity(gs.morphs.Terrain(
        height_field=height_field, horizontal_scale=horizontal_scale,
        vertical_scale=1.0, pos=(-half_m, -half_m, 0.0),
        visualization=False, collision=True))
    # The cameras see the shared textured terrain mesh (Gazebo's and
    # Isaac's too) with its georeferenced UVs: Terrain's own UVs would
    # stretch the orthophoto, which only covers the WildNav tile mosaic,
    # over the whole heightfield.
    terrain_surface = gs.surfaces.Default(
        diffuse_texture=gs.textures.ImageTexture(
            image_path=str(assets / manifest["assets"]["texture"])))
    scene.add_entity(gs.morphs.Mesh(
        file=str(assets / manifest["assets"]["mesh"]), fixed=True,
        collision=False), surface=terrain_surface)
    # The common home is the top of Gazebo's conservative DEM collision cell.
    # Put a small real collision pad at that same surveyed elevation so the
    # lightweight drone starts at rest instead of falling onto a steep,
    # interpolated heightfield triangle while ArduPilot initializes.
    scene.add_entity(gs.morphs.Box(
        pos=(float(home[0]), float(home[1]), float(home[2] - 0.12)),
        size=(3.0, 3.0, 0.2), fixed=True))
    drone = scene.add_entity(gs.morphs.Drone(
        file="urdf/drones/cf2x.urdf", pos=tuple(home.tolist())))
    tracks, target_entities = add_tracking_targets(gs, scene, assets)
    # far: the CRUISE_AGL_M cruise plus the terrain's relief across the
    # footprint must stay in range of the depth image.
    camera = scene.add_camera(
        res=(args.width, args.height), pos=tuple(home.tolist()),
        lookat=(home[0], home[1], home[2] - 10.0), up=(1.0, 0.0, 0.0),
        fov=vertical_fov_deg(args.fov_deg, args.width, args.height),
        near=0.3, far=1000.0, GUI=False)
    print("Genesis scene build starting", flush=True)
    scene.build()
    print("Genesis scene build complete", flush=True)
    faulthandler.cancel_dump_traceback_later()

    rclpy.init()
    publisher = GenesisSensorPublisher(args)
    if tracks is not None:
        publisher.subscribe_hide(tracks)
    transport = ArduPilotJSON(port=int(os.environ.get("ARDUPILOT_JSON_PORT", "9002")))
    Path("/tmp/genesis_sim_ready").touch()
    dt = 1.0 / args.rate_hz
    camera_stride = max(1, int(round(args.rate_hz / args.camera_rate_hz)))
    maximum_rotor_rpm = env_float("GENESIS_MAX_ROTOR_RPM", 21700.0)
    maximum_thrust_n = CF2X_KF * maximum_rotor_rpm ** 2
    if not np.allclose([float(drone.get_mass()), drone.KF, drone.KM],
                       [CF2X_MASS_KG, CF2X_KF, CF2X_KM], rtol=1e-3):
        raise RuntimeError("Genesis cf2x.urdf no longer matches the rotor model")
    print(
        f"Genesis rotors: {4.0 * maximum_thrust_n:.3f} N maximum thrust, "
        f"hover at {CF2X_MASS_KG * GRAVITY / (4.0 * maximum_thrust_n):.3f} "
        "of it (MOT_THST_HOVER)", flush=True)
    previous_velocity = np.zeros(3)
    position = to_numpy(drone.get_pos())[:3]
    rotation = quaternion_to_matrix(to_numpy(drone.get_quat())[:4])
    camera_up = np.array([1.0, 0.0, 0.0])
    tick = 0
    start = time.monotonic()
    try:
        while rclpy.ok():
            frame = transport.receive()
            if frame is None:
                # Wake on ArduPilot's next servo frame, not a whole dt later.
                select.select([transport.socket], [], [], dt)
                continue
            # The previous step's attitude: forces are applied in world axes.
            force, torque = rotor_wrench(rotor_thrusts(
                frame.pwm[:4], maximum_thrust_n,
                air_density_ratio(args.origin_alt + position[2])))
            drone.solver.apply_links_external_force(
                (rotation @ force)[None], [drone.base_link_idx], ref="link_com")
            drone.solver.apply_links_external_torque(
                (rotation @ torque)[None], [drone.base_link_idx], ref="link_com")
            scene.step()

            position = to_numpy(drone.get_pos())[:3]
            quaternion = to_numpy(drone.get_quat())[:4]
            rotation = quaternion_to_matrix(quaternion)
            velocity = to_numpy(drone.get_vel())[:3]
            angular = to_numpy(drone.get_ang())[:3]  # world axes
            acceleration_world = (velocity - previous_velocity) / dt
            previous_velocity = velocity.copy()
            specific_force_flu = rotation.T @ (
                acceleration_world - np.array([0.0, 0.0, -GRAVITY]))
            transport.send(state_from_enu(
                position, velocity, quaternion, rotation.T @ angular,
                specific_force_flu, args.lat, args.lon, args.origin_alt,
                timestamp=tick * dt, reference_enu=home))

            if tick % camera_stride == 0:
                # Body-fixed nadir camera: the image top points along the
                # nose (horizontal body x axis), like the physical OAK-1 with
                # camera_mount_yaw_deg 0 and the DemNav/WildNav de-rotation.
                forward = rotation[:, 0]
                horizontal = np.array([forward[0], forward[1], 0.0])
                if np.linalg.norm(horizontal) > 1e-6:
                    camera_up = horizontal / np.linalg.norm(horizontal)
                camera.set_pose(
                    pos=position.tolist(),
                    lookat=(position + np.array([0.0, 0.0, -10.0])).tolist(),
                    up=tuple(float(value) for value in camera_up))
                # The node only publishes: no need to spin it every tick.
                rclpy.spin_once(publisher, timeout_sec=0.0)
                publisher.publish_clock(tick * dt)
                # The targets move with the simulation clock; the frame
                # shows them where the ground truth published with it says.
                if tracks is not None:
                    poses = tracks.poses(tick * dt)
                    place_tracking_targets(target_entities, tracks.placed(poses))
                rendered = camera.render(rgb=True, depth=True)
                publisher.publish_camera(rendered[0], rendered[1])
                if tracks is not None:
                    publisher.publish_targets(
                        truth_json(tick * dt, poses, tracks.origin, tracks.hidden))
                publisher.publish_truth(position.tolist(), quaternion.tolist(),
                                        velocity.tolist(), angular.tolist())
            if tick % max(1, int(args.rate_hz * 5.0)) == 0:
                print(
                    "Genesis physics "
                    f"frame_rate={frame.frame_rate_hz}Hz "
                    f"pwm={list(frame.pwm[:4])} "
                    f"position={position.round(3).tolist()} "
                    f"quaternion={quaternion.round(4).tolist()}",
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
