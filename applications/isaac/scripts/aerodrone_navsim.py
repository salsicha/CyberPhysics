#!/usr/bin/env python3
"""Isaac Sim terrain, rotor physics, sensors, and ArduPilot JSON backend.

Lock-step: every new ArduPilot servo frame advances the physics by exactly
one ``1 / ISAAC_PHYSICS_HZ`` step and is answered with the resulting state,
stamped ``tick * dt`` (ArduPilot's clock, also published on /clock). No
physics runs without a servo frame. ``World.step(render=True)`` must never
be used here: with the world's rendering_dt it runs a whole camera period
of physics substeps (120 at 1200 Hz) in one call, without the motor forces
and outside ArduPilot's clock.

Camera: every ``ISAAC_PHYSICS_HZ / ISAAC_CAMERA_HZ`` ticks the stage is
rendered without stepping physics until the annotators deliver the frame of
the current simulation time (RTX hands annotators each frame two renders
later), then the RGB and depth frames, their camera_info, the ground truth
and /clock are published together with one stamp, so each frame shows the
vehicle pose it is stamped with.
"""

from __future__ import annotations

import argparse
import array
import json
import math
import os
from pathlib import Path
import select
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ardupilot_sitl" / "scripts"))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "demnav" / "scripts"))
from ardupilot_json import (  # noqa: E402
    ArduPilotJSON,
    pwm_to_rotor_speed,
    quaternion_to_matrix,
    state_from_enu,
)
from tracking_targets import HIDE_TOPIC, TRUTH_TOPIC, Tracks, truth_json  # noqa: E402


DRONE_SIZE_M = (0.47, 0.47, 0.11)
CAMERA_PATH = "/World/NavDrone/downward_camera"
CAMERA_FRAME = "downward_camera_optical_frame"
# Optical centre below the body origin.
CAMERA_OFFSET_M = 0.08
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
# 60 degrees across the width; RTX derives the vertical extent from the
# image aspect ratio (square pixels).
HORIZONTAL_APERTURE_MM = 21.0
FOCAL_LENGTH_MM = 18.2
# Uniform sky: the ground shows the orthophoto's own (already sunlit)
# colours, as WildNav's reference tiles do. Isaac Sim 6 lights follow USD's
# physical units, so a small-angle DistantLight of a few thousand units
# leaves the terrain black.
SKY_INTENSITY = 900.0
# Anti-aliasing at the full image resolution (DLAA). The real-time path
# tracer otherwise uses Isaac's default DLSS "performance" mode, which
# renders a 640x480 frame at 320x240. Applied once the stage exists: a new
# stage resets the launch config's render settings.
RENDER_SETTINGS = {"/rtx/post/aa/op": 4}
# Renders allowed for a camera frame to reach the annotators.
MAX_RENDERS_PER_FRAME = 8


def env_float(name, default):
    return float(os.environ.get(name, default))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--lat", type=float, default=env_float("INITIAL_LAT", 37.9234))
    parser.add_argument("--lon", type=float, default=env_float("INITIAL_LON", -122.5967))
    parser.add_argument("--origin-alt", type=float, default=env_float("ORIGIN_ALT", 781.0))
    parser.add_argument("--assets", default="/data/navsim_assets")
    parser.add_argument("--duration-s", type=float, default=env_float("ISAAC_DURATION_S", 0.0))
    parser.add_argument("--rgb-topic", default=os.environ.get(
        "WILDNAV_IMAGE_TOPIC", "/aerodrone/sensor_measurements/downward_rgb/image_raw"))
    parser.add_argument("--rgb-info-topic", default=os.environ.get(
        "WILDNAV_CAMERA_INFO_TOPIC", "/aerodrone/sensor_measurements/downward_rgb/camera_info"))
    parser.add_argument("--depth-topic", default=os.environ.get(
        "DEPTH_TOPIC", "/aerodrone/sensor_measurements/downward_rgbd/depth"))
    parser.add_argument("--depth-info-topic", default=os.environ.get(
        "CAMERA_INFO_TOPIC", "/aerodrone/sensor_measurements/downward_rgbd/depth/camera_info"))
    return parser.parse_known_args()[0]


def camera_intrinsics(width=IMAGE_WIDTH, height=IMAGE_HEIGHT,
                      horizontal_aperture_mm=HORIZONTAL_APERTURE_MM,
                      focal_length_mm=FOCAL_LENGTH_MM):
    """(fx, fy, cx, cy) of the rendered image in ROS convention (pixel
    centres at integer coordinates). RTX maps the horizontal aperture onto
    the image width with square pixels and the optical axis through the
    image centre."""
    fx = width * focal_length_mm / horizontal_aperture_mm
    return fx, fx, (width - 1) / 2.0, (height - 1) / 2.0


def render_stride(physics_rate_hz, camera_rate_hz):
    """Physics ticks per camera frame."""
    return max(1, int(round(physics_rate_hz / camera_rate_hz)))


def is_new_frame(frame, last_frame_count):
    """A servo frame ArduPilot has not been answered for yet. ArduPilot
    re-sends the same frame (same frame_count) when no state arrives within
    a second; that one must not advance the physics again."""
    return frame is not None and frame.frame_count != last_frame_count


def rgb8(rgba):
    """Annotator RGBA (H x W x 4, uint8) to a contiguous rgb8 array."""
    return np.ascontiguousarray(np.asarray(rgba, dtype=np.uint8)[..., :3])


def depth32(distance):
    """distance_to_image_plane (metres along the optical axis) as 32FC1."""
    return np.ascontiguousarray(np.asarray(distance, dtype=np.float32).reshape(
        np.shape(distance)[:2]))


def as_numpy(value):
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value, dtype=np.float64)


def add_terrain(stage, asset_dir):
    from pxr import PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

    data = np.load(Path(asset_dir) / "terrain_mesh.npz")
    vertices = data["vertices"]
    faces = data["faces"]
    uvs = data["uvs"]
    mesh = UsdGeom.Mesh.Define(stage, "/World/GeoreferencedTerrain")
    mesh.CreatePointsAttr(vertices.tolist())
    mesh.CreateFaceVertexCountsAttr([3] * len(faces))
    mesh.CreateFaceVertexIndicesAttr(faces.reshape(-1).tolist())
    mesh.CreateSubdivisionSchemeAttr("none")
    primvars = UsdGeom.PrimvarsAPI(mesh)
    st = primvars.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                UsdGeom.Tokens.vertex)
    st.Set(uvs.tolist())
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")

    material = UsdShade.Material.Define(stage, "/World/Looks/Orthophoto")
    surface = UsdShade.Shader.Define(stage, "/World/Looks/Orthophoto/Surface")
    surface.CreateIdAttr("UsdPreviewSurface")
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.92)
    texture = UsdShade.Shader.Define(stage, "/World/Looks/Orthophoto/Texture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
        str(Path(asset_dir) / "orthophoto.jpg"))
    texture.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    texture.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    reader = UsdShade.Shader.Define(stage, "/World/Looks/Orthophoto/Primvar")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result")
    surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture.ConnectableAPI(), "rgb")
    material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)


def add_target_material(stage, path, texture_file):
    """UsdPreviewSurface reading a target model's baked texture atlas."""
    from pxr import Sdf, UsdShade

    material = UsdShade.Material.Define(stage, path)
    surface = UsdShade.Shader.Define(stage, f"{path}/Surface")
    surface.CreateIdAttr("UsdPreviewSurface")
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    texture = UsdShade.Shader.Define(stage, f"{path}/Texture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(texture_file))
    texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    for wrap in ("wrapS", "wrapT"):
        texture.CreateInput(wrap, Sdf.ValueTypeNames.Token).Set("clamp")
    reader = UsdShade.Shader.Define(stage, f"{path}/Primvar")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result")
    surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture.ConnectableAPI(), "rgb")
    material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
    return material


def add_tracking_targets(stage, asset_dir):
    """The tracking targets of tracking_targets.json (cars, people:
    build_georeferenced_terrain.py --targets) as visual-only meshes, no
    physics, each under an Xform that place_tracking_targets moves.
    Returns (Tracks, [(translate op, orient op)]), or (None, []) for
    terrain without targets."""
    from pxr import Sdf, UsdGeom, UsdShade, Vt

    path = Path(asset_dir) / "tracking_targets.json"
    if not path.is_file():
        return None, []
    tracks = Tracks.load(path)
    UsdGeom.Xform.Define(stage, "/World/Targets")
    materials = {}
    ops = []
    for track in tracks.tracks:
        if track.model not in materials:
            materials[track.model] = add_target_material(
                stage, f"/World/Looks/Target_{track.model}", tracks.model_file(track, "texture"))
        xform = UsdGeom.Xform.Define(stage, f"/World/Targets/{track.name}")
        translate = xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
        orient = xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
        data = np.load(tracks.model_file(track, "mesh_npz"))
        positions = np.ascontiguousarray(data["positions"], dtype=np.float32)
        mesh = UsdGeom.Mesh.Define(stage, f"/World/Targets/{track.name}/Model")
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(positions))
        mesh.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(
            np.stack([positions.min(axis=0), positions.max(axis=0)])))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(data["triangles"])))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(
            np.ascontiguousarray(data["triangles"].reshape(-1), dtype=np.int32)))
        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(
            np.ascontiguousarray(data["normals"], dtype=np.float32)))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        mesh.CreateSubdivisionSchemeAttr("none")
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        st.Set(Vt.Vec2fArray.FromNumpy(np.ascontiguousarray(data["uvs"], dtype=np.float32)))
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(materials[track.model])
        ops.append((translate, orient))
    place_tracking_targets(ops, tracks.poses(0.0))
    print(f"Isaac tracking targets: {', '.join(t.name for t in tracks.tracks)}", flush=True)
    return tracks, ops


def place_tracking_targets(ops, poses):
    """Move each target to its pose (terrain ENU) before a render."""
    from pxr import Gf

    for (translate, orient), pose in zip(ops, poses):
        translate.Set(Gf.Vec3d(pose.east, pose.north, pose.up))
        w, x, y, z = pose.quaternion_wxyz()
        orient.Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))


def add_sky(stage):
    from pxr import UsdLux

    UsdLux.DomeLight.Define(stage, "/World/Sky").CreateIntensityAttr(SKY_INTENSITY)


def add_camera(stage):
    from pxr import Gf, UsdGeom

    # A child of the drone, looking down its -z axis: the image top points
    # along the body +y axis (left of the nose), camera_mount_yaw_deg 90.
    # The drone prim is scaled to its box size; undo that for the camera.
    camera = UsdGeom.Camera.Define(stage, CAMERA_PATH)
    camera.GetHorizontalApertureAttr().Set(HORIZONTAL_APERTURE_MM)
    camera.GetVerticalApertureAttr().Set(
        HORIZONTAL_APERTURE_MM * IMAGE_HEIGHT / IMAGE_WIDTH)
    camera.GetFocalLengthAttr().Set(FOCAL_LENGTH_MM)
    # Far enough for the CRUISE_AGL_M cruise over the terrain's relief.
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.3, 1000.0))
    xform = UsdGeom.Xformable(camera)
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -CAMERA_OFFSET_M / DRONE_SIZE_M[2]))
    xform.AddScaleOp().Set(Gf.Vec3f(*(1.0 / size for size in DRONE_SIZE_M)))


class DownwardCamera:
    """RGB and depth annotators on the downward camera's render product,
    with the simulation time of the frame they hold."""

    def __init__(self):
        import omni.replicator.core as rep

        self.render_product = rep.create.render_product(
            CAMERA_PATH, (IMAGE_WIDTH, IMAGE_HEIGHT))
        self.rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        self.depth = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        self.time = rep.AnnotatorRegistry.get_annotator("IsaacReadSimulationTime")
        for annotator in (self.rgb, self.depth, self.time):
            annotator.attach([self.render_product])

    def frame_time(self):
        """Simulation time the annotators' frame was rendered at, or None."""
        try:
            return float(self.time.get_data()["simulationTime"])
        except (KeyError, TypeError, ValueError):
            return None

    def read(self):
        """(rgb8, 32FC1 depth), or None before the first rendered frame."""
        rgba = np.asarray(self.rgb.get_data())
        distance = np.asarray(self.depth.get_data())
        if rgba.ndim != 3 or distance.ndim < 2:
            return None
        return rgb8(rgba), depth32(distance)

    def capture(self, world, tolerance_s):
        """Render the current physics state (no physics step) until the
        annotators hold its frame: (rgb8, depth), or None if they do not
        within MAX_RENDERS_PER_FRAME renders."""
        for _ in range(MAX_RENDERS_PER_FRAME):
            world.render()
            frame_time = self.frame_time()
            if frame_time is not None and abs(frame_time - world.current_time) <= tolerance_s:
                return self.read()
        return None


def build_world(asset_dir, physics_rate, camera_rate):
    """The terrain, sky, launch pad, drone and its downward camera.
    Returns (world, rigid body view of the drone, home ENU position)."""
    import carb
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
    from isaacsim.core.experimental.prims import RigidPrim
    from pxr import PhysxSchema

    manifest = json.loads((Path(asset_dir) / "manifest.json").read_text(encoding="utf-8"))
    home = np.asarray(manifest["home_enu_m"], dtype=np.float64)
    world = World(physics_dt=1.0 / physics_rate,
                  rendering_dt=1.0 / camera_rate,
                  stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    add_terrain(stage, asset_dir)
    add_sky(stage)
    # The common home is the surveyed top of the conservative DEM collision
    # cell.  This pad prevents a cold-start fall onto a coarser mesh triangle
    # while ArduPilot initializes, without replacing the surrounding terrain.
    world.scene.add(FixedCuboid(
        prim_path="/World/LaunchPad", name="launch_pad",
        position=home + np.array([0.0, 0.0, -0.12]),
        scale=np.array([3.0, 3.0, 0.2]),
        color=np.array([0.20, 0.22, 0.24])))
    world.scene.add(DynamicCuboid(
        prim_path="/World/NavDrone", name="nav_drone", position=home,
        scale=np.array(DRONE_SIZE_M), mass=1.5,
        color=np.array([0.04, 0.18, 0.72])))
    # Match the passive rigid-body losses present in the Gazebo rotor model.
    # A perfectly lossless cuboid is not a credible airframe and makes the
    # stock ArduCopter rate loop unrealistically underdamped.
    physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(
        stage.GetPrimAtPath("/World/NavDrone"))
    physx_body.CreateLinearDampingAttr().Set(0.15)
    physx_body.CreateAngularDampingAttr().Set(0.20)
    add_camera(stage)
    rigid = RigidPrim("/World/NavDrone", masses=[1.5])
    world.reset()
    settings = carb.settings.get_settings()
    for key, value in RENDER_SETTINGS.items():
        settings.set(key, value)
    return world, rigid, home


def make_publisher(args):
    """ROS 2 node publishing ground truth, /clock and the camera frames."""
    from geometry_msgs.msg import Quaternion
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String

    class SimPublisher(Node):
        def __init__(self):
            super().__init__("isaac_sim_ground_truth")
            self.publisher = self.create_publisher(
                Odometry, "/aerodrone/simulation/ground_truth/odom", 10)
            self.clock_publisher = self.create_publisher(Clock, "/clock", 10)
            self.rgb_publisher = self.create_publisher(Image, args.rgb_topic, 10)
            self.rgb_info_publisher = self.create_publisher(
                CameraInfo, args.rgb_info_topic, 10)
            self.depth_publisher = self.create_publisher(Image, args.depth_topic, 10)
            self.depth_info_publisher = self.create_publisher(
                CameraInfo, args.depth_info_topic, 10)
            self.targets_publisher = self.create_publisher(String, TRUTH_TOPIC, 10)

        def publish_targets(self, text):
            """Tracking-target ground truth (systems/aerodrone/tracking.md)."""
            msg = String()
            msg.data = text
            self.targets_publisher.publish(msg)

        def subscribe_hide(self, tracks):
            """Hide the targets named on HIDE_TOPIC (tracking_targets.py)."""
            self.create_subscription(String, HIDE_TOPIC,
                                     lambda msg: tracks.hide(msg.data), 10)

        def stamp(self):
            return self.get_clock().now().to_msg()

        def publish_clock(self, seconds):
            """Simulation time (ArduPilot's lock-step time) on /clock: the
            navigation fusion ages its dead reckoning by it, not by wall time."""
            nanoseconds = round(seconds * 1e9)
            msg = Clock()
            msg.clock.sec, msg.clock.nanosec = divmod(nanoseconds, 1_000_000_000)
            self.clock_publisher.publish(msg)

        def publish_state(self, stamp, position, quaternion, velocity, angular):
            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = "earth"
            msg.child_frame_id = "aerodrone/base_link"
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = position
            msg.pose.pose.orientation = Quaternion(
                w=float(quaternion[0]), x=float(quaternion[1]),
                y=float(quaternion[2]), z=float(quaternion[3]))
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = velocity
            msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = angular
            self.publisher.publish(msg)

        def camera_info(self, stamp, width, height):
            fx, fy, cx, cy = camera_intrinsics(width, height)
            msg = CameraInfo()
            msg.header.stamp = stamp
            msg.header.frame_id = CAMERA_FRAME
            msg.width, msg.height = width, height
            msg.distortion_model = "plumb_bob"
            msg.d = [0.0] * 5
            msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            return msg

        @staticmethod
        def image(stamp, pixels, encoding, pixel_bytes):
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = CAMERA_FRAME
            msg.height, msg.width = pixels.shape[:2]
            msg.encoding = encoding
            msg.is_bigendian = 0
            msg.step = msg.width * pixel_bytes
            # An array.array skips rclpy's per-byte range check (~0.5 s per
            # frame pair).
            msg.data = array.array("B", pixels.tobytes())
            return msg

        def publish_camera(self, stamp, rgb, depth):
            self.rgb_publisher.publish(self.image(stamp, rgb, "rgb8", 3))
            self.rgb_info_publisher.publish(
                self.camera_info(stamp, rgb.shape[1], rgb.shape[0]))
            self.depth_publisher.publish(self.image(stamp, depth, "32FC1", 4))
            self.depth_info_publisher.publish(
                self.camera_info(stamp, depth.shape[1], depth.shape[0]))

    return SimPublisher()


def wait_for_frame(transport, timeout_s):
    """The newest servo frame, waiting up to timeout_s for one."""
    frame = transport.receive()
    if frame is None:
        select.select([transport.socket], [], [], timeout_s)
        frame = transport.receive()
    return frame


def start_app(args):
    """Isaac Sim with its bundled ROS 2 (rclpy importable afterwards)."""
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({
        "headless": args.headless,
        # Isaac Sim 6 renders with the real-time path tracer whatever is
        # asked for here (RaytracedLighting falls back to it).
        "renderer": os.environ.get("ISAAC_RENDERER", "RealTimePathTracing"),
    })
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()
    return simulation_app


def main():
    args = parse_args()
    simulation_app = start_app(args)
    import rclpy

    asset_dir = Path(args.assets)
    ready_path = Path("/tmp/isaac_sim_ready")
    ready_path.unlink(missing_ok=True)
    physics_rate = env_float("ISAAC_PHYSICS_HZ", 1200.0)
    camera_rate = env_float("ISAAC_CAMERA_HZ", 10.0)
    world, rigid, home = build_world(asset_dir, physics_rate, camera_rate)
    import omni.usd

    tracks, target_ops = add_tracking_targets(omni.usd.get_context().get_stage(), asset_dir)
    camera = DownwardCamera()

    rclpy.init()
    publisher = make_publisher(args)
    if tracks is not None:
        publisher.subscribe_hide(tracks)
    transport = ArduPilotJSON(port=int(os.environ.get("ARDUPILOT_JSON_PORT", "9002")))
    ready_path.touch()
    previous_velocity = np.zeros(3)
    filtered_acceleration_world = np.zeros(3)
    rotor_speed = np.zeros(4)
    # Pinned ArduPilot quad-X output order: front-right, rear-left,
    # front-left, rear-right. Reaction torque is opposite propeller spin.
    rotor_positions = np.array([[0.13, -0.22], [-0.13, 0.20],
                                [0.13, 0.22], [-0.13, -0.20]])
    yaw_sign = np.array([-1.0, -1.0, 1.0, 1.0])
    thrust_coefficient = 8.54858e-6
    moment_coefficient = 0.016
    dt = 1.0 / physics_rate
    stride = render_stride(physics_rate, camera_rate)
    maximum_rotor_speed = env_float("ISAAC_MAX_ROTOR_SPEED", 1000.0)
    tick = 0
    last_frame_count = None
    state = None
    start = time.monotonic()
    report = (tick, start)

    try:
        while simulation_app.is_running() and rclpy.ok():
            if args.duration_s > 0 and time.monotonic() - start >= args.duration_s:
                break
            frame = wait_for_frame(transport, 0.01)
            if frame is None:
                continue
            if not is_new_frame(frame, last_frame_count):
                if state is not None:
                    transport.send(state)  # the answer to this frame was lost
                continue
            last_frame_count = frame.frame_count
            target_rotor_speed = pwm_to_rotor_speed(
                frame.pwm[:4], maximum_speed=maximum_rotor_speed)
            motor_tau = np.where(target_rotor_speed > rotor_speed, 0.0125, 0.025)
            rotor_speed += (target_rotor_speed - rotor_speed) * (
                1.0 - np.exp(-dt / motor_tau))
            rotor_thrust = thrust_coefficient * rotor_speed * rotor_speed
            force = np.array([[0.0, 0.0, float(rotor_thrust.sum())]])
            torque = np.array([[
                float(np.dot(rotor_positions[:, 1], rotor_thrust)),
                float(np.dot(-rotor_positions[:, 0], rotor_thrust)),
                float(np.dot(yaw_sign, moment_coefficient * rotor_thrust)),
            ]])
            rigid.apply_forces_and_torques_at_pos(
                force, torque, local_frame=True)
            world.step(render=False)  # exactly one physics step
            tick += 1

            positions, orientations = rigid.get_world_poses()
            linear, angular = rigid.get_velocities()
            position = as_numpy(positions)[0]
            quaternion = as_numpy(orientations)[0]
            velocity = as_numpy(linear)[0]
            angular_velocity_world = as_numpy(angular)[0]
            acceleration_world = (velocity - previous_velocity) / dt
            previous_velocity = velocity.copy()
            rotation = quaternion_to_matrix(quaternion)
            accel_alpha = 1.0 - math.exp(-dt / 0.015)
            filtered_acceleration_world += accel_alpha * (
                acceleration_world - filtered_acceleration_world)
            angular_velocity_flu = rotation.T @ angular_velocity_world
            specific_force_flu = rotation.T @ (
                filtered_acceleration_world - np.array([0.0, 0.0, -9.80665]))
            state = state_from_enu(
                position, velocity, quaternion, angular_velocity_flu,
                specific_force_flu, args.lat, args.lon, args.origin_alt,
                timestamp=tick * dt, reference_enu=home)
            if tick % stride == 0:
                # Render this physics state (no physics step) before ArduPilot
                # sees it, then publish the frame, its pose and time together.
                # The targets move with the simulation clock: the frame shows
                # them where the ground truth published with it says.
                if tracks is not None:
                    poses = tracks.poses(tick * dt)
                    place_tracking_targets(target_ops, tracks.placed(poses))
                frames = camera.capture(world, 0.5 * dt)
                stamp = publisher.stamp()
                publisher.publish_clock(tick * dt)
                publisher.publish_state(stamp, position.tolist(), quaternion.tolist(),
                                        velocity.tolist(), angular_velocity_world.tolist())
                if frames is not None:
                    publisher.publish_camera(stamp, *frames)
                else:
                    print(f"Isaac camera: no frame rendered for t={tick * dt:.3f} s", flush=True)
                if tracks is not None:
                    publisher.publish_targets(
                        truth_json(tick * dt, poses, tracks.origin, tracks.hidden))
                rclpy.spin_once(publisher, timeout_sec=0.0)
            transport.send(state)
            if tick % max(1, int(physics_rate * 5.0)) == 0:
                now = time.monotonic()
                print(
                    "Isaac physics "
                    f"frame_rate={frame.frame_rate_hz}Hz "
                    f"pwm={list(frame.pwm[:4])} "
                    f"position={position.round(3).tolist()} "
                    f"quaternion={quaternion.round(4).tolist()} "
                    f"real_time_factor={(tick - report[0]) * dt / max(now - report[1], 1e-6):.2f}",
                    flush=True,
                )
                report = (tick, now)
    except KeyboardInterrupt:
        pass
    finally:
        transport.close()
        publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
