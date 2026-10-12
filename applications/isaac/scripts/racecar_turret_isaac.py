#!/usr/bin/env python3
"""Isaac Sim RACECAR Turret: the car with the turret, its sensors, and an
arena with walking people, for driving (SLAM Toolbox + Nav2) and aiming at
the same time (systems/racecar_turret/interfaces.md,
applications/isaac/RACECAR_TURRET.md).

The vehicle is systems/racecar_turret/urdf/racecar_turret.urdf.xacro with
sim_vehicle:=true (expanded by racecar_turret_assets.py), imported as one
articulation: four wheels on velocity drives, two steering knuckles and the
pan/tilt joints on position drives. racecar_turret_sim.py holds the models
between ROS and the drives: the ESC and steering servo behind
/ackermann_cmd, the ST3215 servos behind /turret/joint_commands, wheel
odometry, the IMU and the lidar.

Lock-step: each loop iteration applies the drive targets, steps the physics
exactly once (World.step(render=False), 120 Hz) and publishes what is due at
that simulation time, stamped with it, and /clock. Every
ISAAC_PHYSICS_HZ / ISAAC_RENDER_HZ steps the stage is rendered once
(World.render() does not step physics). The annotators hand over each frame
one render later, so a frame reaches ROS one render period after the
simulation time it shows (67 ms at 15 Hz, about a USB camera's latency): it
is published stamped with that time (IsaacReadSimulationTime), with the
people, the car and the turret where they were then. Simulation time runs
slower than real time when the GPU cannot keep up (and never faster than
ISAAC_MAX_REAL_TIME_FACTOR); every node of the stack runs on /clock.
"""

from __future__ import annotations

import argparse
import array
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time
import traceback

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "demnav" / "scripts"))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "aerostack2" / "scripts"))
from launcher_standin import LauncherPlant
from simulation_trigger import SimulationTrigger
import racecar_turret_sim as rts  # noqa: E402
from tracking_targets import Tracks  # noqa: E402

ROBOT_PATH = "/World/RacecarTurret"
WHEEL_JOINTS = ("front_left_wheel_joint", "front_right_wheel_joint",
                "rear_left_wheel_joint", "rear_right_wheel_joint")
STEERING_JOINTS = ("front_left_steering_joint", "front_right_steering_joint")
TURRET_JOINTS = ("pan_joint", "tilt_joint")
# Hardware geometry (racecar_turret.urdf.xacro sim_vehicle, the hardware
# cmd_vel_to_ackermann.yaml): wheelbase, track, tyre radius, steering lock.
WHEELBASE, TRACK, WHEEL_RADIUS, MAX_STEERING = 0.33, 0.30, 0.055, 0.42
# The front RealSense: systems/racecarneo/calibration/camera.yaml (640 x 480,
# fx = fy = 617.1 px).
FRONT_WIDTH, FRONT_HEIGHT, FRONT_FX = 640, 480, 617.1
TURRET_WIDTH, TURRET_HEIGHT = 640, 480
# Depth: systems/racecarneo/config/sensor_models.yaml; the D455's range.
DEPTH_NOISE_STD_M, DEPTH_DROPOUT, DEPTH_MAX_M = 0.015, 0.01, 10.0
# 2D lidar: RPLIDAR class, 360 degrees in 0.5 degree steps at 10 Hz, 0.15 to
# 12 m (the 12 m of slam_toolbox.yaml and nav2.yaml); noise and dropout of
# sensor_models.yaml.
LIDAR_SAMPLES, LIDAR_HZ, LIDAR_RANGE = 720, 10.0, (0.15, 12.0)
LIDAR_NOISE_STD_M, LIDAR_DROPOUT = 0.015, 0.005
# Tyres on concrete.
TYRE_FRICTION = (1.0, 0.9)
# Drive gains in SI (N m/rad, N m s/rad). Wheels: velocity drives whose
# torque saturates at the URDF effort, the motor's; knuckles and turret:
# stiff position drives (the servos' own dynamics are modelled in
# racecar_turret_sim.py and only their output positions are driven).
WHEEL_DAMPING = 0.6
STEERING_GAINS = (60.0, 1.5)
TURRET_GAINS = (20.0, 0.4)
# Renders allowed for the first frame to reach the annotators.
MAX_WARMUP_RENDERS = 30
# Anti-aliasing at the full image resolution (DLAA), as aerodrone_navsim.py.
RENDER_SETTINGS = {"/rtx/post/aa/op": 4}
SKY_INTENSITY = 1000.0
STOP_REQUESTED = []


def env_float(name, default):
    return float(os.environ.get(name, default))


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--assets", default="/data/racecar_turret_assets",
                        help="racecar_turret_assets.py output: URDF, people, textures")
    parser.add_argument("--scenario", default="/workspace/systems/racecar_turret/scenarios/isaac_arena.json")
    parser.add_argument("--physics-hz", type=float, default=env_float("ISAAC_PHYSICS_HZ", 120.0))
    parser.add_argument("--render-hz", type=float, default=env_float("ISAAC_RENDER_HZ", 15.0))
    parser.add_argument("--camera-hz", type=float, default=env_float("ISAAC_CAMERA_HZ", 15.0))
    parser.add_argument("--imu-hz", type=float, default=env_float("ISAAC_IMU_HZ", 100.0))
    parser.add_argument("--odom-hz", type=float, default=env_float("ISAAC_ODOM_HZ", 50.0))
    parser.add_argument("--turret-hfov-deg", type=float,
                        default=env_float("ISAAC_TURRET_HFOV_DEG", 70.0))
    parser.add_argument("--max-real-time-factor", type=float,
                        default=env_float("ISAAC_MAX_REAL_TIME_FACTOR", 1.0),
                        help="never run simulation time faster than this (0: unpaced)")
    parser.add_argument("--seed", type=int, default=int(os.environ.get("ISAAC_SEED", "7")))
    parser.add_argument("--duration-s", type=float, default=env_float("ISAAC_DURATION_S", 0.0))
    parser.add_argument("--ready-file", default="/tmp/isaac_sim_ready")
    return parser.parse_known_args(argv)[0]


def strides(physics_hz, rates):
    """Physics steps per period of each rate (at least one)."""
    return {name: max(1, int(round(physics_hz / rate))) for name, rate in rates.items()}


def link_prim_paths(stage, root, joints):
    """{URDF link: prim path} of the imported vehicle under root. The
    importer encodes link names ('turret/pan_link' becomes
    'tn__turretpan_link_yL') but keeps joint names: each movable joint's
    child link is its USD joint's body1, and base_link is the articulation
    root (fixed joints are merged into their parents)."""
    from pxr import Usd, UsdPhysics

    bodies, found = {}, {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.IsA(UsdPhysics.Joint):
            bodies[prim.GetName()] = UsdPhysics.Joint(prim).GetBody1Rel().GetTargets()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            found["base_link"] = str(prim.GetPath())
    for joint in joints.values():
        if bodies.get(joint.name):
            found[joint.child] = str(bodies[joint.name][0])
    return found


# --- stage ------------------------------------------------------------------

def start_app(args):
    from isaacsim import SimulationApp

    app = SimulationApp({
        "headless": args.headless,
        "renderer": os.environ.get("ISAAC_RENDERER", "RealTimePathTracing"),
    })
    from isaacsim.core.utils.extensions import enable_extension

    for extension in ("isaacsim.asset.importer.urdf", "isaacsim.ros2.bridge"):
        enable_extension(extension)
    app.update()
    return app


def physics_material(stage, path, friction):
    from pxr import UsdPhysics, UsdShade

    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr().Set(float(friction[0]))
    api.CreateDynamicFrictionAttr().Set(float(friction[1]))
    api.CreateRestitutionAttr().Set(0.0)
    return material


def bind_physics_material(prim, material):
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, "physics")


def preview_material(stage, path, color, texture=None, roughness=0.8):
    from pxr import Sdf, UsdShade

    material = UsdShade.Material.Define(stage, path)
    surface = UsdShade.Shader.Define(stage, f"{path}/Surface")
    surface.CreateIdAttr("UsdPreviewSurface")
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    if texture:
        image = UsdShade.Shader.Define(stage, f"{path}/Texture")
        image.CreateIdAttr("UsdUVTexture")
        image.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(texture))
        image.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        for wrap in ("wrapS", "wrapT"):
            image.CreateInput(wrap, Sdf.ValueTypeNames.Token).Set("repeat")
        reader = UsdShade.Shader.Define(stage, f"{path}/Primvar")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        image.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            reader.ConnectableAPI(), "result")
        surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            image.ConnectableAPI(), "rgb")
    else:
        surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(tuple(color))
    material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
    return material


def add_floor(stage, arena, assets, friction_material):
    """The arena floor: one textured quad (2 m texture repeat) that is also
    its triangle-mesh collider, with the tyres' friction."""
    from pxr import Sdf, UsdGeom, UsdPhysics, UsdShade

    (x0, y0), (x1, y1) = arena["min_xy"], arena["max_xy"]
    margin = float(arena.get("wall_thickness_m", 0.2)) + 1.0
    x0, y0, x1, y1 = x0 - margin, y0 - margin, x1 + margin, y1 + margin
    mesh = UsdGeom.Mesh.Define(stage, "/World/Floor")
    mesh.CreatePointsAttr([(x0, y0, 0.0), (x1, y0, 0.0), (x1, y1, 0.0), (x0, y1, 0.0)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([(0.0, 0.0, 1.0)] * 4)
    mesh.CreateSubdivisionSchemeAttr("none")
    repeat = 2.0
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set([(x0 / repeat, y0 / repeat), (x1 / repeat, y0 / repeat),
            (x1 / repeat, y1 / repeat), (x0 / repeat, y1 / repeat)])
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")
    texture = Path(assets) / "textures" / "floor.png"
    material = preview_material(stage, "/World/Looks/Floor", arena.get("floor_color", (0.4, 0.4, 0.4)),
                                texture if texture.is_file() else None, roughness=0.9)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    bind_physics_material(mesh.GetPrim(), friction_material)


def add_obstacles(stage, obstacles):
    """Walls and obstacles: static colliders the lidar and the cameras see."""
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade

    UsdGeom.Xform.Define(stage, "/World/Obstacles")
    for obstacle in obstacles:
        path = f"/World/Obstacles/{obstacle.name}"
        if obstacle.kind == "box":
            geom = UsdGeom.Cube.Define(stage, path)
            geom.CreateSizeAttr(1.0)
            scale = Gf.Vec3f(*(float(v) for v in obstacle.size))
        else:
            geom = UsdGeom.Cylinder.Define(stage, path)
            geom.CreateRadiusAttr(obstacle.radius)
            geom.CreateHeightAttr(obstacle.height)
            geom.CreateAxisAttr("Z")
            scale = None
        xform = UsdGeom.Xformable(geom)
        xform.AddTranslateOp().Set(Gf.Vec3d(obstacle.center[0], obstacle.center[1],
                                            obstacle.height / 2.0))
        xform.AddRotateZOp().Set(math.degrees(obstacle.yaw))
        if scale is not None:
            xform.AddScaleOp().Set(scale)
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        material = preview_material(stage, f"/World/Looks/{obstacle.name}", obstacle.color)
        UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(material)


def add_lights(stage):
    from pxr import UsdLux

    UsdLux.DomeLight.Define(stage, "/World/Sky").CreateIntensityAttr(SKY_INTENSITY)


def add_people(stage, tracks, sizes, collision_radius):
    """Each person: an Xform moved to their track pose before every physics
    step and render, holding the baked mesh (target_models.py) and a
    kinematic capsule collider the lidar sees (purpose guide: not rendered).
    Returns [(translate op, orient op)]."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, Vt

    UsdGeom.Xform.Define(stage, "/World/People")
    materials = {}
    ops = []
    for track in tracks.tracks:
        if track.model not in materials:
            materials[track.model] = preview_material(
                stage, f"/World/Looks/Person_{track.model}", (0.6, 0.6, 0.6),
                tracks.model_file(track, "texture"), roughness=0.7)
        root = UsdGeom.Xform.Define(stage, f"/World/People/{track.name}")
        UsdPhysics.RigidBodyAPI.Apply(root.GetPrim()).CreateKinematicEnabledAttr(True)
        translate = root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
        orient = root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
        data = np.load(tracks.model_file(track, "mesh_npz"))
        positions = np.ascontiguousarray(data["positions"], dtype=np.float32)
        mesh = UsdGeom.Mesh.Define(stage, f"/World/People/{track.name}/Model")
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
        height = float(sizes[track.model][2])
        capsule = UsdGeom.Capsule.Define(stage, f"/World/People/{track.name}/Collider")
        capsule.CreateRadiusAttr(collision_radius)
        capsule.CreateHeightAttr(max(0.0, height - 2.0 * collision_radius))
        capsule.CreateAxisAttr("Z")
        capsule.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdGeom.Xformable(capsule).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height / 2.0))
        UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())
        ops.append((translate, orient))
    return ops


def place_people(ops, poses):
    from pxr import Gf

    for (translate, orient), pose in zip(ops, poses):
        translate.Set(Gf.Vec3d(pose.east, pose.north, 0.0))
        w, x, y, z = pose.quaternion_wxyz()
        orient.Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))


def import_vehicle(urdf_path, work_dir):
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    config = URDFImporterConfig(
        urdf_path=str(urdf_path), usd_path=str(work_dir),
        # Sensor frames are massless: merge them into their bodies (their
        # poses come from the URDF, racecar_turret_sim.fixed_transform).
        merge_fixed_joints=True, merge_mesh=False, collision_from_visuals=False,
        allow_self_collision=False, fix_base=False)
    usd = URDFImporter(config).import_urdf()
    if not usd:
        raise RuntimeError(f"failed to import {urdf_path}")
    return usd


def disable_ground_reaching_collisions(stage, base_path, clearance=0.02):
    """racecarneo.urdf.xacro's base_link box is the car's envelope down to
    the ground; turn off every collision of the base body that reaches below
    `clearance` (racecar_turret.urdf.xacro's chassis box stays)."""
    from pxr import Usd, UsdGeom, UsdPhysics

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_,
                                                       UsdGeom.Tokens.guide])
    disabled = []
    base = stage.GetPrimAtPath(base_path)
    prims = iter(Usd.PrimRange(base))
    for prim in prims:
        if prim != base and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prims.PruneChildren()  # another link (a wheel, the turret)
            continue
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        low = cache.ComputeWorldBound(prim).ComputeAlignedRange().GetMin()
        if low[2] < clearance:
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
            disabled.append(str(prim.GetPath()))
    return disabled


def bind_tyres(stage, paths, material):
    from pxr import Usd, UsdPhysics

    for path in paths:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                bind_physics_material(prim, material)


def add_camera(stage, path, translation, fx, width, height):
    """A USD camera at a body frame (x forward, y left, z up) rendering
    width x height with focal length fx pixels."""
    from pxr import Gf, UsdGeom

    focal, horizontal, vertical = rts.usd_apertures(width, height, fx)
    camera = UsdGeom.Camera.Define(stage, path)
    camera.GetFocalLengthAttr().Set(focal)
    camera.GetHorizontalApertureAttr().Set(horizontal)
    camera.GetVerticalApertureAttr().Set(vertical)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 100.0))
    xform = UsdGeom.Xformable(camera)
    xform.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in translation)))
    w, x, y, z = rts.USD_CAMERA_FROM_BODY_WXYZ
    xform.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))
    return path


class CameraRig:
    """Render products of the front RGB-D and turret cameras, rendered
    together, and the simulation time of the frame their annotators hold."""

    def __init__(self, front_path, turret_path):
        import omni.replicator.core as rep

        self.front = rep.create.render_product(front_path, (FRONT_WIDTH, FRONT_HEIGHT))
        self.turret = rep.create.render_product(turret_path, (TURRET_WIDTH, TURRET_HEIGHT))
        get = rep.AnnotatorRegistry.get_annotator
        self.front_rgb, self.front_depth = get("rgb"), get("distance_to_image_plane")
        self.turret_rgb = get("rgb")
        self.front_time, self.turret_time = get("IsaacReadSimulationTime"), get("IsaacReadSimulationTime")
        for annotator in (self.front_rgb, self.front_depth, self.front_time):
            annotator.attach([self.front])
        for annotator in (self.turret_rgb, self.turret_time):
            annotator.attach([self.turret])

    @staticmethod
    def _time(annotator):
        try:
            return float(annotator.get_data()["simulationTime"])
        except (KeyError, TypeError, ValueError):
            return None

    def frames(self):
        """(time, front rgb, front depth, turret rgb) of the frame the
        annotators hold, or None if they disagree or hold none yet."""
        front_t, turret_t = self._time(self.front_time), self._time(self.turret_time)
        if front_t is None or turret_t is None or abs(front_t - turret_t) > 1e-6:
            return None
        front = np.asarray(self.front_rgb.get_data())
        depth = np.asarray(self.front_depth.get_data())
        turret = np.asarray(self.turret_rgb.get_data())
        if front.ndim != 3 or turret.ndim != 3 or depth.ndim < 2:
            return None
        return (front_t, np.ascontiguousarray(front[..., :3]),
                np.asarray(depth, dtype=np.float32).reshape(depth.shape[:2]),
                np.ascontiguousarray(turret[..., :3]))


# --- ROS --------------------------------------------------------------------

def make_ros_node(turret_hfov_deg):
    """The simulator's ROS 2 node: sensor and truth publishers, the drive
    and turret command subscriptions (racecarneo_ros_bridge.py's topics,
    frames and units for the car)."""
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
    from std_msgs.msg import Bool, Float32, String

    latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

    class RacecarTurretSim(Node):
        def __init__(self):
            super().__init__("racecar_turret_isaac")
            self.lock = threading.Lock()
            self.commands = []
            self.turret_commands = []
            self.launcher = LauncherPlant()
            self.simulated_operator = SimulationTrigger(action='tag')
            self.auto_fire = os.environ.get('SIM_AUTO_FIRE', 'true').lower() in ('true', '1', 'yes')
            self.simulation_time = 0.0
            self.emergency_stop = False
            pub = self.create_publisher
            self.clock_pub = pub(Clock, "/clock", 10)
            self.color_pub = pub(Image, "/camera/color/image_raw", 5)
            self.color_info_pub = pub(CameraInfo, "/camera/color/camera_info", 5)
            self.depth_pub = pub(Image, "/camera/depth/image_rect_raw", 5)
            self.depth_info_pub = pub(CameraInfo, "/camera/depth/camera_info", 5)
            self.scan_pub = pub(LaserScan, "/scan", 5)
            self.imu_pub = pub(Imu, "/imu/data_raw", 20)
            self.odom_pub = pub(Odometry, "/odom", 10)
            self.feedback_pub = pub(AckermannDriveStamped, "/ackermann_feedback", 10)
            self.manual_pub = pub(Bool, "/racecarneo/manual_override", 5)
            self.estop_pub = pub(Bool, "/racecarneo/estop", 5)
            self.battery_pub = pub(Float32, "/racecarneo/battery_voltage", 5)
            self.joint_pub = pub(JointState, "/turret/joint_states", 20)
            self.ready_pub = pub(Bool, "/turret/actuator_ready", latched)
            self.shots_pub = pub(String, '/simulation/launcher/shots', 10)
            self.operator_pub = pub(String, '/turret/command', 10)
            self.turret_image_pub = pub(Image, "/turret/camera/image_raw", 5)
            self.turret_info_pub = pub(CameraInfo, "/turret/camera/camera_info", 5)
            self.truth_pub = pub(String, rts.TRUTH_TOPIC, 10)
            self.create_subscription(AckermannDriveStamped, "/ackermann_cmd", self._ackermann, 10)
            self.create_subscription(JointState, "/turret/joint_commands", self._joint_command, 10)
            self.create_subscription(String, '/turret/status', self._status, 10)
            for qos in (latched, 10):
                self.create_subscription(Bool, "/turret/emergency_stop", self._estop, qos)
            self.front_info = rts.camera_info_values(
                FRONT_WIDTH, FRONT_HEIGHT, *rts.pinhole(FRONT_WIDTH, FRONT_HEIGHT, FRONT_FX))
            turret_fx = rts.fx_of_hfov(TURRET_WIDTH, turret_hfov_deg)
            self.turret_info = rts.camera_info_values(
                TURRET_WIDTH, TURRET_HEIGHT, *rts.pinhole(TURRET_WIDTH, TURRET_HEIGHT, turret_fx))
            ready = Bool()
            ready.data = True
            self.ready_pub.publish(ready)

        # The callbacks run on the executor's thread (spin_in_background);
        # the simulation loop takes what arrived with drain().
        def _ackermann(self, msg):
            with self.lock:
                self.commands.append((float(msg.drive.speed), float(msg.drive.steering_angle)))

        def _joint_command(self, msg):
            positions = dict(zip(msg.name, msg.position))
            if "pan_joint" in positions and "tilt_joint" in positions:
                with self.lock:
                    self.turret_commands.append((float(positions["pan_joint"]),
                                                 float(positions["tilt_joint"]),
                                                 positions.get('actuator_joint'),
                                                 positions.get('trigger_joint')))

        def _status(self, msg):
            if not self.auto_fire:
                return
            try:
                status = json.loads(msg.data)
            except (TypeError, ValueError):
                return
            command = self.simulated_operator.command(status, self.simulation_time)
            if command is not None:
                self.operator_pub.publish(String(data=json.dumps(command)))

        def _estop(self, msg):
            self.emergency_stop = bool(msg.data)

        def drain(self):
            """(ackermann commands, turret commands) received since the last call."""
            with self.lock:
                commands, self.commands = self.commands, []
                turret, self.turret_commands = self.turret_commands, []
            return commands, turret

        @staticmethod
        def stamp(seconds):
            from builtin_interfaces.msg import Time

            nanoseconds = int(round(seconds * 1e9))
            return Time(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)

        def publish_clock(self, seconds):
            self.simulation_time = seconds
            msg = Clock()
            msg.clock = self.stamp(seconds)
            self.clock_pub.publish(msg)

        @staticmethod
        def image(stamp, frame_id, pixels, encoding, pixel_bytes):
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = frame_id
            msg.height, msg.width = pixels.shape[:2]
            msg.encoding = encoding
            msg.is_bigendian = 0
            msg.step = msg.width * pixel_bytes
            # An array.array skips rclpy's per-byte range check.
            msg.data = array.array("B", np.ascontiguousarray(pixels).tobytes())
            return msg

        @staticmethod
        def camera_info(stamp, frame_id, values):
            msg = CameraInfo()
            msg.header.stamp = stamp
            msg.header.frame_id = frame_id
            msg.width, msg.height = values["width"], values["height"]
            msg.distortion_model = values["distortion_model"]
            msg.d, msg.k, msg.r, msg.p = values["d"], values["k"], values["r"], values["p"]
            return msg

        def publish_front(self, seconds, rgb, depth):
            stamp = self.stamp(seconds)
            color_frame = "racecarneo/camera_color_optical_frame"
            depth_frame = "racecarneo/camera_depth_optical_frame"
            # bgr8, as racecarneo_ros_bridge.py publishes it.
            self.color_pub.publish(self.image(stamp, color_frame, rgb[..., ::-1], "bgr8", 3))
            self.color_info_pub.publish(self.camera_info(stamp, color_frame, self.front_info))
            self.depth_pub.publish(self.image(stamp, depth_frame, depth, "32FC1", 4))
            self.depth_info_pub.publish(self.camera_info(stamp, depth_frame, self.front_info))

        def publish_turret_camera(self, seconds, rgb):
            stamp = self.stamp(seconds)
            frame = "turret/camera_optical_frame"
            self.turret_image_pub.publish(self.image(stamp, frame, rgb, "rgb8", 3))
            self.turret_info_pub.publish(self.camera_info(stamp, frame, self.turret_info))

        def publish_scan(self, seconds, ranges):
            msg = LaserScan()
            msg.header.stamp = self.stamp(seconds)
            msg.header.frame_id = "racecarneo/lidar_link"
            msg.angle_min = -math.pi
            msg.angle_increment = 2.0 * math.pi / len(ranges)
            msg.angle_max = msg.angle_min + (len(ranges) - 1) * msg.angle_increment
            msg.time_increment = 0.0
            msg.scan_time = 1.0 / LIDAR_HZ
            msg.range_min, msg.range_max = LIDAR_RANGE
            msg.ranges = array.array("f", ranges.astype(np.float32).tobytes())
            self.scan_pub.publish(msg)

        def publish_imu(self, seconds, accel, gyro, accel_std, gyro_std):
            msg = Imu()
            msg.header.stamp = self.stamp(seconds)
            msg.header.frame_id = "racecarneo/imu_link"
            msg.orientation_covariance[0] = -1.0
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
                float(v) for v in accel)
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
                float(v) for v in gyro)
            msg.linear_acceleration_covariance = [accel_std ** 2, 0.0, 0.0, 0.0, accel_std ** 2,
                                                  0.0, 0.0, 0.0, accel_std ** 2]
            msg.angular_velocity_covariance = [gyro_std ** 2, 0.0, 0.0, 0.0, gyro_std ** 2,
                                               0.0, 0.0, 0.0, gyro_std ** 2]
            self.imu_pub.publish(msg)

        def publish_odometry(self, seconds, pose, steering):
            x, y, yaw, speed, yaw_rate = pose
            stamp = self.stamp(seconds)
            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"
            msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
            msg.pose.pose.orientation.z = math.sin(0.5 * yaw)
            msg.pose.pose.orientation.w = math.cos(0.5 * yaw)
            msg.twist.twist.linear.x = speed
            msg.twist.twist.angular.z = yaw_rate
            msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05
            msg.pose.covariance[35] = 0.2
            msg.twist.covariance[0] = 0.1
            msg.twist.covariance[35] = 0.2
            self.odom_pub.publish(msg)
            feedback = AckermannDriveStamped()
            feedback.header.stamp = stamp
            feedback.header.frame_id = "base_link"
            feedback.drive.speed = float(speed)
            feedback.drive.steering_angle = float(steering)
            self.feedback_pub.publish(feedback)
            for publisher, value in ((self.manual_pub, False), (self.estop_pub, False)):
                flag = Bool()
                flag.data = value
                publisher.publish(flag)
            battery = Float32()
            battery.data = float(11.1 - 0.8 * min(1.0, abs(speed)))
            self.battery_pub.publish(battery)

        def publish_joints(self, seconds, pan, tilt):
            msg = JointState()
            msg.header.stamp = self.stamp(seconds)
            msg.name = list(TURRET_JOINTS)
            msg.position = [float(pan), float(tilt)]
            msg.name.extend(['actuator_joint', 'trigger_joint'])
            msg.position.extend([float(self.launcher.rack), float(self.launcher.trigger)])
            self.joint_pub.publish(msg)
            self.ready_pub.publish(Bool(data=not self.launcher.estop))

        def publish_truth(self, message):
            msg = String()
            msg.data = json.dumps(message, separators=(",", ":"))
            self.truth_pub.publish(msg)

    return RacecarTurretSim()


# --- simulation -------------------------------------------------------------

class Vehicle:
    """The imported articulation and its drives."""

    def __init__(self, world, stage, urdf_text, usd_path, spawn, tyre_material):
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import add_reference_to_stage

        add_reference_to_stage(usd_path=usd_path, prim_path=ROBOT_PATH)
        self.joints = rts.urdf_joints(urdf_text)
        self.links = link_prim_paths(stage, ROBOT_PATH, self.joints)
        missing = [link for link in ("base_link", "turret/tilt_link") if link not in self.links]
        if missing:
            raise RuntimeError(f"imported vehicle lacks links {missing}")
        disabled = disable_ground_reaching_collisions(stage, self.links["base_link"])
        print(f"Isaac vehicle: disabled ground-reaching collisions {disabled}", flush=True)
        bind_tyres(stage, [path for link, path in self.links.items() if link.endswith("wheel_link")],
                   tyre_material)
        self.articulation = world.scene.add(SingleArticulation(prim_path=ROBOT_PATH,
                                                               name="racecar_turret"))
        self.spawn = spawn
        self.indices = {}

    def initialize(self):
        from isaacsim.core.utils.types import ArticulationAction  # noqa: F401

        names = list(self.articulation.dof_names)
        self.indices = {name: names.index(name) for name in names}
        missing = [j for j in WHEEL_JOINTS + STEERING_JOINTS + TURRET_JOINTS if j not in self.indices]
        if missing:
            raise RuntimeError(f"articulation lacks joints {missing}: {names}")
        controller = self.articulation.get_articulation_controller()
        kps = np.zeros(len(names), dtype=np.float32)
        kds = np.zeros(len(names), dtype=np.float32)
        for joint in WHEEL_JOINTS:
            kds[self.indices[joint]] = WHEEL_DAMPING
        for joint in STEERING_JOINTS:
            kps[self.indices[joint]], kds[self.indices[joint]] = STEERING_GAINS
        for joint in TURRET_JOINTS:
            kps[self.indices[joint]], kds[self.indices[joint]] = TURRET_GAINS
        # Any other joint (none in racecar_turret_sim.urdf) is held at zero.
        self.others = np.array([index for name, index in self.indices.items()
                                if name not in WHEEL_JOINTS + STEERING_JOINTS + TURRET_JOINTS],
                               dtype=np.int64)
        kps[self.others], kds[self.others] = STEERING_GAINS
        controller.set_gains(kps=kps, kds=kds)
        spawn = self.spawn
        half = 0.5 * spawn.yaw
        # The wheels' lowest points are at base_link's z = 0, on the floor.
        self.articulation.set_world_pose(
            position=np.array([spawn.x, spawn.y, 0.002]),
            orientation=np.array([math.cos(half), 0.0, 0.0, math.sin(half)]))
        self.wheels = np.array([self.indices[j] for j in WHEEL_JOINTS])
        self.steering = np.array([self.indices[j] for j in STEERING_JOINTS])
        self.turret = np.array([self.indices[j] for j in TURRET_JOINTS])
        print(f"Isaac vehicle: dofs {names}, gains {controller.get_gains()}", flush=True)

    def apply(self, wheel_rates, knuckles, turret):
        from isaacsim.core.utils.types import ArticulationAction

        self.articulation.apply_action(ArticulationAction(
            joint_velocities=np.asarray(wheel_rates, dtype=np.float32), joint_indices=self.wheels))
        self.articulation.apply_action(ArticulationAction(
            joint_positions=np.asarray(list(knuckles) + list(turret) + [0.0] * len(self.others),
                                       dtype=np.float32),
            joint_indices=np.concatenate([self.steering, self.turret, self.others])))

    def state(self):
        """World pose (position, rotation matrix), linear and angular velocity
        of base_link, joint positions and velocities."""
        position, orientation = self.articulation.get_world_pose()
        return (np.asarray(position, dtype=np.float64),
                rts.quat_wxyz_matrix(np.asarray(orientation, dtype=np.float64)),
                np.asarray(self.articulation.get_linear_velocity(), dtype=np.float64),
                np.asarray(self.articulation.get_angular_velocity(), dtype=np.float64),
                np.asarray(self.articulation.get_joint_positions(), dtype=np.float64),
                np.asarray(self.articulation.get_joint_velocities(), dtype=np.float64))


class Lidar:
    """The 2D lidar at racecarneo/lidar_link: its rays cast with PhysX scene
    queries from the lidar's world pose (720 rays take about 9 ms). Isaac
    6.0.1's experimental raycast sensor returned one range for every ray
    here, so it is not used."""

    def __init__(self, mount):
        from omni.physx import get_physx_scene_query_interface

        self.mount = mount
        self.directions = rts.scan_directions(LIDAR_SAMPLES)
        self.queries = get_physx_scene_query_interface()

    def depths(self, base_position, base_rotation):
        """Range of each ray (inf: no hit within range)."""
        import carb

        rotation = base_rotation @ self.mount[:3, :3]
        origin = carb.Float3(*(base_position + base_rotation @ self.mount[:3, 3]))
        result = np.full(LIDAR_SAMPLES, np.inf)
        for index, direction in enumerate(self.directions @ rotation.T):
            hit = self.queries.raycast_closest(origin, carb.Float3(*direction), LIDAR_RANGE[1])
            if hit["hit"]:
                result[index] = hit["distance"]
        return result


def spin_in_background(node):
    """Serve the node's subscriptions on a thread: the simulation loop
    never waits for ROS, and commands arrive whatever its rate."""
    from rclpy.executors import SingleThreadedExecutor

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, name="ros", daemon=True).start()
    return executor


def turret_camera_pose(joints, base_position, base_rotation, pan, tilt):
    """World (position, forward, up) of turret/camera_link at the joints."""
    base = np.eye(4)
    base[:3, :3], base[:3, 3] = base_rotation, base_position
    camera = base @ rts.chain_transform(joints, "base_link", "turret/camera_link",
                                        {"pan_joint": pan, "tilt_joint": tilt})
    return camera[:3, 3], camera[:3, 0], camera[:3, 2]


def _stop_on_signal(signum, frame):
    # Only set a flag: an exception raised here could land inside a Kit
    # update callback, which Kit logs and swallows.
    STOP_REQUESTED.append(signum)


def main():
    args = parse_args()
    app = start_app(args)
    import rclpy

    ready_path = Path(args.ready_file)
    ready_path.unlink(missing_ok=True)
    assets = Path(args.assets)
    manifest = json.loads((assets / "manifest.json").read_text(encoding="utf-8"))
    scenario = rts.load_scenario(args.scenario)
    spawn_spec = scenario.get("spawn", {})
    spawn = rts.Spawn(spawn_spec.get("x", 0.0), spawn_spec.get("y", 0.0),
                      math.radians(float(spawn_spec.get("yaw_deg", 0.0))))
    obstacles = rts.scenario_obstacles(scenario)
    tracks = Tracks.load(assets / "tracking_targets.json")
    sizes = {key: model["size_m"] for key, model in manifest["models"].items()}
    collision_radius = float(scenario["people"].get("collision_radius_m", 0.22))
    footprint = scenario.get("vehicle_footprint_m", (0.50, 0.34))
    urdf_path = assets / manifest["urdf"]
    urdf_text = urdf_path.read_text(encoding="utf-8")

    import carb
    import omni.usd
    from isaacsim.core.api import World

    dt = 1.0 / args.physics_hz
    world = World(physics_dt=dt, rendering_dt=1.0 / args.render_hz, stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    tyre_material = physics_material(stage, "/World/Physics/Tyre", TYRE_FRICTION)
    add_floor(stage, scenario["arena"], assets, tyre_material)
    add_obstacles(stage, obstacles)
    add_lights(stage)
    people_ops = add_people(stage, tracks, sizes, collision_radius)
    place_people(people_ops, tracks.poses(0.0))
    vehicle = Vehicle(world, stage, urdf_text, import_vehicle(urdf_path, "/tmp/racecar_turret_usd"),
                      spawn, tyre_material)

    joints = vehicle.joints
    base_link = vehicle.links["base_link"]
    front = rts.fixed_transform(joints, "base_link", "racecarneo/camera_link")
    lidar = rts.fixed_transform(joints, "base_link", "racecarneo/lidar_link")
    tilt_to_camera = rts.fixed_transform(joints, "turret/tilt_link", "turret/camera_link")
    front_path = add_camera(stage, f"{base_link}/front_camera", front[:3, 3], FRONT_FX,
                            FRONT_WIDTH, FRONT_HEIGHT)
    turret_path = add_camera(stage, f"{vehicle.links['turret/tilt_link']}/turret_camera",
                             tilt_to_camera[:3, 3], rts.fx_of_hfov(TURRET_WIDTH, args.turret_hfov_deg),
                             TURRET_WIDTH, TURRET_HEIGHT)
    scan = Lidar(lidar)

    world.reset()
    vehicle.initialize()
    settings = carb.settings.get_settings()
    for key, value in RENDER_SETTINGS.items():
        settings.set(key, value)
    cameras = CameraRig(front_path, turret_path)

    rclpy.init()
    node = make_ros_node(args.turret_hfov_deg)
    executor = spin_in_background(node)
    signal.signal(signal.SIGTERM, _stop_on_signal)
    rng = np.random.default_rng(args.seed)
    drive = rts.DriveTrain(WHEELBASE, TRACK, WHEEL_RADIUS, max_steering=MAX_STEERING)
    odometry = rts.WheelOdometry(WHEELBASE, WHEEL_RADIUS, rng=rng)
    imu = rts.ImuModel(rng=rng)
    turret = rts.TurretActuator()
    collisions = rts.CollisionCounter(*footprint)
    walkers = rts.Walkers(tracks, collision_radius)
    position, rotation = vehicle.state()[:2]
    yaw = rts.yaw_of(rotation)
    every = strides(args.physics_hz, {"render": args.render_hz, "camera": args.camera_hz,
                                      "imu": args.imu_hz, "odom": args.odom_hz,
                                      "lidar": LIDAR_HZ, "truth": args.camera_hz})
    camera_period = 1.0 / args.camera_hz
    selected = scenario.get("selected_target")
    turret_fx = rts.fx_of_hfov(TURRET_WIDTH, args.turret_hfov_deg)
    intrinsics = rts.pinhole(TURRET_WIDTH, TURRET_HEIGHT, turret_fx)
    print(f"Isaac RACECAR Turret: physics {args.physics_hz:g} Hz, render {args.render_hz:g} Hz, "
          f"cameras {args.camera_hz:g} Hz, steps per period {every}, target {selected}", flush=True)

    # Warm up the renderer until the annotators deliver frames.
    for _ in range(MAX_WARMUP_RENDERS):
        world.render()
        if cameras.frames() is not None:
            break
    ready_path.write_text("ready\n")
    tick = 0
    last_camera_t = -math.inf
    start_wall = time.monotonic()
    report = (0.0, start_wall)
    try:
        while app.is_running() and rclpy.ok() and not STOP_REQUESTED:
            t = tick * dt
            if args.duration_s > 0 and t >= args.duration_s:
                break
            commands, turret_commands = node.drain()
            for speed, steering in commands:
                if not drive.command(t, speed, steering):
                    node.get_logger().warn("Ignoring non-finite ackermann_cmd",
                                           throttle_duration_sec=1.0)
            node.launcher.emergency_stop(node.emergency_stop)
            for pan, tilt, rack, trigger in turret_commands:
                turret.command(t, pan, tilt)
                node.launcher.command(t, rack, trigger)
            shot = node.launcher.update(t, dt)
            if shot is not None:
                from std_msgs.msg import String
                node.shots_pub.publish(String(data=json.dumps({'stamp': shot, 'shot': len(node.launcher.shots)})))
            turret.emergency_stop = node.emergency_stop
            speed_cmd, steering_cmd = drive.update(t, dt)
            pan_goal, tilt_goal = turret.update(t, dt)
            vehicle.apply(drive.wheel_rates(), drive.knuckle_angles(), (pan_goal, tilt_goal))
            poses = walkers.step(dt, position, yaw, *footprint)
            place_people(people_ops, poses)
            world.step(render=False)  # exactly one physics step
            tick += 1
            t = tick * dt

            position, rotation, velocity, angular, q, qd = vehicle.state()
            imu.update(dt, velocity)
            pan, tilt = q[vehicle.turret]
            turret.record(t, pan, tilt)
            rear = 0.5 * (qd[vehicle.indices["rear_left_wheel_joint"]]
                          + qd[vehicle.indices["rear_right_wheel_joint"]])
            steering_measured = rts.bicycle_steering(*q[vehicle.steering], WHEELBASE, TRACK)
            odom_pose = odometry.update(dt, rear, steering_measured)
            yaw = rts.yaw_of(rotation)
            people = [(p.name, (p.east, p.north), collision_radius) for p in poses]
            collisions.update(t, position, yaw, obstacles, people)

            node.publish_clock(t)
            feedback = turret.feedback(t)
            if feedback is not None:
                node.publish_joints(*feedback)
            if tick % every["imu"] == 0:
                accel, gyro = imu.measure(rotation, angular)
                node.publish_imu(t, accel, gyro, imu.accel_noise_std, imu.gyro_noise_std)
            if tick % every["odom"] == 0:
                node.publish_odometry(t, odom_pose, steering_measured)
            if tick % every["lidar"] == 0:
                depths = scan.depths(position, rotation)
                node.publish_scan(t, rts.scan_ranges(depths, *LIDAR_RANGE, LIDAR_NOISE_STD_M,
                                                     LIDAR_DROPOUT, rng))
            if tick % every["truth"] == 0:
                camera = turret_camera_pose(joints, position, rotation, pan, tilt)
                occluders = obstacles + [
                    rts.Obstacle(p.name, "cylinder", (p.east, p.north), radius=collision_radius,
                                 height=float(sizes[p.model][2])) for p in poses]
                targets = []
                for pose in poses:
                    height = float(sizes[pose.model][2])
                    base = np.array([pose.east, pose.north, 0.0])
                    centroid = base + np.array([0.0, 0.0, height / 2.0])
                    others = [o for o in occluders if o.name != pose.name]
                    pixel = rts.project(centroid, *camera, *intrinsics)
                    targets.append({
                        "name": pose.name, "class": pose.cls,
                        "position": spawn.point(base), "centroid": spawn.point(centroid),
                        "visible": rts.visible_fraction(camera[0], base, height, others) >= 0.5,
                        "in_view": pixel is not None and 0 <= pixel[0] < TURRET_WIDTH
                        and 0 <= pixel[1] < TURRET_HEIGHT,
                    })
                odom_xy = spawn.point(position)
                node.publish_truth(rts.truth_message(
                    t, (odom_xy[0], odom_xy[1], spawn.yaw_of(yaw), float(velocity @ rotation[:, 0])),
                    (spawn.point(camera[0]), spawn.vector(camera[1]), spawn.vector(camera[2])),
                    targets, collisions.count))
            if tick % every["render"] == 0:
                world.render()
                frames = cameras.frames()
                if frames is not None and frames[0] >= last_camera_t + camera_period - 0.5 * dt:
                    frame_t, front_rgb, depth, turret_rgb = frames
                    last_camera_t = frame_t
                    node.publish_front(frame_t, front_rgb, rts.depth_measurement(
                        depth, DEPTH_NOISE_STD_M, DEPTH_DROPOUT, rng, DEPTH_MAX_M))
                    node.publish_turret_camera(frame_t, turret_rgb)
            if args.max_real_time_factor > 0:
                ahead = t / args.max_real_time_factor - (time.monotonic() - start_wall)
                if ahead > 0:
                    time.sleep(ahead)
            if tick % int(args.physics_hz * 10) == 0:
                now = time.monotonic()
                print(f"Isaac RACECAR Turret t={t:.1f}s rtf={(t - report[0]) / max(now - report[1], 1e-6):.2f} "
                      f"pose=({position[0]:.2f},{position[1]:.2f},{math.degrees(yaw):.0f}deg) "
                      f"speed={float(velocity @ rotation[:, 0]):.2f} cmd=({speed_cmd:.2f},{steering_cmd:.2f}) "
                      f"odom=({odom_pose[0]:.2f},{odom_pose[1]:.2f}) turret=({pan:.2f},{tilt:.2f}) "
                      f"collisions={collisions.count}", flush=True)
                report = (t, now)
    except KeyboardInterrupt:
        pass
    except BaseException as exc:
        print(f"Isaac RACECAR Turret aborted by {type(exc).__name__}: {exc!r}", flush=True)
        traceback.print_exc()
        raise
    finally:
        ready_path.unlink(missing_ok=True)
        print(f"Isaac RACECAR Turret stopped at t={tick * dt:.1f}s, collisions {collisions.events}, "
              f"people waited for the car {[round(w, 1) for w in walkers.waited]} s", flush=True)
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        app.close()


if __name__ == "__main__":
    main()
