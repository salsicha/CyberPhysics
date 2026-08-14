#!/usr/bin/env python3
"""Isaac Sim terrain, rotor physics, sensors, and ArduPilot JSON backend."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
from isaacsim import SimulationApp


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


args = parse_args()
simulation_app = SimulationApp({
    "headless": args.headless,
    "renderer": os.environ.get("ISAAC_RENDERER", "RaytracedLighting"),
})

import omni.graph.core as og
import omni.kit.commands
import omni.usd
import usdrt
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.utils.extensions import enable_extension
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node

sys.path.insert(0, "/workspace/applications/ardupilot_sitl/scripts")
from ardupilot_json import (  # noqa: E402
    ArduPilotJSON,
    latlon_from_enu,
    pwm_to_rotor_speed,
    quaternion_to_matrix,
    state_from_enu,
)


CAMERA_PATH = "/World/NavDrone/downward_camera"
CAMERA_GRAPH = "/World/ROS2CameraGraph"


def as_numpy(value):
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value, dtype=np.float64)


def add_terrain(stage, asset_dir):
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


def add_camera(stage, tick_rate):
    camera = UsdGeom.Camera.Define(stage, CAMERA_PATH)
    # Isaac Sim 6 uses this authored sensor rate to gate render-product work.
    # Without it, OnPlaybackTick can drive the ROS writers at the 1200 Hz
    # flight-physics rate even when World.step(render=False) is requested.
    camera.GetPrim().CreateAttribute(
        "omni:sensor:tickRate", Sdf.ValueTypeNames.Float).Set(tick_rate)
    camera.GetHorizontalApertureAttr().Set(21.0)
    camera.GetVerticalApertureAttr().Set(15.75)
    camera.GetFocalLengthAttr().Set(18.2)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.3, 300.0))
    xform = UsdGeom.Xformable(camera)
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.08))


def create_camera_graph():
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": CAMERA_GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("createRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("rgbInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("depthInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "createRenderProduct.inputs:execIn"),
                *[("createRenderProduct.outputs:execOut", f"{name}.inputs:execIn")
                  for name in ("rgb", "rgbInfo", "depth", "depthInfo")],
                *[("createRenderProduct.outputs:renderProductPath",
                   f"{name}.inputs:renderProductPath")
                  for name in ("rgb", "rgbInfo", "depth", "depthInfo")],
            ],
            keys.SET_VALUES: [
                ("createRenderProduct.inputs:cameraPrim",
                 [usdrt.Sdf.Path(CAMERA_PATH)]),
                ("createRenderProduct.inputs:width", 640),
                ("createRenderProduct.inputs:height", 480),
                ("rgb.inputs:frameId", "downward_camera_optical_frame"),
                ("rgb.inputs:topicName", args.rgb_topic),
                ("rgb.inputs:type", "rgb"),
                ("rgbInfo.inputs:frameId", "downward_camera_optical_frame"),
                ("rgbInfo.inputs:topicName", args.rgb_info_topic),
                ("depth.inputs:frameId", "downward_camera_optical_frame"),
                ("depth.inputs:topicName", args.depth_topic),
                ("depth.inputs:type", "depth"),
                ("depthInfo.inputs:frameId", "downward_camera_optical_frame"),
                ("depthInfo.inputs:topicName", args.depth_info_topic),
                *[(f"{name}.inputs:useSystemTime", True)
                  for name in ("rgb", "rgbInfo", "depth", "depthInfo")],
            ],
        },
    )


class TruthPublisher(Node):
    def __init__(self):
        super().__init__("isaac_sim_ground_truth")
        self.publisher = self.create_publisher(
            Odometry, "/aerodrone/simulation/ground_truth/odom", 10)

    def publish_state(self, position, quaternion, velocity, angular):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "earth"
        msg.child_frame_id = "aerodrone/base_link"
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = position
        msg.pose.pose.orientation = Quaternion(
            w=float(quaternion[0]), x=float(quaternion[1]),
            y=float(quaternion[2]), z=float(quaternion[3]))
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = velocity
        msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = angular
        self.publisher.publish(msg)


def main():
    asset_dir = Path(args.assets)
    manifest = json.loads((asset_dir / "manifest.json").read_text(encoding="utf-8"))
    home = np.asarray(manifest["home_enu_m"], dtype=np.float64)
    ready_path = Path("/tmp/isaac_sim_ready")
    ready_path.unlink(missing_ok=True)
    physics_rate = env_float("ISAAC_PHYSICS_HZ", 1200.0)
    camera_rate = env_float("ISAAC_CAMERA_HZ", 10.0)
    world = World(physics_dt=1.0 / physics_rate,
                  rendering_dt=1.0 / camera_rate,
                  stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    add_terrain(stage, asset_dir)
    omni.kit.commands.execute("CreatePrim", prim_path="/World/Sun",
                              prim_type="DistantLight",
                              attributes={"inputs:intensity": 3500.0, "inputs:angle": 0.8})
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
        scale=np.array([0.47, 0.47, 0.11]), mass=1.5,
        color=np.array([0.04, 0.18, 0.72])))
    # Match the passive rigid-body losses present in the Gazebo rotor model.
    # A perfectly lossless cuboid is not a credible airframe and makes the
    # stock ArduCopter rate loop unrealistically underdamped.
    physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(
        stage.GetPrimAtPath("/World/NavDrone"))
    physx_body.CreateLinearDampingAttr().Set(0.15)
    physx_body.CreateAngularDampingAttr().Set(0.20)
    add_camera(stage, camera_rate)
    rigid = RigidPrim("/World/NavDrone", masses=[1.5])
    world.reset()
    create_camera_graph()

    rclpy.init()
    truth = TruthPublisher()
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
    render_stride = max(1, int(round(physics_rate / camera_rate)))
    maximum_rotor_speed = env_float("ISAAC_MAX_ROTOR_SPEED", 1000.0)
    tick = 0
    start = time.monotonic()

    try:
        while simulation_app.is_running() and rclpy.ok():
            rclpy.spin_once(truth, timeout_sec=0.0)
            frame = transport.receive()
            if frame is None:
                world.step(render=(tick % render_stride == 0))
                tick += 1
                time.sleep(dt)
                continue
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
            render = tick % render_stride == 0
            world.step(render=render)

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
            transport.send(state_from_enu(
                position, velocity, quaternion, angular_velocity_flu,
                specific_force_flu, args.lat, args.lon, args.origin_alt,
                timestamp=tick * dt, reference_enu=home))
            if render:
                truth.publish_state(position.tolist(), quaternion.tolist(),
                                    velocity.tolist(), angular_velocity_world.tolist())
            if tick % max(1, int(physics_rate * 5.0)) == 0:
                print(
                    "Isaac physics "
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
        truth.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
