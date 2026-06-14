#!/usr/bin/env python3
"""Isaac Sim sensor bridge for AS2, DemNav, and WildNav testing."""

import argparse
import math
import os

import numpy as np
from isaacsim import SimulationApp


def _env_float(name, default):
    return float(os.environ.get(name, default))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--lat", type=float, default=_env_float("INITIAL_LAT", 37.8044))
    parser.add_argument("--lon", type=float, default=_env_float("INITIAL_LON", -122.4661))
    parser.add_argument("--origin-alt", type=float, default=_env_float("ORIGIN_ALT", 0.0))
    parser.add_argument("--flight-altitude", type=float, default=_env_float("ISAAC_FLIGHT_ALTITUDE", 60.0))
    parser.add_argument("--speed-mps", type=float, default=_env_float("ISAAC_SPEED_MPS", 3.0))
    parser.add_argument("--radius-m", type=float, default=_env_float("ISAAC_FLIGHT_RADIUS_M", 35.0))
    parser.add_argument("--duration-s", type=float, default=_env_float("ISAAC_DURATION_S", 0.0))
    parser.add_argument("--gps-topic", default=os.environ.get("GPS_TOPIC", "/drone_sim_0/sensor_measurements/gps"))
    parser.add_argument("--odom-topic", default=os.environ.get("ODOM_TOPIC", "/drone_sim_0/sensor_measurements/odom"))
    parser.add_argument("--rgb-topic", default=os.environ.get("WILDNAV_IMAGE_TOPIC", "/drone_sim_0/downward_rgb/image"))
    parser.add_argument("--rgb-info-topic", default=os.environ.get("WILDNAV_CAMERA_INFO_TOPIC", "/drone_sim_0/downward_rgb/camera_info"))
    parser.add_argument("--depth-topic", default=os.environ.get("DEPTH_TOPIC", "/drone_sim_0/downward_rgbd/depth/image"))
    parser.add_argument("--depth-info-topic", default=os.environ.get("CAMERA_INFO_TOPIC", "/drone_sim_0/downward_rgbd/camera_info"))
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
from isaacsim.core.api.materials import OmniPBR
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import get_active_viewport
from pxr import Gf, UsdGeom

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

try:
    import rclpy
    from geometry_msgs.msg import Quaternion
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import NavSatFix, NavSatStatus
except ImportError as exc:
    simulation_app.close()
    raise RuntimeError(
        "Isaac Sim ROS 2 Python libraries are unavailable. Configure the Isaac "
        "ROS 2 bridge environment, including ROS_DISTRO and bridge library paths."
    ) from exc


CAMERA_STAGE_PATH = "/World/NavDrone/downward_camera"
ROS_CAMERA_GRAPH_PATH = "/World/ROS2CameraGraph"


class NavStatePublisher(Node):
    def __init__(self, gps_topic, odom_topic):
        super().__init__("isaac_aerostack2_navsim_state")
        self.gps_pub = self.create_publisher(NavSatFix, gps_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)

    def publish(self, lat, lon, alt, x, y, z, yaw, vx, vy):
        stamp = self.get_clock().now().to_msg()

        gps = NavSatFix()
        gps.header.stamp = stamp
        gps.header.frame_id = "gps"
        gps.status.status = NavSatStatus.STATUS_FIX
        gps.status.service = NavSatStatus.SERVICE_GPS
        gps.latitude = lat
        gps.longitude = lon
        gps.altitude = alt
        gps.position_covariance = [0.8, 0.0, 0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 1.5]
        gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.gps_pub.publish(gps)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation = yaw_to_quaternion(yaw)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        self.odom_pub.publish(odom)


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    q = Quaternion()
    q.w = math.cos(half)
    q.z = math.sin(half)
    return q


def latlon_from_offsets(origin_lat, origin_lon, north_m, east_m):
    lat = origin_lat + north_m / 111_320.0
    lon = origin_lon + east_m / (111_320.0 * math.cos(math.radians(origin_lat)))
    return lat, lon


def add_marker(path, position, scale, color):
    cube = VisualCuboid(
        prim_path=path,
        name=path.rsplit("/", 1)[-1],
        position=np.array(position),
        scale=np.array(scale),
        color=np.array(color),
    )
    material = OmniPBR(
        prim_path=f"/Looks/{path.rsplit('/', 1)[-1]}_material",
        name=f"{path.rsplit('/', 1)[-1]}_material",
        color=np.array(color),
    )
    cube.apply_visual_material(material)
    return cube


def build_scene():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()

    omni.kit.commands.execute(
        "CreatePrim",
        prim_type="DistantLight",
        attributes={"inputs:angle": 0.8, "inputs:intensity": 3500},
    )

    add_marker("/World/marker_red", (-22.0, -18.0, 0.15), (10.0, 4.0, 0.3), (1.0, 0.05, 0.03))
    add_marker("/World/marker_blue", (24.0, -8.0, 0.15), (5.0, 11.0, 0.3), (0.02, 0.2, 1.0))
    add_marker("/World/marker_green", (-5.0, 24.0, 0.15), (16.0, 3.0, 0.3), (0.05, 0.7, 0.18))
    add_marker("/World/marker_yellow", (15.0, 19.0, 0.15), (4.0, 4.0, 0.3), (1.0, 0.85, 0.03))

    drone = UsdGeom.Xform.Define(stage, "/World/NavDrone")
    camera = UsdGeom.Camera.Define(stage, CAMERA_STAGE_PATH)
    camera.GetHorizontalApertureAttr().Set(21)
    camera.GetVerticalApertureAttr().Set(16)
    camera.GetProjectionAttr().Set("perspective")
    camera.GetFocalLengthAttr().Set(12)
    camera.GetFocusDistanceAttr().Set(400)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 500.0))
    xform = UsdGeom.Xformable(camera)
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, args.flight_altitude))
    xform.AddOrientOp().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0.0, math.radians(90.0), 0.0]))))

    world.reset()
    return world, drone, camera


def create_camera_graph():
    keys = og.Controller.Keys
    graph, _, _, _ = og.Controller.edit(
        {
            "graph_path": ROS_CAMERA_GRAPH_PATH,
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("createViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
                ("getRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
                ("setCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),
                ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("cameraHelperRgbInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("cameraHelperDepthInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
                ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                ("setCamera.outputs:execOut", "cameraHelperRgbInfo.inputs:execIn"),
                ("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                ("setCamera.outputs:execOut", "cameraHelperDepthInfo.inputs:execIn"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgbInfo.inputs:renderProductPath"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperDepthInfo.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("createViewport.inputs:viewportId", 0),
                ("cameraHelperRgb.inputs:frameId", "downward_camera"),
                ("cameraHelperRgb.inputs:topicName", args.rgb_topic),
                ("cameraHelperRgb.inputs:type", "rgb"),
                ("cameraHelperRgbInfo.inputs:frameId", "downward_camera"),
                ("cameraHelperRgbInfo.inputs:topicName", args.rgb_info_topic),
                ("cameraHelperDepth.inputs:frameId", "downward_camera"),
                ("cameraHelperDepth.inputs:topicName", args.depth_topic),
                ("cameraHelperDepth.inputs:type", "depth"),
                ("cameraHelperDepthInfo.inputs:frameId", "downward_camera"),
                ("cameraHelperDepthInfo.inputs:topicName", args.depth_info_topic),
                ("setCamera.inputs:cameraPrim", [usdrt.Sdf.Path(CAMERA_STAGE_PATH)]),
            ],
        },
    )
    og.Controller.evaluate_sync(graph)
    simulation_app.update()
    return graph


def set_drone_pose(drone, x, y, z, yaw):
    xform = UsdGeom.Xformable(drone.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), float(z)))
    quat = euler_angles_to_quat(np.array([0.0, math.radians(90.0), yaw]))
    xform.AddOrientOp().Set(Gf.Quatf(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])))


def main():
    world, drone, _ = build_scene()
    create_camera_graph()

    rclpy.init()
    bridge = NavStatePublisher(args.gps_topic, args.odom_topic)
    tick = 0
    dt = 1.0 / 30.0
    max_ticks = int(args.duration_s / dt) if args.duration_s > 0.0 else None

    try:
        while simulation_app.is_running() and rclpy.ok():
            t = tick * dt
            omega = args.speed_mps / max(args.radius_m, 1.0)
            x = args.radius_m * math.cos(omega * t)
            y = args.radius_m * math.sin(omega * t)
            vx = -args.speed_mps * math.sin(omega * t)
            vy = args.speed_mps * math.cos(omega * t)
            yaw = math.atan2(vy, vx)
            lat, lon = latlon_from_offsets(args.lat, args.lon, y, x)

            set_drone_pose(drone, x, y, args.flight_altitude, yaw)
            rclpy.spin_once(bridge, timeout_sec=0.0)
            bridge.publish(lat, lon, args.origin_alt + args.flight_altitude, x, y, args.flight_altitude, yaw, vx, vy)
            world.step(render=True)

            tick += 1
            if max_ticks is not None and tick >= max_ticks:
                break
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
