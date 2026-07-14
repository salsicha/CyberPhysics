#!/usr/bin/env python3
"""Import and run SO-101 in Isaac Sim using the common ROS 2 interface."""

import argparse
import json
import os
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp

from so101_common import (
    CAMERA_FRAME_ID,
    CAMERA_HEIGHT,
    CAMERA_IMU_TOPIC,
    CAMERA_WIDTH,
    DEPTH_CAMERA_INFO_TOPIC,
    DEPTH_FRAME_ID,
    DEPTH_TOPIC,
    JOINT_NAMES,
    JOINT_QUANTIZATION,
    LOWER_LIMITS,
    RGB_CAMERA_INFO_TOPIC,
    RGB_TOPIC,
    UPPER_LIMITS,
    camera_info_values,
    quantize,
    quaternion_from_euler,
    synthetic_rgbd,
)

parser = argparse.ArgumentParser()
parser.add_argument("--urdf", default="/workspace/systems/so101/urdf/so101.urdf")
parser.add_argument("--usd", default="/tmp/so101.usd")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--scenario", default="/workspace/systems/so101/scenarios/picking_table.json")
parser.add_argument("--camera-source", choices=("rendered", "synthetic"), default="rendered")
parser.add_argument(
    "--experience",
    default="/isaac-sim/apps/isaacsim.exp.base.python.kit",
    help="Isaac Sim .kit experience to launch. The base Python experience avoids streaming/RTX UI startup for headless ROS demos.",
)
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": args.headless}, experience=args.experience)

import omni.kit.app
import omni.kit.commands
import omni.usd
from isaacsim.core.utils.extensions import enable_extension

# Load simulator extensions before importing their Python APIs.
enable_extension("isaacsim.asset.importer.urdf")
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation

try:
    from isaacsim.sensors.camera import Camera
except ImportError:
    Camera = None

try:
    from isaacsim.core.utils.rotations import euler_angles_to_quat
except ImportError:
    euler_angles_to_quat = None

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
    from std_msgs.msg import Float64MultiArray
except ImportError as exc:
    simulation_app.close()
    raise RuntimeError(
        "Isaac Sim ROS 2 Python libraries are unavailable. Start the script with "
        "ROS_DISTRO=jazzy and the Isaac Sim ROS 2 bridge environment configured."
    ) from exc


def load_scenario_camera(path):
    defaults = {
        "xyz": [0.17, -0.52, 0.92],
        "rpy": [1.10, 0.0, 0.0],
        "resolution": [CAMERA_WIDTH, CAMERA_HEIGHT],
        "horizontal_fov_deg": 69.0,
    }
    if not path:
        return defaults
    try:
        data = json.loads(Path(path).read_text())
    except (OSError, json.JSONDecodeError):
        return defaults
    camera = data.get("camera", {})
    return {**defaults, **camera}


def isaac_orientation_from_rpy(rpy):
    rpy = np.asarray(rpy, dtype=np.float64)
    if euler_angles_to_quat is not None:
        return np.asarray(euler_angles_to_quat(rpy), dtype=np.float64)
    qx, qy, qz, qw = quaternion_from_euler(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    return np.asarray([qw, qx, qy, qz], dtype=np.float64)


def create_rendered_camera(camera_config):
    if Camera is None:
        return None
    width, height = camera_config.get("resolution", [CAMERA_WIDTH, CAMERA_HEIGHT])
    camera = Camera(
        prim_path="/World/groot_camera",
        position=np.asarray(camera_config.get("xyz"), dtype=np.float64),
        orientation=isaac_orientation_from_rpy(camera_config.get("rpy")),
        frequency=30,
        resolution=(int(width), int(height)),
    )
    camera.initialize()
    if hasattr(camera, "set_horizontal_aperture"):
        # Keep the camera close to the RealSense D435i/D405 tabletop FOV.
        fov_deg = float(camera_config.get("horizontal_fov_deg", 69.0))
        aperture = 2.0 * camera.get_focal_length() * np.tan(np.radians(fov_deg) / 2.0)
        camera.set_horizontal_aperture(float(aperture))
    if hasattr(camera, "add_distance_to_image_plane_to_frame"):
        camera.add_distance_to_image_plane_to_frame()
    elif hasattr(camera, "add_distance_to_camera_to_frame"):
        camera.add_distance_to_camera_to_frame()
    return camera


class IsaacBridge(Node):
    def __init__(self, rendered_camera=None, camera_source="rendered"):
        super().__init__("so101_isaacsim_bridge")
        self.target = np.zeros(len(JOINT_NAMES), dtype=np.float32)
        self.rendered_camera = rendered_camera
        self.camera_source = camera_source
        self.warned_camera_fallback = False
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.image_pub = self.create_publisher(Image, RGB_TOPIC, 5)
        self.camera_info_pub = self.create_publisher(CameraInfo, RGB_CAMERA_INFO_TOPIC, 5)
        self.depth_pub = self.create_publisher(Image, DEPTH_TOPIC, 5)
        self.depth_info_pub = self.create_publisher(CameraInfo, DEPTH_CAMERA_INFO_TOPIC, 5)
        self.imu_pub = self.create_publisher(Imu, CAMERA_IMU_TOPIC, 20)
        self.camera_width = CAMERA_WIDTH
        self.camera_height = CAMERA_HEIGHT
        self.subscription = self.create_subscription(
            Float64MultiArray, "/so101/joint_commands", self._command, 10
        )

    def _command(self, msg):
        if len(msg.data) != len(JOINT_NAMES):
            self.get_logger().warning("Expected six SO-101 joint commands")
            return
        self.target = np.clip(
            np.asarray(msg.data), LOWER_LIMITS, UPPER_LIMITS
        ).astype(np.float32)

    def publish_state(self, positions, velocities):
        msg = JointState()
        stamp = self.get_clock().now().to_msg()
        msg.header.stamp = stamp
        msg.name = JOINT_NAMES
        measured_positions = quantize(np.asarray(positions), JOINT_QUANTIZATION)
        msg.position = measured_positions.tolist()
        msg.velocity = quantize(np.asarray(velocities), JOINT_QUANTIZATION).tolist()
        self.publisher.publish(msg)
        self.publish_camera(stamp, measured_positions)
        self.publish_imu(stamp, velocities)

    def publish_camera(self, stamp, positions):
        h = self.camera_height
        w = self.camera_width
        image, depth = self._camera_frame(positions)

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = CAMERA_FRAME_ID
        msg.height = h
        msg.width = w
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = image.tobytes()
        self.image_pub.publish(msg)
        self.camera_info_pub.publish(self.camera_info(stamp, CAMERA_FRAME_ID))

        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = DEPTH_FRAME_ID
        depth_msg.height = h
        depth_msg.width = w
        depth_msg.encoding = "32FC1"
        depth_msg.is_bigendian = 0
        depth_msg.step = w * 4
        depth_msg.data = depth.tobytes()
        self.depth_pub.publish(depth_msg)
        self.depth_info_pub.publish(self.camera_info(stamp, DEPTH_FRAME_ID))

    def _camera_frame(self, positions):
        if self.camera_source == "rendered" and self.rendered_camera is not None:
            frame = self.rendered_camera.get_current_frame()
            rgba = frame.get("rgba") if isinstance(frame, dict) else None
            depth = None
            if isinstance(frame, dict):
                depth = frame.get("distance_to_image_plane")
                if depth is None:
                    depth = frame.get("distance_to_camera")
                if depth is None:
                    depth = frame.get("depth")
            if rgba is not None:
                image = np.asarray(rgba)[:, :, :3].astype(np.uint8)
                if depth is None:
                    depth = np.full(image.shape[:2], np.nan, dtype=np.float32)
                return image, np.asarray(depth, dtype=np.float32)
        if self.camera_source == "rendered" and not self.warned_camera_fallback:
            self.get_logger().warning("Rendered Isaac camera frame unavailable; using synthetic RGB-D fallback")
            self.warned_camera_fallback = True
        return synthetic_rgbd(positions, self.camera_width, self.camera_height)

    def publish_imu(self, stamp, velocities):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = CAMERA_FRAME_ID
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.z = float(np.asarray(velocities)[0])
        msg.angular_velocity_covariance = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0008]
        msg.linear_acceleration.z = 9.80665
        msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.02]
        self.imu_pub.publish(msg)

    def camera_info(self, stamp, frame_id):
        values = camera_info_values(self.camera_width, self.camera_height)
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


def import_robot(urdf_path):
    result, config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not result:
        raise RuntimeError("Failed to create Isaac Sim URDF import configuration")
    config.fix_base = True
    config.merge_fixed_joints = False
    config.import_inertia_tensor = True
    config.make_default_prim = True
    config.self_collision = False
    config.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_POSITION
    config.default_drive_strength = 80.0
    config.default_position_drive_damping = 8.0
    config.distance_scale = 1.0

    result, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=config,
        get_articulation_root=True,
    )
    if not result:
        raise RuntimeError(f"Failed to import SO-101 URDF: {urdf_path}")
    return prim_path


def main():
    if not os.path.isfile(args.urdf):
        raise FileNotFoundError(args.urdf)

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    camera_config = load_scenario_camera(args.scenario)
    rendered_camera = None
    if args.camera_source == "rendered":
        rendered_camera = create_rendered_camera(camera_config)
    prim_path = import_robot(args.urdf)
    robot = world.scene.add(Articulation(prim_path=prim_path, name="so101"))
    world.reset()
    robot.initialize()
    if args.usd:
        omni.usd.get_context().save_as_stage(args.usd)

    index_by_name = {name: index for index, name in enumerate(robot.dof_names)}
    missing = [name for name in JOINT_NAMES if name not in index_by_name]
    if missing:
        raise RuntimeError(f"Imported articulation is missing joints: {missing}")
    indices = np.array([index_by_name[name] for name in JOINT_NAMES])

    rclpy.init()
    bridge = IsaacBridge(rendered_camera=rendered_camera, camera_source=args.camera_source)
    try:
        while simulation_app.is_running() and rclpy.ok():
            rclpy.spin_once(bridge, timeout_sec=0.0)
            robot.set_joint_position_targets(bridge.target, joint_indices=indices)
            world.step(render=(not args.headless) or args.camera_source == "rendered")
            bridge.publish_state(
                robot.get_joint_positions(joint_indices=indices),
                robot.get_joint_velocities(joint_indices=indices),
            )
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
