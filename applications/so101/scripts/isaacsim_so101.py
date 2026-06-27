#!/usr/bin/env python3
"""Import and run SO-101 in Isaac Sim using the common ROS 2 interface."""

import argparse
import os

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
    synthetic_rgbd,
)

parser = argparse.ArgumentParser()
parser.add_argument("--urdf", default="/workspace/systems/so101/urdf/so101.urdf")
parser.add_argument("--usd", default="/tmp/so101.usd")
parser.add_argument("--headless", action="store_true")
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


class IsaacBridge(Node):
    def __init__(self):
        super().__init__("so101_isaacsim_bridge")
        self.target = np.zeros(len(JOINT_NAMES), dtype=np.float32)
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
        image, depth = synthetic_rgbd(positions, w, h)

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
    bridge = IsaacBridge()
    try:
        while simulation_app.is_running() and rclpy.ok():
            rclpy.spin_once(bridge, timeout_sec=0.0)
            robot.set_joint_position_targets(bridge.target, joint_indices=indices)
            world.step(render=not args.headless)
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
