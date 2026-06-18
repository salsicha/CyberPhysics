#!/usr/bin/env python3
"""Import and run SO-101 in Isaac Sim using the common ROS 2 interface."""

import argparse
import os

import numpy as np
from isaacsim import SimulationApp

from so101_common import JOINT_NAMES, LOWER_LIMITS, UPPER_LIMITS

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
    from sensor_msgs.msg import CameraInfo, Image, JointState
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
        self.image_pub = self.create_publisher(Image, "/so101/camera/image_raw", 5)
        self.camera_info_pub = self.create_publisher(CameraInfo, "/so101/camera/camera_info", 5)
        self.camera_width = 640
        self.camera_height = 480
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
        msg.position = np.asarray(positions).tolist()
        msg.velocity = np.asarray(velocities).tolist()
        self.publisher.publish(msg)
        self.publish_camera(stamp, positions)

    def publish_camera(self, stamp, positions):
        positions = np.asarray(positions, dtype=np.float32)
        h = self.camera_height
        w = self.camera_width
        y = np.linspace(0, 255, h, dtype=np.uint8)[:, None]
        x = np.linspace(0, 255, w, dtype=np.uint8)[None, :]
        image = np.zeros((h, w, 3), dtype=np.uint8)
        image[:, :, 0] = (x + int((positions[0] + 2.0) * 35.0)) % 255
        image[:, :, 1] = (y + int((positions[1] + 2.0) * 35.0)) % 255
        image[:, :, 2] = 80

        # Draw a small target marker so policy input visibly changes with the arm state.
        cx = int(w * (0.5 + 0.22 * np.sin(float(positions[2]))))
        cy = int(h * (0.5 - 0.22 * np.sin(float(positions[3]))))
        image[max(0, cy - 12):min(h, cy + 12), max(0, cx - 12):min(w, cx + 12), :] = [255, 220, 30]

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "groot_camera_rgb_optical_frame"
        msg.height = h
        msg.width = w
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = image.tobytes()
        self.image_pub.publish(msg)

        info = CameraInfo()
        info.header = msg.header
        info.width = w
        info.height = h
        fx = fy = 554.0
        cx0 = w / 2.0
        cy0 = h / 2.0
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx0, 0.0, fy, cy0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx0, 0.0, 0.0, fy, cy0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(info)


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
