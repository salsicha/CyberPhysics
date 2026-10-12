#!/usr/bin/env python3
"""Move the tracking targets through the Gazebo world and publish their
ground truth (systems/aerodrone/tracking.md, ``/simulation/targets``).

build_georeferenced_terrain.py --targets puts each target model (cars,
people; visual only, no gravity) into the world at its start pose and
writes the tracks every simulator follows (tracking_targets.json). This
node follows Gazebo's simulation time on /clock (bridged by
as2_gazebo_assets with use_sim_time): every ``--period-s`` of it, it sets
each model to its pose at that time through the world's ``set_pose``
service (UserCommands; this node starts the ros_gz_bridge service bridge)
and publishes /simulation/targets for the same time. Gazebo applies a pose
in its next iteration (5 ms of simulation time), before the following
camera frame. Targets named on /simulation/targets/hide are put under the
terrain (tracking_targets.HIDE_TOPIC).

Genesis and Isaac move the same tracks inside their own simulation loops
(applications/genesis|isaac/scripts/aerodrone_navsim.py).
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys

# tracking_targets.py: mounted there by the composition, in the repository
# next to this application.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "demnav" / "scripts"))
sys.path.insert(0, "/workspace/applications/demnav/scripts")
from tracking_targets import HIDE_TOPIC, TRUTH_TOPIC, Tracks, truth_json  # noqa: E402


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--assets", default="/data/navsim_assets")
    parser.add_argument("--world", default="mount_tamalpais")
    parser.add_argument("--period-s", type=float, default=0.02,
                        help="simulation time between pose updates and truth messages")
    parser.add_argument("--clock-topic", default="/clock")
    parser.add_argument("--no-bridge", action="store_true",
                        help="the set_pose service bridge runs elsewhere")
    return parser.parse_args(argv)


def set_pose_service(world):
    return f"/world/{world}/set_pose"


def bridge_command(world):
    """ros_gz_bridge service bridge of the world's set_pose (UserCommands)."""
    return ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
            f"{set_pose_service(world)}@ros_gz_interfaces/srv/SetEntityPose"]


class Schedule:
    """When to update: every period of simulation time, and again from the
    start when the clock goes back (a world reset)."""

    def __init__(self, period_s):
        self.period_s = float(period_s)
        self.last = None

    def due(self, t):
        if self.last is None or t < self.last or t - self.last >= 0.999 * self.period_s:
            self.last = t
            return True
        return False


def run(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import SetEntityPose
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String

    tracks = Tracks.load(Path(args.assets) / "tracking_targets.json")

    class GazeboTrackingTargets(Node):
        def __init__(self):
            super().__init__("gazebo_tracking_targets")
            self.client = self.create_client(SetEntityPose, set_pose_service(args.world))
            self.truth = self.create_publisher(String, TRUTH_TOPIC, 10)
            self.schedule = Schedule(args.period_s)
            self.pending = {}
            self.sent = self.failed = self.skipped = self.published = 0
            self.create_subscription(Clock, args.clock_topic, self.on_clock,
                                     qos_profile_sensor_data)
            self.create_subscription(String, HIDE_TOPIC, self.on_hide, 10)
            self.create_timer(10.0, self.report)
            self.get_logger().info(
                f"Moving {', '.join(t.name for t in tracks.tracks)} in world {args.world} "
                f"every {args.period_s} s of simulation time; truth on {TRUTH_TOPIC}")

        def on_hide(self, msg):
            hidden = tracks.hide(msg.data)
            self.get_logger().info(f"hidden targets: {', '.join(sorted(hidden)) or 'none'}")

        def on_clock(self, msg):
            t = msg.clock.sec + msg.clock.nanosec * 1e-9
            if not self.schedule.due(t):
                return
            poses = tracks.poses(t)
            if self.client.service_is_ready():
                for pose in tracks.placed(poses):
                    self.set_pose(pose)
            data = String()
            data.data = truth_json(t, poses, tracks.origin, tracks.hidden)
            self.truth.publish(data)
            self.published += 1

        def set_pose(self, pose):
            future = self.pending.get(pose.name)
            if future is not None and not future.done():
                self.skipped += 1  # Gazebo has not answered the previous one
                return
            request = SetEntityPose.Request()
            request.entity.name = pose.name
            request.entity.type = Entity.MODEL
            request.pose.position.x = pose.east
            request.pose.position.y = pose.north
            request.pose.position.z = pose.up
            w, x, y, z = pose.quaternion_wxyz()
            orientation = request.pose.orientation
            orientation.w, orientation.x, orientation.y, orientation.z = w, x, y, z
            future = self.client.call_async(request)
            future.add_done_callback(self.on_done)
            self.pending[pose.name] = future
            self.sent += 1

        def on_done(self, future):
            result = future.result()
            if result is None or not result.success:
                self.failed += 1

        def report(self):
            self.get_logger().info(
                f"set_pose sent {self.sent}, failed {self.failed}, skipped {self.skipped}; "
                f"truth messages {self.published}; service "
                f"{'ready' if self.client.service_is_ready() else 'NOT ready'}")

    bridge = None
    if not args.no_bridge:
        # Its own session: the stop signal reaches this node, which then
        # stops the bridge.
        bridge = subprocess.Popen(bridge_command(args.world), start_new_session=True)
    # SIGINT (compose's stop_signal) and SIGTERM end the loop here rather
    # than shutting rclpy down under a running spin.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GazeboTrackingTargets()
    stop = []
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda *_: stop.append(True))
    try:
        while not stop:
            rclpy.spin_once(node, timeout_sec=0.5)
            if bridge is not None and bridge.poll() is not None:
                node.get_logger().error(f"set_pose bridge exited ({bridge.returncode})")
                return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if bridge is not None and bridge.poll() is None:
            os.killpg(bridge.pid, signal.SIGINT)
            try:
                bridge.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(bridge.pid, signal.SIGKILL)
    return 0


if __name__ == "__main__":
    sys.exit(run(parse_args()))
