#!/usr/bin/env python3
"""Send tracked RACECAR Neo waypoint missions to Nav2."""

import argparse
import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class MissionRunner(Node):
    def __init__(self, mission_file, mission_id, action_name):
        super().__init__('racecarneo_nav2_mission_runner')
        self.mission_file = Path(mission_file)
        self.mission_id = mission_id
        self.action_name = action_name
        self.client = ActionClient(self, NavigateThroughPoses, action_name)

    def load_mission(self):
        data = json.loads(self.mission_file.read_text())
        missions = {mission['id']: mission for mission in data.get('missions', [])}
        if self.mission_id not in missions:
            raise SystemExit(f'Mission {self.mission_id!r} not found in {self.mission_file}')
        mission = missions[self.mission_id]
        frame_id = mission.get('frame_id', data.get('default_frame_id', 'map'))
        laps = int(mission.get('laps', 1))
        poses = []
        for _ in range(max(1, laps)):
            for waypoint in mission.get('waypoints', []):
                poses.append(self.pose_stamped(frame_id, waypoint['pose']))
        return data, mission, poses

    def pose_stamped(self, frame_id, pose):
        x, y, yaw = pose
        qx, qy, qz, qw = quaternion_from_yaw(float(yaw))
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def run(self):
        _, mission, poses = self.load_mission()
        if not poses:
            raise SystemExit(f'Mission {mission["id"]!r} has no waypoints')
        self.get_logger().info(f'Waiting for Nav2 action {self.action_name}')
        self.client.wait_for_server()
        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        self.get_logger().info(f'Sending mission {mission["id"]} with {len(poses)} poses')
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            raise SystemExit(f'Nav2 rejected mission {mission["id"]}')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        status = getattr(result, 'status', None)
        self.get_logger().info(f'Mission {mission["id"]} finished with action status {status}')
        return 0 if status == 4 else 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mission-file', default='/workspace/systems/racecarneo/missions/nav2_waypoints.json')
    parser.add_argument('--mission-id', default='simple_loop')
    parser.add_argument('--action-name', default='navigate_through_poses')
    args = parser.parse_args()

    rclpy.init()
    node = MissionRunner(args.mission_file, args.mission_id, args.action_name)
    try:
        raise SystemExit(node.run())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
