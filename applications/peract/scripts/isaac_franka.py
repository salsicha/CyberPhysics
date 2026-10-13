#!/usr/bin/env python3
"""Franka + rendered RGB-D + PerAct sparse-pose reaching in Isaac Sim.

Only collection labels and evaluation scoring inspect block poses. Runtime
policy requests contain rendered RGB, depth-derived points, and proprioception.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import io
import json
import os
from pathlib import Path
import signal
import time
import urllib.request

import numpy as np
from contracts import checked_action, config_digest, sample_scene
from isaacsim import SimulationApp


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", default="/workspace/systems/franka/config/peract_reach.json")
    p.add_argument("--controller-config", default="/workspace/systems/franka/config/joint_control.json")
    p.add_argument("--mode", choices=["collect", "evaluate"], default="evaluate")
    p.add_argument("--output", default="/workspace/results")
    p.add_argument("--samples", type=int, default=256)
    p.add_argument("--episodes", type=int, default=0)
    p.add_argument("--seed", type=int, default=9876)
    p.add_argument("--policy-url", default="http://127.0.0.1:5562")
    p.add_argument("--headless", action=argparse.BooleanOptionalAction, default=os.environ.get("PERACT_HEADLESS", "true").lower() == "true")
    args = p.parse_args()
    if args.samples < 1 or args.episodes < 0:
        p.error("samples must be positive and episodes must be nonnegative")
    config = json.loads(Path(args.config).read_text())
    control = json.loads(Path(args.controller_config).read_text())
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    app = SimulationApp({"headless": args.headless, "width": 960, "height": 720,
                         "extra_args": ["--enable", "isaacsim.robot.manipulators.examples", "--enable", "isaacsim.sensors.camera"]},
                        experience="/isaac-sim/apps/isaacsim.exp.base.python.kit")
    # Isaac imports must occur after SimulationApp has initialized Kit.
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid
    from isaacsim.core.utils.rotations import euler_angles_to_quat
    from isaacsim.core.utils.viewports import set_camera_view
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.robot.manipulators.examples.franka import Franka, KinematicsSolver
    from isaacsim.sensors.camera import Camera
    from pxr import UsdLux
    import rclpy
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String

    print("Initializing ROS telemetry", flush=True)
    rclpy.init()
    node = rclpy.create_node("franka_peract_isaac")
    joint_pub = node.create_publisher(JointState, "/franka/joint_states", 5)
    target_pub = node.create_publisher(PoseStamped, "/franka/peract/target", 5)
    status_pub = node.create_publisher(String, "/franka/peract/status", 5)
    stop = False

    def stopping(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGTERM, stopping)
    signal.signal(signal.SIGINT, stopping)
    print("Loading Franka scene and rendering assets", flush=True)
    world = World(stage_units_in_meters=1.0, physics_dt=1/config["physics_hz"], rendering_dt=1/config["physics_hz"])
    world.scene.add_default_ground_plane()
    dome = UsdLux.DomeLight.Define(world.stage, "/World/Light")
    dome.CreateIntensityAttr(1000)
    robot = world.scene.add(Franka(prim_path="/World/Franka", name="franka"))
    blocks = [world.scene.add(DynamicCuboid(
        prim_path=f"/World/{name}", name=name, position=np.asarray([0.45, y, config["block_size"]/2]),
        size=config["block_size"], mass=0.05, color=np.asarray(color),
    )) for name, y, color in [("red", -0.15, [0.85, 0.03, 0.03]), ("blue", 0.15, [0.03, 0.06, 0.85])]]
    world.reset()
    robot.get_articulation_controller().switch_control_mode("position")
    robot.get_articulation_controller().set_gains(kps=np.asarray(control["stiffness"]),
                                                 kds=np.asarray(control["damping"]))
    home = np.asarray([0., -0.6, 0., -2.2, 0., 1.6, 0.8, 0.04, 0.04])
    robot.set_joints_default_state(positions=home)
    robot.set_joint_positions(home)
    camera_pos = np.asarray([1.05, -0.65, 0.85])
    d = np.asarray([0.48, 0., 0.05]) - camera_pos
    orient = euler_angles_to_quat(np.array([0, -np.arctan2(d[2], np.linalg.norm(d[:2])), np.arctan2(d[1], d[0])]))
    w, h = config["camera_resolution"]
    camera = Camera("/World/Camera", position=camera_pos, orientation=orient, resolution=(w, h), frequency=30)
    camera.initialize()
    camera.set_focal_length(18)
    camera.set_horizontal_aperture(20.955)
    camera.set_clipping_range(0.05, 5.)
    camera.add_distance_to_image_plane_to_frame()
    set_camera_view(eye=np.array([1.5, 1.2, 1.1]), target=np.array([0.4, 0., 0.2]))
    overview = None
    if args.mode == "evaluate":
        eye = np.array([1.3, 1.1, 0.95])
        direction = np.array([0.35, 0., 0.3]) - eye
        view_quat = euler_angles_to_quat(np.array([0., -np.arctan2(direction[2], np.linalg.norm(direction[:2])),
                                                  np.arctan2(direction[1], direction[0])]))
        overview = Camera("/World/Overview", position=eye, orientation=view_quat, resolution=(640, 480), frequency=10)
        overview.initialize()
        overview.set_focal_length(18)
        overview.set_horizontal_aperture(20.955)
    kinematics = KinematicsSolver(robot)
    rng = np.random.default_rng(args.seed)
    rows, collected = [], []
    executor = ThreadPoolExecutor(max_workers=1)
    step_count = 0

    def step():
        nonlocal step_count
        if stop or not app.is_running():
            raise InterruptedError("Simulator stopped")
        world.step(render=True)
        step_count += 1
        if step_count % 6 == 0:
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = robot.dof_names
            msg.position = robot.get_joint_positions().tolist()
            joint_pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0)

    def observation():
        for _ in range(120):
            step()
            frame = camera.get_current_frame()
            rgba = frame.get("rgb", frame.get("rgba"))
            depth = frame.get("distance_to_image_plane")
            if rgba is not None and depth is not None and np.asarray(rgba).shape[:2] == (h, w):
                rgb = np.asarray(rgba)[..., :3].astype(np.uint8)
                depth = np.asarray(depth).reshape(h, w)
                if np.isfinite(depth).sum() < w*h/4:
                    continue
                yy, xx = np.indices((h, w))
                points = np.asarray(camera.get_world_points_from_image_coords(
                    np.column_stack((xx.ravel(), yy.ravel())), depth.ravel(), device="cpu")).reshape(h, w, 3).astype(np.float32)
                prop = np.array([1., *robot.get_joint_positions()[-2:], 1.], dtype=np.float32)
                return rgb, points, prop
        raise RuntimeError("Rendered RGB-D camera failed to initialize")

    def predict(rgb, points, prop, instruction):
        buf = io.BytesIO()
        np.savez_compressed(buf, rgb=rgb, points=points, proprio=prop, instruction=instruction,
                            config_digest=config_digest(config))
        request = urllib.request.Request(args.policy_url + "/predict", data=buf.getvalue(),
                                         headers={"Content-Type": "application/octet-stream"})
        with urllib.request.urlopen(request, timeout=60) as response:
            return json.load(response)

    try:
        print("Warming up rendered camera observations", flush=True)
        for _ in range(60):
            step()
        count = args.samples if args.mode == "collect" else (args.episodes or config["episodes"])
        for episode in range(count):
            robot.set_joint_positions(home)
            robot.set_joint_velocities(np.zeros(9))
            robot.get_articulation_controller().apply_action(ArticulationAction(joint_positions=home))
            placements = sample_scene(rng, config)
            for block, position in zip(blocks, placements):
                block.set_world_pose(position=position, orientation=np.array([1., 0., 0., 0.]))
                block.set_linear_velocity(np.zeros(3))
                block.set_angular_velocity(np.zeros(3))
            for _ in range(20):
                step()
            rgb, points, prop = observation()
            if episode == 0:
                from PIL import Image
                Image.fromarray(rgb).save(out / "camera.png")
            if args.mode == "collect":
                for language_id in range(2):
                    target = blocks[language_id].get_world_pose()[0].copy()
                    target[2] = config["reach_height"]
                    collected.append((rgb, points, prop, target, language_id))
                if episode % 32 == 0:
                    print(f"Collected {episode}/{count} scenes", flush=True)
                continue
            language_id = episode % 2
            instruction = config["instructions"][language_id]
            # This pose is only used by the scorer after the policy chooses.
            truth = blocks[language_id].get_world_pose()[0].copy()
            truth[2] = config["reach_height"]
            future = executor.submit(predict, rgb, points, prop, instruction)
            while not future.done():
                step()
            action = future.result()
            row = {"episode": episode, "instruction": instruction, "action": action, "success": False}
            frames = []
            try:
                target, euler = checked_action(action, config)
                quat = euler_angles_to_quat(np.radians(euler))
                joint_goal, reachable = kinematics.compute_inverse_kinematics(target, quat)
                if not reachable:
                    raise ValueError("Predicted pose is unreachable")
                target_msg = PoseStamped()
                target_msg.header.frame_id = "world"
                target_msg.header.stamp = node.get_clock().now().to_msg()
                target_msg.pose.position.x, target_msg.pose.position.y, target_msg.pose.position.z = map(float, target)
                target_msg.pose.orientation.w, target_msg.pose.orientation.x, target_msg.pose.orientation.y, target_msg.pose.orientation.z = map(float, quat)
                target_pub.publish(target_msg)
                settled = 0
                joint_indices = joint_goal.joint_indices
                start_joints = robot.get_joint_positions()[joint_indices].copy()
                goal_joints = np.asarray(joint_goal.joint_positions)
                # A minimum-jerk trajectory executes the model's sparse pose.
                # Joint displacement sets duration; no object pose enters control.
                duration = max(control["minimum_trajectory_seconds"],
                               float(np.max(np.abs(goal_joints-start_joints))) / control["joint_displacement_per_second"])
                for motion_step in range(config["max_motion_steps"]):
                    t = min((motion_step+1) / (duration*config["physics_hz"]), 1.)
                    blend = 10*t**3 - 15*t**4 + 6*t**5
                    joint_action = ArticulationAction(joint_positions=start_joints + blend*(goal_joints-start_joints),
                                                      joint_indices=joint_indices)
                    robot.get_articulation_controller().apply_action(joint_action)
                    step()
                    if episode < 2 and motion_step % 10 == 0:
                        from PIL import Image
                        view = overview.get_current_frame()
                        pixels = view.get("rgb", view.get("rgba"))
                        if pixels is not None and np.asarray(pixels).size:
                            frames.append(Image.fromarray(np.asarray(pixels)[..., :3].astype(np.uint8)))
                    actual, _ = kinematics.compute_end_effector_pose()
                    tracking_error = float(np.linalg.norm(actual - target))
                    settled = settled + 1 if t >= 1. and tracking_error < 0.015 else 0
                    if settled >= config["settle_steps"]:
                        break
                task_error = float(np.linalg.norm(actual - truth))
                row.update(success=bool(settled >= config["settle_steps"] and task_error < config["position_tolerance"]),
                           task_error_m=task_error, tracking_error_m=tracking_error,
                           prediction_error_m=float(np.linalg.norm(target-truth)), motion_steps=motion_step+1,
                           actual_position=actual.tolist(), scoring_target=truth.tolist(),
                           joint_goal=goal_joints.tolist(), joint_actual=robot.get_joint_positions()[joint_indices].tolist(),
                           trajectory_duration_s=duration)
            except ValueError as exc:
                row["error"] = str(exc)
            rows.append(row)
            from PIL import Image
            if frames:
                frames[0].save(out / f"episode_{episode:03d}.gif", save_all=True,
                               append_images=frames[1:], duration=167, loop=0)
                frames[-1].save(out / f"overview_{episode:03d}.png")
            final_frame = camera.get_current_frame()
            Image.fromarray(np.asarray(final_frame.get("rgb", final_frame.get("rgba")))[..., :3].astype(np.uint8)).save(out / f"episode_{episode:03d}.png")
            print(json.dumps(row), flush=True)
            status_pub.publish(String(data=json.dumps(row)))
            (out / "report.json").write_text(json.dumps({"task": config["task"], "seed": args.seed,
                "mode": "learned_peract", "policy_used_ground_truth": False, "episodes": rows,
                "executor": "lula_ik_minimum_jerk", "config_digest": config_digest(config),
                "joint_control": control,
                "requested_episodes": count, "completed": len(rows) == count,
                "successes": sum(r["success"] for r in rows), "total": len(rows)}, indent=2))
        if args.mode == "collect":
            np.savez_compressed(out / "dataset.npz", rgb=np.stack([r[0] for r in collected]),
                points=np.stack([r[1] for r in collected]), proprio=np.stack([r[2] for r in collected]),
                targets=np.stack([r[3] for r in collected]), language_ids=np.array([r[4] for r in collected]),
                seed=args.seed, config_digest=config_digest(config))
        elif sum(r["success"] for r in rows) / len(rows) < 0.8:
            raise RuntimeError("PerAct reaching acceptance failed: success below 80%")
    finally:
        executor.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        rclpy.shutdown()
        app.close()


if __name__ == "__main__":
    main()
