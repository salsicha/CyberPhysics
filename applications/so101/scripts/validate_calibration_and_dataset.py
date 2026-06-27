#!/usr/bin/env python3
"""Validate SO-101 camera calibration, frame alignment, and dataset sync manifests."""

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from so101_common import (
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
    camera_info_values,
)


REQUIRED_URDF_JOINTS = [
    "base_to_groot_camera",
    "groot_camera_optical_joint",
    "wrist_to_gripper",
    "gripper",
    "fixed_jaw_joint",
]
REQUIRED_COLLISION_LINKS = [
    "base_link",
    "groot_camera_link",
    "gripper_base_link",
    "gripper_jaw_link",
    "fixed_jaw_link",
]
REQUIRED_SYNC_TOPICS = [
    "/so101/camera/image_raw",
    "/so101/camera/camera_info",
    "/so101/camera/depth/image_rect_raw",
    "/so101/camera/depth/camera_info",
    "/joint_states",
    "/so101/joint_commands",
]


def load_json(path):
    return json.loads(Path(path).read_text())


def parse_xyz_rpy(origin):
    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]
    if origin is not None:
        xyz = [float(value) for value in origin.attrib.get("xyz", "0 0 0").split()]
        rpy = [float(value) for value in origin.attrib.get("rpy", "0 0 0").split()]
    return xyz, rpy


def urdf_model(path):
    root = ET.parse(path).getroot()
    joints = {}
    for joint in root.findall("joint"):
        joints[joint.attrib["name"]] = joint
    links = {}
    for link in root.findall("link"):
        links[link.attrib["name"]] = link
    return joints, links


def assert_near(name, actual, expected, tolerance):
    actual = np.asarray(actual, dtype=np.float64)
    expected = np.asarray(expected, dtype=np.float64)
    error = float(np.max(np.abs(actual - expected)))
    if error > tolerance:
        raise AssertionError(f"{name} differs by {error:.6g}, tolerance {tolerance:.6g}")
    return error


def validate_urdf_alignment(urdf_path):
    joints, links = urdf_model(urdf_path)
    missing_joints = [name for name in REQUIRED_URDF_JOINTS if name not in joints]
    if missing_joints:
        raise AssertionError(f"Missing URDF joints: {missing_joints}")
    missing_collision = [name for name in REQUIRED_COLLISION_LINKS if links[name].find("collision") is None]
    if missing_collision:
        raise AssertionError(f"Missing collision geometry on links: {missing_collision}")

    camera_xyz, camera_rpy = parse_xyz_rpy(joints["base_to_groot_camera"].find("origin"))
    optical_xyz, optical_rpy = parse_xyz_rpy(joints["groot_camera_optical_joint"].find("origin"))
    gripper_xyz, _ = parse_xyz_rpy(joints["wrist_to_gripper"].find("origin"))
    moving_jaw_xyz, _ = parse_xyz_rpy(joints["gripper"].find("origin"))

    metrics = {
        "base_to_camera_xyz": camera_xyz,
        "base_to_camera_rpy": camera_rpy,
        "camera_optical_xyz_error_m": assert_near("camera optical translation", optical_xyz, [0.0, 0.0, 0.0], 1e-9),
        "camera_optical_rpy_error_rad": assert_near("camera optical rpy", optical_rpy, [-math.pi / 2.0, 0.0, -math.pi / 2.0], 1e-6),
        "wrist_to_gripper_xyz": gripper_xyz,
        "moving_jaw_nominal_xyz": moving_jaw_xyz,
    }
    if not (0.16 <= camera_xyz[0] <= 0.32 and -0.38 <= camera_xyz[1] <= -0.18 and 0.25 <= camera_xyz[2] <= 0.45):
        raise AssertionError("base_to_groot_camera is outside expected tabletop camera mount bounds")
    if moving_jaw_xyz[0] <= 0.0:
        raise AssertionError("moving gripper jaw must open along positive x from gripper_base_link")
    return metrics


def validate_scenario_camera(scenario_path, camera_info_path=None):
    scenario = load_json(scenario_path)
    camera = scenario.get("camera", {})
    width, height = camera.get("resolution", [CAMERA_WIDTH, CAMERA_HEIGHT])
    expected = camera_info_values(width, height)
    metrics = {
        "scenario": scenario.get("name"),
        "camera_frame_id": camera.get("frame_id"),
        "expected_camera_info": expected,
    }
    if camera.get("frame_id") != "groot_camera_rgb_optical_frame":
        raise AssertionError("scenario camera frame_id must be groot_camera_rgb_optical_frame")
    if camera_info_path:
        observed = load_json(camera_info_path)
        metrics["camera_info_k_error"] = assert_near("camera_info.k", observed["k"], expected["k"], 1e-6)
        metrics["camera_info_p_error"] = assert_near("camera_info.p", observed["p"], expected["p"], 1e-6)
        if observed.get("frame_id") != camera.get("frame_id"):
            raise AssertionError("camera_info frame_id does not match scenario camera frame")
    return metrics


def validate_tf_manifest(tf_path):
    transforms = load_json(tf_path).get("transforms", [])
    by_child = {entry["child_frame_id"]: entry for entry in transforms}
    required = {
        "groot_camera_link": "base_link",
        "groot_camera_rgb_optical_frame": "groot_camera_link",
        "groot_camera_depth_optical_frame": "groot_camera_rgb_optical_frame",
        "table_frame": "world",
    }
    missing = [child for child in required if child not in by_child]
    if missing:
        raise AssertionError(f"Missing TF entries: {missing}")
    for child, parent in required.items():
        if by_child[child].get("parent_frame_id") != parent:
            raise AssertionError(f"TF {child} parent is {by_child[child].get('parent_frame_id')}, expected {parent}")
    return {"tf_entries": len(transforms)}


def validate_sync_manifest(sync_path):
    manifest = load_json(sync_path)
    topics = manifest.get("topics", {})
    missing = [topic for topic in REQUIRED_SYNC_TOPICS if topic not in topics]
    if missing:
        raise AssertionError(f"Missing required synchronized topics: {missing}")
    max_skew_s = float(manifest.get("max_skew_s", 0.05))
    for topic, stamps in topics.items():
        if len(stamps) < 2:
            raise AssertionError(f"Topic {topic} has fewer than two samples")
    reference = np.asarray(topics["/joint_states"], dtype=np.float64)
    skew_by_topic = {}
    for topic in REQUIRED_SYNC_TOPICS:
        stamps = np.asarray(topics[topic], dtype=np.float64)
        skews = [float(np.min(np.abs(reference - stamp))) for stamp in stamps]
        skew_by_topic[topic] = max(skews)
        if skew_by_topic[topic] > max_skew_s:
            raise AssertionError(f"Topic {topic} max sync skew {skew_by_topic[topic]:.4f}s exceeds {max_skew_s:.4f}s")
    return {"max_skew_s": max(skew_by_topic.values()), "skew_by_topic": skew_by_topic}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--camera-info-json")
    parser.add_argument("--tf-json")
    parser.add_argument("--sync-json")
    parser.add_argument("--output")
    args = parser.parse_args()

    results = {
        "urdf_alignment": validate_urdf_alignment(args.urdf),
        "scenario_camera": validate_scenario_camera(args.scenario, args.camera_info_json),
    }
    if args.tf_json:
        results["tf_manifest"] = validate_tf_manifest(args.tf_json)
    if args.sync_json:
        results["sync_manifest"] = validate_sync_manifest(args.sync_json)

    rendered = json.dumps(results, indent=2) + "\n"
    if args.output:
        Path(args.output).write_text(rendered)
    print(rendered, end="")


if __name__ == "__main__":
    main()
