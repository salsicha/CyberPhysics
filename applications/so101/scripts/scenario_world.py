#!/usr/bin/env python3
"""Generate the Gazebo world of an SO-101 scenario.

The scenario JSON (systems/so101/scenarios/picking_table.json) is the scene's
single source of truth; Isaac builds its scene from it at start-up, and
Gazebo loads the SDF this script writes from it:

  scenario_world.py                 rewrite systems/so101/worlds/picking_table.sdf
  scenario_world.py --check         exit 1 if the committed world is stale
  scenario_world.py --scenario S --output W

Static assets become static models, objects become free bodies with the
scenario's mass, inertia and friction (they start resting on their surfaces,
see so101_common.scene_support_errors), and the scenario camera becomes an
RGB-D sensor at its pose (Gazebo cameras look along +X with +Z up, the frame
of groot_camera_link), publishing on gz topics under /so101/camera that
gazebo.launch.py bridges to the common ROS topics. The robot is not in the
world: gazebo.launch.py spawns it at the scenario's robot_spawn.
"""

import argparse
import math
from pathlib import Path
import sys

from so101_common import (
    CAMERA_FRAME_ID,
    load_scenario_file,
    scenario_camera,
    scene_color,
    scene_support_errors,
)

GZ_CAMERA_TOPIC = "/so101/camera"   # <topic>/image, /depth_image, /camera_info, /points
DEFAULT_SCENARIO = Path("systems", "so101", "scenarios", "picking_table.json")
DEFAULT_WORLD = Path("systems", "so101", "worlds", "picking_table.sdf")


def _num(value):
    """Shortest exact-enough text for an SDF number (the scenario's values
    round-trip; colours computed from them drop float noise)."""
    text = f"{float(value):.9g}"
    return "0" if text in ("-0", "0") else text


def _vec(values):
    return " ".join(_num(value) for value in values)


def _pose(entry):
    return _vec(list(entry["xyz"][:3]) + list(entry.get("rpy", [0.0, 0.0, 0.0])[:3]))


def _material(entry):
    r, g, b = scene_color(entry)
    return (f"<material><ambient>{_vec((0.6 * r, 0.6 * g, 0.6 * b, 1))}</ambient>"
            f"<diffuse>{_vec((r, g, b, 1))}</diffuse>"
            f"<specular>{_vec((0.06, 0.06, 0.06, 1))}</specular></material>")


def _geometry(entry):
    if entry.get("shape") == "cylinder":
        return (f"<cylinder><radius>{_num(entry['radius_m'])}</radius>"
                f"<length>{_num(entry['height_m'])}</length></cylinder>")
    return f"<box><size>{_vec(entry['size'][:3])}</size></box>"


def _inertia(entry):
    mass = float(entry["mass_kg"])
    if entry.get("shape") == "cylinder":
        r, h = float(entry["radius_m"]), float(entry["height_m"])
        ixx = iyy = mass * (3.0 * r * r + h * h) / 12.0
        izz = mass * r * r / 2.0
    else:
        x, y, z = (float(value) for value in entry["size"][:3])
        ixx, iyy, izz = (mass * (y * y + z * z) / 12.0, mass * (x * x + z * z) / 12.0,
                         mass * (x * x + y * y) / 12.0)
    return (f"<inertial><mass>{_num(mass)}</mass><inertia><ixx>{ixx:.4g}</ixx><ixy>0</ixy><ixz>0</ixz>"
            f"<iyy>{iyy:.4g}</iyy><iyz>0</iyz><izz>{izz:.4g}</izz></inertia></inertial>")


def _surface(entry):
    if "friction" not in entry:
        return ""
    mu = _num(entry["friction"])
    return f"<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>"


def _static_model(entry):
    name = entry["name"]
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{_pose(entry)}</pose>
      <link name="link">
        <collision name="collision"><geometry>{_geometry(entry)}</geometry></collision>
        <visual name="visual"><geometry>{_geometry(entry)}</geometry>{_material(entry)}</visual>
      </link>
    </model>"""


def _object_model(entry):
    name = entry["id"]
    return f"""
    <model name="{name}">
      <pose>{_pose(entry)}</pose>
      <link name="link">
        {_inertia(entry)}
        <collision name="collision"><geometry>{_geometry(entry)}</geometry>{_surface(entry)}</collision>
        <visual name="visual"><geometry>{_geometry(entry)}</geometry>{_material(entry)}</visual>
      </link>
    </model>"""


def _camera_model(camera):
    width, height = (int(value) for value in camera["resolution"])
    near, far = (float(value) for value in camera.get("clipping_range_m", [0.01, 100.0]))
    far = min(far, 10.0)  # the depth image's range; the scene is within 2 m
    hfov = math.radians(float(camera["horizontal_fov_deg"]))
    rate = float(camera.get("frame_rate_hz", 30.0))
    return f"""
    <model name="{camera.get('name', 'groot_camera')}">
      <static>true</static>
      <pose>{_vec(list(camera['xyz'][:3]) + list(camera['rpy'][:3]))}</pose>
      <link name="link">
        <sensor name="rgbd" type="rgbd_camera">
          <gz_frame_id>{CAMERA_FRAME_ID}</gz_frame_id>
          <topic>{GZ_CAMERA_TOPIC}</topic>
          <always_on>true</always_on>
          <update_rate>{_num(rate)}</update_rate>
          <camera>
            <horizontal_fov>{hfov:.6f}</horizontal_fov>
            <image><width>{width}</width><height>{height}</height><format>R8G8B8</format></image>
            <clip><near>{_num(near)}</near><far>{_num(far)}</far></clip>
            <depth_camera><clip><near>{_num(near)}</near><far>{_num(far)}</far></clip></depth_camera>
          </camera>
        </sensor>
      </link>
    </model>"""


def world_sdf(scenario, source="systems/so101/scenarios/picking_table.json"):
    """The SDF world of ``scenario`` (raises ValueError if it is inconsistent)."""
    errors = scene_support_errors(scenario)
    if errors:
        raise ValueError("inconsistent scenario:\n  " + "\n  ".join(errors))
    statics = "".join(_static_model(entry) for entry in scenario.get("static_assets", []))
    objects = "".join(_object_model(entry) for entry in scenario.get("objects", []))
    camera = _camera_model(scenario_camera(scenario)) if scenario.get("camera") else ""
    name = scenario.get("name", "so101_scene")
    return f"""<?xml version="1.0" ?>
<!-- Generated from {source} by
     applications/so101/scripts/scenario_world.py: edit the scenario and
     regenerate, do not edit this file. -->
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.7 0.72 0.75 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="key_light">
      <pose>-1.2 -0.8 3.2 0 0 0</pose>
      <diffuse>0.9 0.86 0.78 1</diffuse>
      <specular>0.25 0.25 0.22 1</specular>
      <direction>0.45 0.3 -1</direction>
      <cast_shadows>true</cast_shadows>
    </light>
    <light type="point" name="bench_fill_light">
      <pose>0.2 0.8 1.5 0 0 0</pose>
      <diffuse>0.35 0.38 0.42 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <attenuation>
        <range>3.0</range>
        <constant>0.6</constant>
        <linear>0.2</linear>
        <quadratic>0.02</quadratic>
      </attenuation>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>5 5</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>5 5</size></plane></geometry>
          <material><ambient>0.38 0.38 0.38 1</ambient><diffuse>0.48 0.48 0.48 1</diffuse></material>
        </visual>
      </link>
    </model>
{statics}
{objects}
{camera}
  </world>
</sdf>
"""


def main(argv=None):
    repo = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", type=Path, default=repo / DEFAULT_SCENARIO)
    parser.add_argument("--output", type=Path, default=repo / DEFAULT_WORLD)
    parser.add_argument("--check", action="store_true", help="exit 1 if --output differs from the scenario")
    args = parser.parse_args(argv)
    try:
        source = args.scenario.resolve().relative_to(repo).as_posix()
    except ValueError:
        source = args.scenario.name
    text = world_sdf(load_scenario_file(args.scenario), source)
    if args.check:
        if not args.output.is_file() or args.output.read_text() != text:
            print(f"{args.output} is stale: run {Path(__file__).name} to regenerate it", file=sys.stderr)
            return 1
        print(f"{args.output} is up to date")
        return 0
    args.output.write_text(text)
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
