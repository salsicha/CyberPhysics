#!/usr/bin/env python3
"""Tracking targets of the Aerodrone simulations: routes on the terrain and
their ground truth.

``applications/aerostack2/missions/tracking_targets.json`` defines each
target once: a closed route of ENU points (the frame of
``wilderness_missions.json``: metres about INITIAL_LAT/INITIAL_LON and
ORIGIN_ALT), a corner radius, a speed and a model. The terrain builder
(build_georeferenced_terrain.py) resolves every route on the georeferenced
terrain mesh, the triangles every simulator renders (``terrain.obj`` /
``terrain_mesh.npz``), into a track sampled every ``sample_spacing_m``:
position, ground height, heading and, for vehicles, the pitch and roll of
the ground under their wheels. It writes the tracks to
``/data/navsim_assets/tracking_targets.json``.

Gazebo and Isaac load that file through ``Tracks`` and place each
target at ``Tracks.poses(t)`` for their own simulation time ``t``, so the
three move the targets identically, and publish ``truth_message(t, ...)``
on ``/simulation/targets`` (systems/aerodrone/tracking.md): ENU metres in
the navigation frame, i.e. about the ArduPilot EKF origin, which is the
vehicle's home (``manifest.json`` ``home_enu_m``; the simulators feed
ArduPilot positions relative to it, ardupilot_json.state_from_enu).
``up`` is the ground under the target, where its model touches the
terrain.

Each simulator also hides the targets named on ``/simulation/targets/hide``
(``HIDE_TOPIC``: comma-separated names, empty shows them all again): it
renders a hidden target ``HIDDEN_DEPTH_M`` under the terrain, where the
camera cannot see it, and keeps publishing its truth on its route, marked
``"hidden": true`` (the occlusion of tools/tracking_sim_test.py).

Plain numpy, no ROS: the builder, the two simulator adapters and the
tests import it.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

SCHEMA_VERSION = 1
TRUTH_TOPIC = "/simulation/targets"
HIDE_TOPIC = "/simulation/targets/hide"
# How far under its ground a hidden target is rendered: beneath the terrain
# surface, out of the downward camera's sight.
HIDDEN_DEPTH_M = 50.0


# --- terrain ----------------------------------------------------------------

class MeshHeight:
    """Height of the terrain render mesh: the grid of terrain_mesh.npz
    (rows north to south, columns west to east), each cell split into the
    triangles (a, c, b) and (b, c, d) that make_mesh writes."""

    def __init__(self, vertices, rows=None, cols=None):
        vertices = np.asarray(vertices, dtype=np.float64)
        if rows is None or cols is None:
            cols = int(np.argmax(np.diff(vertices[:, 1]) != 0)) + 1
            rows = len(vertices) // cols
        grid = vertices.reshape(rows, cols, 3)
        self.east = grid[0, :, 0]
        self.north = grid[:, 0, 1]
        self.heights = grid[:, :, 2]
        if not (np.all(np.diff(self.east) > 0) and np.all(np.diff(self.north) < 0)):
            raise ValueError("terrain mesh vertices are not a west-east, north-south grid")

    @classmethod
    def load(cls, path, rows=None, cols=None):
        return cls(np.load(path)["vertices"], rows, cols)

    def __call__(self, east, north):
        east = np.clip(np.asarray(east, dtype=np.float64), self.east[0], self.east[-1])
        north = np.clip(np.asarray(north, dtype=np.float64), self.north[-1], self.north[0])
        col = np.clip(np.searchsorted(self.east, east, side="right") - 1, 0, len(self.east) - 2)
        # north decreases with the row: search the reversed axis.
        reverse = self.north[::-1]
        row = len(self.north) - 1 - np.searchsorted(reverse, north, side="left")
        row = np.clip(row, 0, len(self.north) - 2)
        u = (east - self.east[col]) / (self.east[col + 1] - self.east[col])
        v = (self.north[row] - north) / (self.north[row] - self.north[row + 1])
        za = self.heights[row, col]
        zb = self.heights[row, col + 1]
        zc = self.heights[row + 1, col]
        zd = self.heights[row + 1, col + 1]
        lower = za + u * (zb - za) + v * (zc - za)
        upper = zd + (1.0 - u) * (zc - zd) + (1.0 - v) * (zb - zd)
        result = np.where(u + v <= 1.0, lower, upper)
        return float(result) if result.ndim == 0 else result


# --- routes -----------------------------------------------------------------

def _wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def route_polyline(points, corner_radius, arc_step_deg=2.0):
    """Closed route through `points` (E, N) with every corner rounded to a
    circular arc of `corner_radius` (smaller where the legs are short), as a
    dense polyline (arcs in `arc_step_deg` steps) that starts where the
    route leaves the first point's corner, heading for the second point."""
    points = np.asarray(points, dtype=np.float64)
    count = len(points)
    if count < 3:
        raise ValueError("a looping route needs at least three points")
    corners = []
    for i in range(count):
        previous, corner, following = points[i - 1], points[i], points[(i + 1) % count]
        d0 = corner - previous
        d1 = following - corner
        l0, l1 = np.linalg.norm(d0), np.linalg.norm(d1)
        if l0 == 0 or l1 == 0:
            raise ValueError("a route repeats a point")
        d0, d1 = d0 / l0, d1 / l1
        turn = math.atan2(d0[0] * d1[1] - d0[1] * d1[0], float(d0 @ d1))
        half = abs(turn) / 2.0
        if half < 1e-9 or corner_radius <= 0:
            corners.append((corner, corner, None, 0.0, 0.0))
            continue
        radius = min(float(corner_radius), 0.5 * min(l0, l1) / math.tan(half))
        tangent = radius * math.tan(half)
        start, end = corner - d0 * tangent, corner + d1 * tangent
        left = np.array([-d0[1], d0[0]])
        centre = start + left * radius * math.copysign(1.0, turn)
        corners.append((start, end, centre, radius, turn))
    positions = []
    for i in range(count):
        end = corners[i][1]
        start_next, _, centre, radius, turn = corners[(i + 1) % count]
        positions.append(end)
        if centre is None:
            continue
        positions.append(start_next)
        steps = max(2, int(math.ceil(abs(math.degrees(turn)) / arc_step_deg)))
        a0 = math.atan2(*(start_next - centre)[::-1])
        for k in range(1, steps):
            angle = a0 + turn * k / steps
            positions.append(centre + radius * np.array([math.cos(angle), math.sin(angle)]))
    return np.asarray(positions)


def resample(positions, spacing):
    """Samples about every `spacing` metres along a closed polyline (the
    closing leg back to the first point included): rows (s, east, north,
    heading), heading in radians counter-clockwise from east, and the loop
    length."""
    closed = np.vstack([positions, positions[:1]])
    legs = np.diff(closed, axis=0)
    steps = np.linalg.norm(legs, axis=1)
    keep = steps > 1e-9
    closed = np.vstack([closed[:-1][keep], closed[:1]])
    legs, steps = legs[keep], steps[keep]
    cumulative = np.concatenate([[0.0], np.cumsum(steps)])
    length = float(cumulative[-1])
    count = max(3, int(round(length / spacing)))
    s = np.linspace(0.0, length, count, endpoint=False)
    east = np.interp(s, cumulative, closed[:, 0])
    north = np.interp(s, cumulative, closed[:, 1])
    leg = np.clip(np.searchsorted(cumulative, s, side="right") - 1, 0, len(steps) - 1)
    heading = np.arctan2(legs[leg, 1], legs[leg, 0])
    return np.column_stack([s, east, north, heading]), length


def ground_attitude(height, east, north, heading, wheelbase, track):
    """Pitch and roll (radians, ROS convention: positive pitch is nose
    down) of a vehicle whose wheels stand on the terrain."""
    forward = np.column_stack([np.cos(heading), np.sin(heading)])
    left = np.column_stack([-np.sin(heading), np.cos(heading)])
    front = height(east + forward[:, 0] * wheelbase / 2, north + forward[:, 1] * wheelbase / 2)
    rear = height(east - forward[:, 0] * wheelbase / 2, north - forward[:, 1] * wheelbase / 2)
    port = height(east + left[:, 0] * track / 2, north + left[:, 1] * track / 2)
    starboard = height(east - left[:, 0] * track / 2, north - left[:, 1] * track / 2)
    pitch = -np.arctan2(front - rear, wheelbase)
    roll = np.arctan2(port - starboard, track)
    return pitch, roll


def resolve_tracks(config, height, origin_enu, models_dir="models"):
    """The resolved-track document of a tracking_targets.json `config` on
    the terrain `height` (a MeshHeight): the content of
    /data/navsim_assets/tracking_targets.json."""
    spacing = float(config.get("sample_spacing_m", 0.2))
    targets = []
    names = set()
    for target in config["targets"]:
        name = target["name"]
        if name in names:
            raise ValueError(f"duplicate target name {name}")
        names.add(name)
        model = config["models"][target["model"]]
        speed = float(target.get("speed_mps", 0.0))
        route = np.asarray(target["route_enu_m"], dtype=np.float64)
        if speed > 0:
            positions = route_polyline(route, float(target.get("corner_radius_m", 0.0)))
            rows, length = resample(positions, spacing)
        else:
            if len(route) != 1:
                raise ValueError(f"{name}: a target with no speed stands at one point")
            heading = math.radians(float(target.get("heading_deg", 0.0)))
            rows, length = np.array([[0.0, route[0, 0], route[0, 1], heading]]), 0.0
        up = height(rows[:, 1], rows[:, 2])
        up = np.atleast_1d(up)
        if model.get("tilt_with_ground", False):
            pitch, roll = ground_attitude(height, rows[:, 1], rows[:, 2], rows[:, 3],
                                          float(model["wheelbase_m"]), float(model["track_m"]))
        else:
            pitch = roll = np.zeros(len(rows))
        samples = np.column_stack([rows[:, :3], up, rows[:, 3], pitch, roll])
        targets.append({
            "name": name, "class": target["class"], "model": target["model"],
            "model_dir": f"{models_dir}/{target['model']}",
            # target_models.bake's files in model_dir.
            "mesh": "model.obj", "mesh_npz": "model_mesh.npz",
            "texture": f"{target['model']}_atlas.png",
            "speed_mps": speed, "length_m": round(length, 4),
            "period_s": round(length / speed, 4) if speed > 0 else 0.0,
            "start_offset_m": float(target.get("start_offset_m", 0.0)),
            "columns": ["s", "east", "north", "up", "yaw", "pitch", "roll"],
            "samples": [[round(float(v), 4) for v in row] for row in samples],
        })
    return {
        "schema_version": SCHEMA_VERSION,
        "frame": "terrain ENU about INITIAL_LAT/INITIAL_LON/ORIGIN_ALT (metres)",
        "navigation_origin_enu_m": [round(float(v), 4) for v in origin_enu],
        "sample_spacing_m": spacing,
        "truth_topic": TRUTH_TOPIC,
        "targets": targets,
    }


# --- runtime ----------------------------------------------------------------

class Pose:
    __slots__ = ("name", "cls", "model", "east", "north", "up", "yaw", "pitch", "roll")

    def __init__(self, name, cls, model, east, north, up, yaw, pitch, roll):
        self.name, self.cls, self.model = name, cls, model
        self.east, self.north, self.up = east, north, up
        self.yaw, self.pitch, self.roll = yaw, pitch, roll

    def quaternion_wxyz(self):
        """Orientation (w, x, y, z) from roll, pitch and yaw (ZYX)."""
        cr, sr = math.cos(self.roll / 2), math.sin(self.roll / 2)
        cp, sp = math.cos(self.pitch / 2), math.sin(self.pitch / 2)
        cy, sy = math.cos(self.yaw / 2), math.sin(self.yaw / 2)
        return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


class Track:
    def __init__(self, entry):
        self.name = entry["name"]
        self.cls = entry["class"]
        self.model = entry["model"]
        self.model_dir = entry["model_dir"]
        self.files = {kind: entry[kind] for kind in ("mesh", "mesh_npz", "texture")}
        self.speed = float(entry["speed_mps"])
        self.length = float(entry["length_m"])
        self.offset = float(entry.get("start_offset_m", 0.0))
        samples = np.asarray(entry["samples"], dtype=np.float64)
        self.s, self.east, self.north, self.up = samples[:, :4].T
        self.yaw, self.pitch, self.roll = samples[:, 4:7].T

    def pose(self, t):
        """Pose at simulation time t (s): the loop's start at t = 0, then
        along it at the track's speed, round and round."""
        if self.speed <= 0 or self.length <= 0:
            return Pose(self.name, self.cls, self.model, self.east[0], self.north[0],
                        self.up[0], self.yaw[0], self.pitch[0], self.roll[0])
        distance = (self.offset + self.speed * float(t)) % self.length
        i = int(np.searchsorted(self.s, distance, side="right") - 1)
        j = (i + 1) % len(self.s)
        step = (self.s[j] if j else self.length) - self.s[i]
        f = (distance - self.s[i]) / step if step > 0 else 0.0

        def mix(values):
            return float(values[i] + f * (values[j] - values[i]))

        def mix_angle(values):
            return float(_wrap(values[i] + f * _wrap(values[j] - values[i])))

        return Pose(self.name, self.cls, self.model, mix(self.east), mix(self.north),
                    mix(self.up), mix_angle(self.yaw), mix_angle(self.pitch),
                    mix_angle(self.roll))


class Tracks:
    """The resolved tracks of /data/navsim_assets/tracking_targets.json."""

    def __init__(self, document, base_dir="."):
        if int(document.get("schema_version", 0)) != SCHEMA_VERSION:
            raise ValueError("unsupported tracking_targets.json schema")
        self.base_dir = Path(base_dir)
        self.origin = np.asarray(document["navigation_origin_enu_m"], dtype=np.float64)
        self.tracks = [Track(entry) for entry in document["targets"]]
        self.hidden = frozenset()

    @classmethod
    def load(cls, path):
        path = Path(path)
        return cls(json.loads(path.read_text(encoding="utf-8")), path.parent)

    def poses(self, t):
        return [track.pose(t) for track in self.tracks]

    def hide(self, text):
        """Hide the targets a HIDE_TOPIC message names (and show the rest)."""
        self.hidden = hidden_names(text)
        return self.hidden

    def placed(self, poses):
        """The poses to render: the hidden targets under the terrain."""
        return [hidden_pose(pose) if pose.name in self.hidden else pose for pose in poses]

    def model_dir(self, track):
        return self.base_dir / track.model_dir

    def model_file(self, track, kind):
        """Path of a track's model file: 'mesh' (OBJ), 'mesh_npz' or 'texture'."""
        return self.base_dir / track.model_dir / track.files[kind]

    def truth_message(self, t, poses=None):
        return truth_message(t, self.poses(t) if poses is None else poses, self.origin,
                             self.hidden)


def hidden_names(text):
    """The target names of a HIDE_TOPIC message."""
    return frozenset(name.strip() for name in str(text).split(",") if name.strip())


def hidden_pose(pose):
    return Pose(pose.name, pose.cls, pose.model, pose.east, pose.north,
                pose.up - HIDDEN_DEPTH_M, pose.yaw, pose.pitch, pose.roll)


def truth_message(t, poses, origin, hidden=()):
    """/simulation/targets content (systems/aerodrone/tracking.md): stamp
    in simulation seconds, positions ENU about the navigation origin; a
    hidden target keeps its route position, marked hidden."""
    targets = []
    for pose in poses:
        item = {
            "name": pose.name, "class": pose.cls,
            "east": round(pose.east - float(origin[0]), 3),
            "north": round(pose.north - float(origin[1]), 3),
            "up": round(pose.up - float(origin[2]), 3),
            "yaw": round(pose.yaw, 4),
        }
        if pose.name in hidden:
            item["hidden"] = True
        targets.append(item)
    return {"stamp": round(float(t), 6), "targets": targets}


def truth_json(t, poses, origin, hidden=()):
    return json.dumps(truth_message(t, poses, origin, hidden), separators=(",", ":"))
