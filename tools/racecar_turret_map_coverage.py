#!/usr/bin/env python3
"""Do nvblox's map and Nav2's costmap see the arena's obstacles?
(applications/isaac/RACECAR_TURRET.md, "Obstacle maps";
tools/racecar_turret_isaac_test.sh runs both steps.)

  racecar_turret_map_coverage.py record --output-dir RUN/maps [--period-s 5]
      A ROS 2 node (the nvblox image, for nvblox_msgs; on /clock with
      --ros-args -p use_sim_time:=true): every --period-s it saves nvblox's
      2D map slice (/nvblox_node/static_map_slice, the input of Nav2's
      nvblox costmap layer) with the transform of its frame into `map` at
      its stamp, Nav2's global costmap (/global_costmap/costmap: SLAM's
      static map, the nvblox layer and the lidar's obstacle layer together)
      and the people's true positions since the last snapshot
      (/simulation/truth) as RUN/maps/snapshot_NNNN.npz.

  racecar_turret_map_coverage.py score --run-dir RUN --scenario isaac_arena.json
      Compares the snapshots with the scenario: for nvblox's slice (cells
      within half a voxel of a surface) and the costmap (lethal cells), the
      fraction of each obstacle's outline (sampled every 5 cm; the arena
      walls' inner faces) with an occupied cell within --tolerance at some
      point of the run, and at the last snapshot; for nvblox also the
      fraction of the outline it observed at all (a known cell, free or
      occupied, within --tolerance): an outline observed but never
      occupied is an obstacle the depth carved out as free space. Occupied
      cells farther than --clearance from every obstacle and from where
      the people walked are phantoms (area inside the arena, and outside).

The scenario frame is the spawn pose (the map and odom frames at the
start); SLAM's map is within 0.16 m of it in these runs, which --tolerance
(0.15 m) and --clearance (0.3 m) allow for. Scoring needs only numpy.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'applications/isaac/scripts'))
import racecar_turret_sim as rts  # noqa: E402

SLICE_TOPIC = '/nvblox_node/static_map_slice'
COSTMAP_TOPIC = '/global_costmap/costmap'
LETHAL = 100  # nav2 OccupancyGrid value of a lethal cell (254 in the costmap)


# --- geometry (tests/test_racecar_turret_map_coverage.py) --------------------

def outline_points(obstacle, spacing=0.05):
    """Points every `spacing` m along an obstacle's footprint outline."""
    if obstacle.kind == 'cylinder':
        count = max(8, int(math.ceil(2.0 * math.pi * obstacle.radius / spacing)))
        angles = np.arange(count) * 2.0 * math.pi / count
        return obstacle.center + obstacle.radius * np.stack([np.cos(angles), np.sin(angles)], 1)
    half = obstacle.size[:2] / 2.0
    corners = np.array([[half[0], half[1]], [-half[0], half[1]], [-half[0], -half[1]],
                        [half[0], -half[1]], [half[0], half[1]]])
    local = []
    for a, b in zip(corners[:-1], corners[1:]):
        count = max(1, int(math.ceil(np.linalg.norm(b - a) / spacing)))
        steps = np.arange(count)[:, None] / count
        local.append(a + steps * (b - a))
    return world_from_local(np.concatenate(local), obstacle.center, obstacle.yaw)


def world_from_local(points, center, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    points = np.asarray(points, dtype=np.float64)
    return np.stack([c * points[:, 0] - s * points[:, 1],
                     s * points[:, 0] + c * points[:, 1]], 1) + np.asarray(center)[:2]


def inner_wall_points(arena, spacing=0.05):
    """The arena walls' faces toward the arena: its rectangle's outline."""
    (x0, y0), (x1, y1) = arena['min_xy'], arena['max_xy']
    box = rts.Obstacle('arena', 'box', ((x0 + x1) / 2.0, (y0 + y1) / 2.0),
                       (x1 - x0, y1 - y0, 1.0))
    return outline_points(box, spacing)


def obstacle_distance(points, obstacle):
    """Distance from floor points (N, 2) to an obstacle's footprint (0 inside)."""
    points = np.asarray(points, dtype=np.float64)
    if obstacle.kind == 'cylinder':
        return np.maximum(np.linalg.norm(points - obstacle.center, axis=1) - obstacle.radius, 0.0)
    local = obstacle._local(points)
    half = obstacle.size[:2] / 2.0
    return np.linalg.norm(np.maximum(np.abs(local) - half, 0.0), axis=1)


def outlines(scenario, spacing=0.05):
    """{name: outline points}: the arena's inner wall faces and every obstacle."""
    result = {'arena_walls': inner_wall_points(scenario['arena'], spacing)}
    for entry in scenario.get('obstacles', []):
        obstacle = rts.Obstacle.from_json(entry)
        result[obstacle.name] = outline_points(obstacle, spacing)
    return result


def spawn_from_scenario(scenario):
    spawn = scenario.get('spawn', {})
    return rts.Spawn(spawn.get('x', 0.0), spawn.get('y', 0.0),
                     math.radians(float(spawn.get('yaw_deg', 0.0))))


def to_spawn_frame(points, spawn):
    """World floor points into the spawn (map/odom) frame."""
    points = np.asarray(points, dtype=np.float64)
    full = np.column_stack([points, np.zeros(len(points))])
    return np.array([spawn.point(p)[:2] for p in full]).reshape(-1, 2)


def grid_cells(data, origin, resolution, width, height, transform=(0.0, 0.0, 0.0)):
    """Centres (N, 2) of every cell of a row-major grid (rows along y)
    whose origin is the corner of cell (0, 0), in the frame `transform`
    (x, y, yaw of the grid's frame) maps it to."""
    rows, cols = np.divmod(np.arange(width * height), width)
    x = origin[0] + (cols + 0.5) * resolution
    y = origin[1] + (rows + 0.5) * resolution
    tx, ty, yaw = transform
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([tx + c * x - s * y, ty + s * x + c * y], 1), np.asarray(data).reshape(-1)


def slice_cells(snapshot):
    """(occupied centres, known centres) of an nvblox map slice in map."""
    centres, values = grid_cells(snapshot['slice_data'], snapshot['slice_origin'],
                                 float(snapshot['slice_resolution']), int(snapshot['slice_width']),
                                 int(snapshot['slice_height']), snapshot['slice_transform'])
    known = np.isfinite(values) & (values != float(snapshot['slice_unknown']))
    occupied = known & (values <= 0.5 * float(snapshot['slice_resolution']))
    return centres[occupied], centres[known]


def costmap_cells(snapshot):
    """(lethal centres, known centres) of an OccupancyGrid costmap."""
    centres, values = grid_cells(snapshot['costmap_data'], snapshot['costmap_origin'],
                                 float(snapshot['costmap_resolution']),
                                 int(snapshot['costmap_width']), int(snapshot['costmap_height']))
    return centres[values == LETHAL], centres[values >= 0]


def near(points, cells, tolerance):
    """For each point, whether a cell centre lies within tolerance."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    hit = np.zeros(len(points), dtype=bool)
    if len(points) == 0 or len(cells) == 0:
        return hit
    keys = cell_keys(cells, tolerance)
    lookup = {}
    for index, key in enumerate(map(tuple, keys)):
        lookup.setdefault(key, []).append(index)
    point_keys = cell_keys(points, tolerance)
    cells = np.asarray(cells, dtype=np.float64)
    for i, (kx, ky) in enumerate(point_keys):
        candidates = [j for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                      for j in lookup.get((kx + dx, ky + dy), ())]
        if candidates and np.min(np.linalg.norm(cells[candidates] - points[i], axis=1)) <= tolerance:
            hit[i] = True
    return hit


def cell_keys(points, size):
    return np.floor(np.asarray(points, dtype=np.float64) / size).astype(np.int64)


def phantom_mask(cells, obstacles, people, clearance, person_radius):
    """Occupied cells farther than clearance from every obstacle footprint
    and from every recorded person position (person_radius + clearance)."""
    cells = np.asarray(cells, dtype=np.float64).reshape(-1, 2)
    far = np.ones(len(cells), dtype=bool)
    for obstacle in obstacles:
        far &= obstacle_distance(cells, obstacle) > clearance
    if len(people) and far.any():
        far[far] = ~near(cells[far], people, person_radius + clearance)
    return far


def inside_arena(cells, arena, spawn):
    """Whether spawn-frame cells lie inside the arena's rectangle."""
    (x0, y0), (x1, y1) = arena['min_xy'], arena['max_xy']
    cells = np.asarray(cells, dtype=np.float64).reshape(-1, 2)
    c, s = math.cos(spawn.yaw), math.sin(spawn.yaw)
    cells = np.stack([spawn.x + c * cells[:, 0] - s * cells[:, 1],
                      spawn.y + s * cells[:, 0] + c * cells[:, 1]], 1)
    return (cells[:, 0] >= x0) & (cells[:, 0] <= x1) & (cells[:, 1] >= y0) & (cells[:, 1] <= y1)


# --- scoring -------------------------------------------------------------------

def load_snapshots(directory):
    snapshots = []
    for path in sorted(Path(directory).glob('snapshot_*.npz')):
        try:
            with np.load(path) as data:
                snapshots.append({key: data[key] for key in data.files})
        except (OSError, ValueError, EOFError, zipfile.BadZipFile):  # cut short by the teardown
            continue
    return snapshots


def score(snapshots, scenario, tolerance=0.15, clearance=0.3, cell=0.05):
    """Coverage and phantoms of each source over the snapshots (see the
    module docstring); scenario geometry in the spawn frame."""
    spawn = spawn_from_scenario(scenario)
    obstacles = [rts.Obstacle(o.name, o.kind, to_spawn_frame([o.center], spawn)[0],
                              size=getattr(o, 'size', None), radius=getattr(o, 'radius', None),
                              height=o.height, yaw=o.yaw - spawn.yaw)
                 for o in rts.scenario_obstacles(scenario)]
    shapes = {name: to_spawn_frame(points, spawn) for name, points in outlines(scenario).items()}
    arena = scenario['arena']
    person_radius = float(scenario.get('people', {}).get('collision_radius_m', 0.22))
    people = np.concatenate([s['people'].reshape(-1, 3)[:, 1:] for s in snapshots
                             if 'people' in s] or [np.empty((0, 2))])
    result = {'snapshots': len(snapshots), 'tolerance_m': tolerance, 'clearance_m': clearance}
    sources = {'nvblox': ('slice_data', slice_cells), 'costmap': ('costmap_data', costmap_cells)}
    for source, (key, cells_of) in sources.items():
        usable = [s for s in snapshots if key in s]
        if not usable:
            result[source] = None
            continue
        hit = {name: np.zeros(len(p), dtype=bool) for name, p in shapes.items()}
        seen = {name: np.zeros(len(p), dtype=bool) for name, p in shapes.items()}
        phantom_keys = set()
        for snapshot in usable:
            occupied, known = cells_of(snapshot)
            for name, points in shapes.items():
                hit[name] |= near(points, occupied, tolerance)
                if source == 'nvblox':
                    seen[name] |= near(points, known, tolerance)
            phantom = occupied[phantom_mask(occupied, obstacles, people, clearance, person_radius)]
            phantom_keys.update(map(tuple, cell_keys(phantom, cell)))
        last_occupied, _ = cells_of(usable[-1])
        last_phantom = last_occupied[phantom_mask(last_occupied, obstacles, people, clearance,
                                                  person_radius)]
        phantom_cells = (np.array(sorted(phantom_keys), dtype=np.float64) + 0.5) * cell \
            if phantom_keys else np.empty((0, 2))
        inside = inside_arena(phantom_cells, arena, spawn)
        resolution = float(usable[-1]['slice_resolution' if source == 'nvblox'
                                       else 'costmap_resolution'])
        total = sum(len(p) for p in shapes.values())
        entry = {
            'outline_occupied': round(sum(int(h.sum()) for h in hit.values()) / total, 3),
            'outline_occupied_last': round(sum(
                int(near(p, last_occupied, tolerance).sum()) for p in shapes.values()) / total, 3),
            'obstacles': {name: round(float(h.mean()), 3) for name, h in hit.items()},
            'phantom_m2_inside_arena': round(float(inside.sum()) * cell * cell, 4),
            'phantom_m2_outside_arena': round(float((~inside).sum()) * cell * cell, 4),
            'phantom_m2_last_snapshot': round(len(last_phantom) * resolution ** 2, 4),
        }
        if source == 'nvblox':
            entry['outline_observed'] = round(sum(int(s.sum()) for s in seen.values()) / total, 3)
            entry['observed_not_occupied'] = round(sum(
                int((seen[n] & ~hit[n]).sum()) for n in shapes) / total, 3)
        result[source] = entry
    return result


def print_summary(label, result):
    for source in ('nvblox', 'costmap'):
        entry = result.get(source)
        if not entry:
            print(f'{label} {source}: no snapshots')
            continue
        observed = (f", observed {entry['outline_observed']:.0%}, observed but never occupied "
                    f"{entry['observed_not_occupied']:.0%}") if 'outline_observed' in entry else ''
        print(f"{label} {source}: obstacle outlines occupied {entry['outline_occupied']:.0%} "
              f"(last snapshot {entry['outline_occupied_last']:.0%}){observed}; phantoms "
              f"{entry['phantom_m2_inside_arena']:.2f} m^2 inside the arena, "
              f"{entry['phantom_m2_outside_arena']:.2f} m^2 outside")


# --- recording -----------------------------------------------------------------

def yaw_of_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def record(args, ros_args):
    import rclpy
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from nav_msgs.msg import OccupancyGrid
    from nvblox_msgs.msg import DistanceMapSlice
    from std_msgs.msg import String
    import tf2_ros

    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class Recorder(Node):
        def __init__(self):
            super().__init__('racecar_turret_map_recorder')
            self.slice = self.costmap = None
            self.people = []
            self.vehicle = None
            self.count = 0
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.create_subscription(DistanceMapSlice, args.slice_topic, self.on_slice, 1)
            self.create_subscription(OccupancyGrid, args.costmap_topic, self.on_costmap, latched)
            self.create_subscription(String, rts.TRUTH_TOPIC, self.on_truth, 10)
            self.create_timer(args.period_s, self.snapshot)
            self.get_logger().info(f'snapshots of {args.slice_topic} and {args.costmap_topic} '
                                   f'every {args.period_s:g} s into {output}')

        def on_slice(self, msg):
            self.slice = msg

        def on_costmap(self, msg):
            self.costmap = msg

        def on_truth(self, msg):
            try:
                truth = json.loads(msg.data)
            except ValueError:
                return
            t = float(truth.get('stamp', 0.0))
            if self.people and t - self.people[-1][0] < 0.5:
                return
            self.vehicle = truth.get('vehicle')
            for target in truth.get('targets', []):
                x, y = target['position'][:2]
                self.people.append((t, float(x), float(y)))

        def snapshot(self):
            arrays = {'time': np.float64(self.get_clock().now().nanoseconds / 1e9)}
            if self.slice is not None:
                msg = self.slice
                try:
                    tf = self.tf_buffer.lookup_transform(
                        args.map_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp),
                        timeout=Duration(seconds=0.2))
                except tf2_ros.TransformException:
                    try:
                        tf = self.tf_buffer.lookup_transform(args.map_frame, msg.header.frame_id, Time())
                    except tf2_ros.TransformException as exc:
                        self.get_logger().warn(f'no {args.map_frame} <- {msg.header.frame_id}: {exc}',
                                               throttle_duration_sec=10.0)
                        tf = None
                if tf is not None:
                    t = tf.transform
                    arrays.update({
                        'slice_data': np.asarray(msg.data, dtype=np.float32),
                        'slice_origin': np.array([msg.origin.x, msg.origin.y]),
                        'slice_resolution': np.float64(msg.resolution),
                        'slice_width': np.int64(msg.width), 'slice_height': np.int64(msg.height),
                        'slice_unknown': np.float32(msg.unknown_value),
                        'slice_transform': np.array([t.translation.x, t.translation.y,
                                                     yaw_of_quaternion(t.rotation)]),
                    })
            if self.costmap is not None:
                msg = self.costmap
                arrays.update({
                    'costmap_data': np.asarray(msg.data, dtype=np.int8),
                    'costmap_origin': np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
                    'costmap_resolution': np.float64(msg.info.resolution),
                    'costmap_width': np.int64(msg.info.width),
                    'costmap_height': np.int64(msg.info.height),
                })
            arrays['people'] = np.array(self.people, dtype=np.float64).reshape(-1, 3)
            self.people = []
            if self.vehicle is not None:
                arrays['vehicle'] = np.array([self.vehicle['x'], self.vehicle['y'], self.vehicle['yaw']])
            if 'slice_data' not in arrays and 'costmap_data' not in arrays:
                return
            self.count += 1
            path = output / f'snapshot_{self.count:04d}.npz'
            np.savez_compressed(output / f'.{path.name}', **arrays)
            (output / f'.{path.name}').rename(path)

    rclpy.init(args=ros_args)
    node = Recorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def parse_args(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    ros_args = []
    if '--ros-args' in argv:
        index = argv.index('--ros-args')
        argv, ros_args = argv[:index], argv[index:]
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = parser.add_subparsers(dest='command', required=True)
    rec = sub.add_parser('record')
    rec.add_argument('--output-dir', required=True)
    rec.add_argument('--period-s', type=float, default=5.0)
    rec.add_argument('--slice-topic', default=SLICE_TOPIC)
    rec.add_argument('--costmap-topic', default=COSTMAP_TOPIC)
    rec.add_argument('--map-frame', default='map')
    sc = sub.add_parser('score')
    sc.add_argument('--run-dir', required=True, help='a run directory with maps/ (or maps/ itself)')
    sc.add_argument('--scenario', required=True)
    sc.add_argument('--tolerance', type=float, default=0.15)
    sc.add_argument('--clearance', type=float, default=0.3)
    sc.add_argument('--label', default='')
    sc.add_argument('--output', default='')
    return parser.parse_args(argv), ros_args


def main(argv=None):
    args, ros_args = parse_args(argv)
    if args.command == 'record':
        return record(args, ros_args)
    run = Path(args.run_dir)
    snapshots = load_snapshots(run / 'maps' if (run / 'maps').is_dir() else run)
    if not snapshots:
        print(f'no map snapshots in {run}', file=sys.stderr)
        return 2
    result = score(snapshots, json.loads(Path(args.scenario).read_text()), args.tolerance,
                   args.clearance)
    if args.output:
        Path(args.output).write_text(json.dumps(result, indent=2) + '\n')
    print_summary(args.label or run.name, result)
    return 0


if __name__ == '__main__':
    sys.exit(main())
