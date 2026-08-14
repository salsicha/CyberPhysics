#!/usr/bin/env python3
"""Build a simulator-ready, georeferenced terrain from real nav caches."""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
from pathlib import Path
import re
import shutil

import cv2
import numpy as np

from terrain_collisions import terrain_collision_height, write_terrain_collision_model


METERS_PER_DEG_LAT = 111_320.0


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--lat", type=float, required=True)
    parser.add_argument("--lon", type=float, required=True)
    parser.add_argument("--origin-alt", type=float, required=True)
    parser.add_argument("--area-km", type=float, required=True)
    parser.add_argument("--resolution-m", type=float, required=True)
    parser.add_argument("--zoom", type=int, required=True)
    parser.add_argument("--dem-cache", default="/data/demnav_cache")
    parser.add_argument("--imagery-cache", default="/data/wildnav_cache")
    parser.add_argument("--output-dir", default="/data/navsim_assets")
    parser.add_argument("--mission", required=True)
    parser.add_argument("--max-grid-size", type=int, default=257)
    return parser.parse_args()


def find_dem(args) -> Path:
    patterns = [
        f"dem_srtm_{args.lat:.4f}_{args.lon:.4f}_{args.area_km:.1f}_{args.resolution_m:.0f}.npy",
        f"dem_{args.lat:.4f}_{args.lon:.4f}_{args.area_km:.1f}_{args.resolution_m:.0f}.npy",
    ]
    for name in patterns:
        candidate = Path(args.dem_cache) / name
        if candidate.is_file():
            return candidate
    candidates = sorted(glob.glob(os.path.join(args.dem_cache, "dem_srtm_*.npy")))
    if not candidates:
        candidates = sorted(glob.glob(os.path.join(args.dem_cache, "dem_*.npy")))
    if not candidates:
        raise FileNotFoundError("no real SRTM DEM exists in the DemNav cache")
    return Path(candidates[-1])


def real_tiles(cache_dir: str, zoom: int):
    pattern = re.compile(rf"^{zoom}_(\d+)_(\d+)\.jpg$")
    result = {}
    for path in Path(cache_dir).glob(f"{zoom}_*_*.jpg"):
        match = pattern.match(path.name)
        if match:
            result[(int(match.group(1)), int(match.group(2)))] = path
    if not result:
        raise FileNotFoundError(
            f"no real zoom-{zoom} orthophoto tiles exist in {cache_dir}")
    return result


def build_mosaic(tiles, output: Path):
    xs = [key[0] for key in tiles]
    ys = [key[1] for key in tiles]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    sample = cv2.imread(str(next(iter(tiles.values()))), cv2.IMREAD_COLOR)
    if sample is None:
        raise RuntimeError("cached orthophoto tile could not be decoded")
    tile_h, tile_w = sample.shape[:2]
    mosaic = np.zeros(((max_y - min_y + 1) * tile_h,
                       (max_x - min_x + 1) * tile_w, 3), dtype=np.uint8)
    missing = 0
    for y in range(min_y, max_y + 1):
        for x in range(min_x, max_x + 1):
            path = tiles.get((x, y))
            image = cv2.imread(str(path), cv2.IMREAD_COLOR) if path else None
            if image is None:
                missing += 1
                continue
            row = (y - min_y) * tile_h
            col = (x - min_x) * tile_w
            mosaic[row:row + tile_h, col:col + tile_w] = image
    if missing:
        raise RuntimeError(
            f"orthophoto cache has {missing} holes inside its rectangular footprint")
    temporary = output.with_name(f".{output.stem}.tmp{output.suffix}")
    if not cv2.imwrite(str(temporary), mosaic, [cv2.IMWRITE_JPEG_QUALITY, 92]):
        raise RuntimeError(f"failed to write {temporary}")
    os.replace(temporary, output)
    return min_x, max_x, min_y, max_y, mosaic.shape[1], mosaic.shape[0]


def latlon_to_tile(lat, lon, zoom):
    scale = 2 ** zoom
    x = (lon + 180.0) / 360.0 * scale
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * scale
    return x, y


def make_mesh(grid, args, tile_bounds, output_dir):
    rows, cols = grid.shape
    stride = max(1, int(math.ceil(max(rows, cols) / args.max_grid_size)))
    row_ids = list(range(0, rows, stride))
    col_ids = list(range(0, cols, stride))
    if row_ids[-1] != rows - 1:
        row_ids.append(rows - 1)
    if col_ids[-1] != cols - 1:
        col_ids.append(cols - 1)

    half_m = args.area_km * 500.0
    east_values = np.linspace(-half_m, half_m, cols)[col_ids]
    north_values = np.linspace(half_m, -half_m, rows)[row_ids]
    min_tx, max_tx, min_ty, max_ty, _, _ = tile_bounds
    tile_span_x = max_tx + 1 - min_tx
    tile_span_y = max_ty + 1 - min_ty

    vertices = []
    uvs = []
    for row_index, north in zip(row_ids, north_values):
        lat = args.lat + north / METERS_PER_DEG_LAT
        for col_index, east in zip(col_ids, east_values):
            lon = args.lon + east / (
                METERS_PER_DEG_LAT * math.cos(math.radians(args.lat)))
            tile_x, tile_y = latlon_to_tile(lat, lon, args.zoom)
            vertices.append((east, north, float(grid[row_index, col_index] - args.origin_alt)))
            uvs.append(((tile_x - min_tx) / tile_span_x,
                        1.0 - (tile_y - min_ty) / tile_span_y))

    mesh_rows = len(row_ids)
    mesh_cols = len(col_ids)
    faces = []
    for row in range(mesh_rows - 1):
        for col in range(mesh_cols - 1):
            a = row * mesh_cols + col
            b = a + 1
            c = a + mesh_cols
            d = c + 1
            faces.extend(((a, c, b), (b, c, d)))

    np.savez_compressed(
        output_dir / "terrain_mesh.npz",
        vertices=np.asarray(vertices, dtype=np.float32),
        uvs=np.asarray(uvs, dtype=np.float32),
        faces=np.asarray(faces, dtype=np.int32),
    )
    with (output_dir / "terrain.obj").open("w", encoding="utf-8") as stream:
        stream.write("mtllib terrain.mtl\nusemtl orthophoto\n")
        for x, y, z in vertices:
            stream.write(f"v {x:.3f} {y:.3f} {z:.3f}\n")
        for u, v in uvs:
            stream.write(f"vt {u:.8f} {v:.8f}\n")
        # DART/ODE requires one normal per mesh vertex; omitting these crashes
        # the Gazebo 6 collision backend while it imports the triangulated DEM.
        for _ in vertices:
            stream.write("vn 0.0 0.0 1.0\n")
        for a, b, c in faces:
            stream.write(
                f"f {a + 1}/{a + 1}/{a + 1} "
                f"{b + 1}/{b + 1}/{b + 1} "
                f"{c + 1}/{c + 1}/{c + 1}\n")
    (output_dir / "terrain.mtl").write_text(
        "newmtl orthophoto\nKa 0.2 0.2 0.2\nKd 1 1 1\nKs 0 0 0\nmap_Kd orthophoto.jpg\n",
        encoding="utf-8",
    )
    return stride, mesh_rows, mesh_cols


def terrain_height(grid, args, east, north):
    half_m = args.area_km * 500.0
    col = np.clip((east + half_m) / (2.0 * half_m) * (grid.shape[1] - 1),
                  0.0, grid.shape[1] - 1.0)
    row = np.clip((half_m - north) / (2.0 * half_m) * (grid.shape[0] - 1),
                  0.0, grid.shape[0] - 1.0)
    r0, c0 = int(math.floor(row)), int(math.floor(col))
    r1, c1 = min(r0 + 1, grid.shape[0] - 1), min(c0 + 1, grid.shape[1] - 1)
    fr, fc = row - r0, col - c0
    value = ((1 - fr) * ((1 - fc) * grid[r0, c0] + fc * grid[r0, c1]) +
             fr * ((1 - fc) * grid[r1, c0] + fc * grid[r1, c1]))
    return float(value - args.origin_alt)


def write_behavior_tree(grid, args, output_dir):
    mission = json.loads(Path(args.mission).read_text(encoding="utf-8"))
    first_waypoint = mission["missions"][0]["phases"][0]["waypoints"][0]
    home_east, home_north, _ = first_waypoint["position_enu_m"]
    home_east = float(home_east)
    home_north = float(home_north)
    home_reference_up = terrain_collision_height(
        grid, args.origin_alt, args.area_km, home_east, home_north) + 0.2
    actions = [
        '      <Action ID="Arm" service_name="set_arming_state"/>',
        '      <Action ID="Offboard" service_name="set_offboard_mode"/>',
        '      <Action ID="TakeOff" height="35.0" speed="1.5"/>',
        '      <Delay delay_msec="2000"><AlwaysSuccess/></Delay>',
    ]
    waypoint_count = 0
    first_point = (home_east, home_north)
    for mission_entry in mission["missions"]:
        for phase in mission_entry["phases"]:
            for waypoint in phase["waypoints"]:
                world_east, world_north, _ = waypoint["position_enu_m"]
                world_east = float(world_east)
                world_north = float(world_north)
                east = world_east - home_east
                north = world_north - home_north
                if waypoint["id"] in {"arm_start", "touchdown"}:
                    continue
                agl = float(waypoint.get("target_agl_m", waypoint["position_enu_m"][2]))
                up = terrain_collision_height(
                    grid, args.origin_alt, args.area_km, world_east, world_north)
                up += agl - home_reference_up
                speed = float(waypoint.get("speed_limit_mps", 2.0))
                actions.append(
                    f'      <!-- {mission_entry["id"]}/{phase["name"]}/{waypoint["id"]} -->')
                actions.append(
                    f'      <Action ID="GoTo" max_speed="{speed:.2f}" '
                    f'pose="{east:.3f};{north:.3f};{up:.3f}" yaw_angle="0.0" yaw_mode="1"/>')
                hold = float(waypoint.get("hold_s", 0.0))
                # Aerostack2 tears down and recreates the GoTo action context at
                # each waypoint.  Leave at least one localization update between
                # actions; otherwise an already-satisfied waypoint can make the
                # following goal race the state estimator and be rejected as
                # "there is no localization".
                settle_s = max(hold, 2.0)
                actions.append(
                    f'      <Delay delay_msec="{int(settle_s * 1000)}"><AlwaysSuccess/></Delay>')
                waypoint_count += 1
    actions.extend([
        '      <Action ID="Land" speed="0.5"/>',
        '      <Action ID="Disarm" service_name="set_arming_state"/>',
    ])
    xml = "\n".join([
        '<?xml version="1.0"?>',
        '<root main_tree_to_execute="WildernessMission">',
        '  <BehaviorTree ID="WildernessMission">',
        '    <Sequence>',
        *actions,
        '    </Sequence>',
        '  </BehaviorTree>',
        '</root>',
        '',
    ])
    (output_dir / "wilderness_mission.xml").write_text(xml, encoding="utf-8")
    return waypoint_count, first_point


def write_gazebo_assets(args, output_dir, home):
    model_dir = output_dir / "models" / "georeferenced_terrain"
    model_dir.mkdir(parents=True, exist_ok=True)
    for name in ("terrain.obj", "terrain.mtl", "orthophoto.jpg", "terrain_height.png"):
        shutil.copy2(output_dir / name, model_dir / name)
    (model_dir / "model.config").write_text(
        """<?xml version="1.0"?>
<model><name>georeferenced_terrain</name><version>1.0</version>
<sdf version="1.9">model.sdf</sdf></model>
""", encoding="utf-8")
    dem = np.load(find_dem(args))
    min_up = float(dem.min() - args.origin_alt)
    height_span = max(float(dem.max() - dem.min()), 1.0)
    terrain_size = args.area_km * 1000.0
    (model_dir / "model.sdf").write_text(
        f"""<?xml version="1.0"?>
<sdf version="1.9"><model name="georeferenced_terrain"><static>true</static><link name="terrain">
<collision name="collision"><geometry><heightmap><uri>terrain_height.png</uri>
<size>{terrain_size:.3f} {terrain_size:.3f} {height_span:.3f}</size><pos>0 0 {min_up:.3f}</pos>
<sampling>2</sampling></heightmap></geometry></collision>
<visual name="visual"><geometry><mesh><uri>terrain.obj</uri></mesh></geometry></visual>
</link></model></sdf>
""", encoding="utf-8")
    write_terrain_collision_model(
        dem, args.origin_alt, args.area_km, model_dir / "model.sdf")
    world = f"""<?xml version="1.0"?>
<sdf version="1.9"><world name="mount_tamalpais">
<physics name="1ms" type="ignored"><max_step_size>0.005</max_step_size><real_time_factor>1.0</real_time_factor></physics>
<plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
<plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
<plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
<plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
<spherical_coordinates><surface_model>EARTH_WGS84</surface_model><latitude_deg>{args.lat}</latitude_deg>
<longitude_deg>{args.lon}</longitude_deg><elevation>{args.origin_alt}</elevation><heading_deg>0</heading_deg></spherical_coordinates>
<scene><ambient>0.35 0.35 0.35 1</ambient><background>0.55 0.7 0.9 1</background><shadows>true</shadows></scene>
<light type="directional" name="sun"><pose>0 0 1000 0 0 0</pose><diffuse>1 0.96 0.88 1</diffuse>
<specular>0.2 0.2 0.2 1</specular><direction>-0.45 0.2 -0.87</direction><cast_shadows>true</cast_shadows></light>
<include><uri>model://georeferenced_terrain</uri></include>
</world></sdf>
"""
    (output_dir / "mount_tamalpais.sdf").write_text(world, encoding="utf-8")
    east, north = home
    ground = terrain_collision_height(dem, args.origin_alt, args.area_km, east, north)
    gazebo_yaml = f"""world_name: mount_tamalpais
origin:
  latitude: {args.lat}
  longitude: {args.lon}
  altitude: {args.origin_alt}
drones:
  - model_type: quadrotor_base
    model_name: aerodrone
    xyz: [{east:.3f}, {north:.3f}, {ground + 0.2:.3f}]
    rpy: [0.0, 0.0, 0.0]
    enable_velocity_control: false
    payload:
      - model_type: hd_camera
        model_name: downward_rgb
        xyz: [0.0, 0.0, -0.05]
        rpy: [0.0, 1.57079632679, 0.0]
      - model_type: rgbd_camera
        model_name: downward_rgbd
        xyz: [0.0, 0.0, -0.06]
        rpy: [0.0, 1.57079632679, 0.0]
objects: []
"""
    (output_dir / "world_gazebo.yaml").write_text(gazebo_yaml, encoding="utf-8")
    return ground


def main():
    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    dem_path = find_dem(args)
    grid = np.load(dem_path).astype(np.float32)
    if not np.isfinite(grid).all() or grid.size < 4:
        raise RuntimeError(f"invalid DEM: {dem_path}")
    # Gazebo collision uses a 2^n+1, 16-bit heightmap derived from the same
    # real DEM as the textured render mesh and terrain-aware mission.
    collision_grid = cv2.resize(grid, (513, 513), interpolation=cv2.INTER_AREA)
    height_min = float(collision_grid.min())
    height_span = max(float(collision_grid.max() - height_min), 1.0)
    height_pixels = np.rint((collision_grid - height_min) / height_span * 65535.0).astype(np.uint16)
    if not cv2.imwrite(str(output_dir / "terrain_height.png"), height_pixels):
        raise RuntimeError("failed to write Gazebo terrain heightmap")
    tiles = real_tiles(args.imagery_cache, args.zoom)
    bounds = build_mosaic(tiles, output_dir / "orthophoto.jpg")
    stride, mesh_rows, mesh_cols = make_mesh(grid, args, bounds, output_dir)
    waypoint_count, home = write_behavior_tree(grid, args, output_dir)
    if home is None:
        raise RuntimeError("mission contains no waypoints")
    home_ground = write_gazebo_assets(args, output_dir, home)
    home_lat = args.lat + home[1] / METERS_PER_DEG_LAT
    home_lon = args.lon + home[0] / (
        METERS_PER_DEG_LAT * math.cos(math.radians(args.lat)))
    home_alt = args.origin_alt + home_ground + 0.2
    manifest = {
        "schema_version": 1,
        "source": {"dem": "SRTM", "imagery": "ArcGIS World Imagery"},
        "origin": {"latitude": args.lat, "longitude": args.lon,
                   "altitude_m": args.origin_alt, "frame": "local_tangent_ENU"},
        "terrain": {"dem_path": str(dem_path), "area_km": args.area_km,
                    "source_resolution_m": args.resolution_m, "mesh_stride": stride,
                    "mesh_rows": mesh_rows, "mesh_cols": mesh_cols,
                    "min_up_m": float(grid.min() - args.origin_alt),
                    "max_up_m": float(grid.max() - args.origin_alt)},
        "imagery": {"zoom": args.zoom, "tile_count": len(tiles),
                    "bounds": list(bounds[:4])},
        "home_enu_m": [home[0], home[1], home_ground + 0.2],
        "home_geodetic": {"latitude": home_lat, "longitude": home_lon,
                            "altitude_m": home_alt},
        "mission_waypoints": waypoint_count,
        "assets": {"mesh": "terrain.obj", "mesh_npz": "terrain_mesh.npz",
                   "texture": "orthophoto.jpg", "behavior_tree": "wilderness_mission.xml"},
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()

