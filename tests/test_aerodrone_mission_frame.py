import importlib.util
import json
from xml.etree import ElementTree
from pathlib import Path
import re
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np


try:
    import cv2  # noqa: F401
except ModuleNotFoundError:
    # The frame-only unit test does not call any image operations.
    sys.modules["cv2"] = SimpleNamespace()


ROOT = Path(__file__).resolve().parents[1]


def load_module(name, relative_path):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


load_module(
    "terrain_collisions", "applications/demnav/scripts/terrain_collisions.py")
terrain_builder = load_module(
    "build_georeferenced_terrain",
    "applications/demnav/scripts/build_georeferenced_terrain.py")


class MissionFrameTest(unittest.TestCase):
    def test_behavior_tree_uses_vehicle_home_relative_coordinates(self):
        mission = {
            "missions": [{
                "id": "frame_test",
                "phases": [{
                    "name": "flight",
                    "waypoints": [
                        {"id": "arm_start", "position_enu_m": [-720, -260, 0]},
                        {
                            "id": "clear_pad",
                            "position_enu_m": [-700, -240, 35],
                            "target_agl_m": 35,
                            "speed_limit_mps": 2,
                        },
                    ],
                }],
            }],
        }
        grid = np.full((9, 9), 100.0, dtype=np.float32)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            mission_path = output / "mission.json"
            mission_path.write_text(json.dumps(mission), encoding="utf-8")
            args = SimpleNamespace(
                mission=str(mission_path), origin_alt=80.0, area_km=8.0,
                cruise_agl=0.0)

            count, home = terrain_builder.write_behavior_tree(grid, args, output)
            xml = (output / "wilderness_mission.xml").read_text(encoding="utf-8")

        self.assertEqual(count, 1)
        self.assertEqual(home, (-720.0, -260.0))
        self.assertIn('pose="20.000;20.000;34.800"', xml)
        self.assertGreaterEqual(
            xml.count('<Delay delay_msec="2000"><AlwaysSuccess/></Delay>'), 2)
        self.assertNotIn('pose="-700.000;-240.000', xml)

    def test_cruise_altitude_raises_only_en_route_waypoints(self):
        def waypoint(name, agl):
            return {"id": name, "position_enu_m": [-700, -240, agl],
                    "target_agl_m": agl}
        mission = {"missions": [{"id": "cruise_test", "phases": [
            {"name": "takeoff", "waypoints": [
                {"id": "arm_start", "position_enu_m": [-720, -260, 0]},
                waypoint("clear_pad", 35)]},
            {"name": "climb", "waypoints": [
                waypoint("low", 55), waypoint("high", 300)]},
            {"name": "land", "waypoints": [waypoint("flare", 8)]},
        ]}]}
        grid = np.full((9, 9), 100.0, dtype=np.float32)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            mission_path = output / "mission.json"
            mission_path.write_text(json.dumps(mission), encoding="utf-8")
            args = SimpleNamespace(
                mission=str(mission_path), origin_alt=80.0, area_km=8.0,
                cruise_agl=250.0)
            terrain_builder.write_behavior_tree(grid, args, output)
            xml = (output / "wilderness_mission.xml").read_text(encoding="utf-8")

        # Flat terrain 20 m above the origin; home reference 0.2 m above it.
        heights = [float(pose.split(";")[2])
                   for pose in re.findall(r'pose="([^"]+)"', xml)]
        # Each waypoint's GoTo goes out twice.
        self.assertEqual(heights, [34.8, 34.8, 249.8, 249.8, 299.8, 299.8, 7.8, 7.8])
        self.assertEqual(terrain_builder.waypoint_agl("climb", 55.0, 0.0), 55.0)

    def test_each_waypoint_is_a_pair_of_gotos_with_a_localization_update_between(self):
        # as2 1.1.3's GoTo can check a new goal against the previous goal's
        # distance and succeed at once; the second GoTo's previous goal is
        # the same waypoint. A GoTo sent before a state has arrived since
        # the last one ended is rejected, so a delay follows each.
        mission = {"missions": [{"id": "m", "phases": [{"name": "loiter", "waypoints": [
            {"id": "arm_start", "position_enu_m": [-720, -260, 0]},
            {"id": "a", "position_enu_m": [-700, -240, 35], "speed_limit_mps": 3},
            {"id": "b", "position_enu_m": [-650, -240, 35], "hold_s": 8.0},
        ]}]}]}
        grid = np.full((9, 9), 100.0, dtype=np.float32)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            mission_path = output / "mission.json"
            mission_path.write_text(json.dumps(mission), encoding="utf-8")
            args = SimpleNamespace(mission=str(mission_path), origin_alt=80.0,
                                   area_km=8.0, cruise_agl=0.0)
            count, _ = terrain_builder.write_behavior_tree(grid, args, output)
            tree = ElementTree.parse(output / "wilderness_mission.xml")
        self.assertEqual(count, 2)
        steps = list(tree.find("./BehaviorTree/Sequence"))
        first = [step.get("ID") for step in steps].index("GoTo")
        waypoints = steps[first:first + 8]
        self.assertEqual(steps[first + 8].get("ID"), "Land")
        for pair, (pose, speed, settle) in zip(
                (waypoints[:4], waypoints[4:]),
                (("20.000;20.000;34.800", "3.00", "2000"),
                 ("70.000;20.000;34.800", "2.00", "8000"))):
            go_to, between, again, after = pair
            self.assertEqual((go_to.get("ID"), go_to.get("pose"), go_to.get("max_speed")),
                             ("GoTo", pose, speed))
            self.assertEqual(again.attrib, go_to.attrib)
            self.assertEqual((between.tag, between.get("delay_msec")), ("Delay", "2000"))
            self.assertEqual((after.tag, after.get("delay_msec")), ("Delay", settle))

    def test_mission_ends_with_a_disarm_that_tolerates_landing_disarmed(self):
        # Aerostack2's Land disarms on touchdown, and its platform then
        # rejects another disarm: that must not fail the mission.
        mission = {"missions": [{"id": "m", "phases": [{"name": "takeoff", "waypoints": [
            {"id": "arm_start", "position_enu_m": [0, 0, 0]}]}]}]}
        grid = np.full((9, 9), 100.0, dtype=np.float32)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            mission_path = output / "mission.json"
            mission_path.write_text(json.dumps(mission), encoding="utf-8")
            args = SimpleNamespace(mission=str(mission_path), origin_alt=80.0,
                                   area_km=8.0, cruise_agl=0.0)
            terrain_builder.write_behavior_tree(grid, args, output)
            tree = ElementTree.parse(output / "wilderness_mission.xml")
        steps = list(tree.find("./BehaviorTree/Sequence"))
        self.assertEqual(steps[-2].get("ID"), "Land")
        self.assertEqual(steps[-1].tag, "ForceSuccess")
        [disarm] = list(steps[-1])
        self.assertEqual((disarm.tag, disarm.get("ID")), ("Action", "Disarm"))


class TerrainTextureTest(unittest.TestCase):
    def test_mesh_uvs_put_each_vertex_on_its_orthophoto_pixel(self):
        """The renderers (Gazebo, Isaac, Genesis) sample the orthophoto
        mosaic at each vertex's UV (v up, as in OBJ/USD). That pixel must
        be the vertex's own geographic position as WildNav's tile cache
        computes it."""
        from synthetic_world import local_to_latlon
        from wildnav_pkg.satellite_cache import latlon_to_tile, tile_pixel_to_latlon

        lat0, lon0, zoom = 37.9234, -122.5967, 17
        args = SimpleNamespace(lat=lat0, lon=lon0, area_km=2.0, zoom=zoom,
                               origin_alt=700.0, max_grid_size=257)
        # A mosaic covering only part of the terrain, like the real one.
        x0, y0 = (int(value) for value in latlon_to_tile(
            *local_to_latlon(-600.0, 500.0, lat0, lon0), zoom))
        x1, y1 = (int(value) for value in latlon_to_tile(
            *local_to_latlon(400.0, -300.0, lat0, lon0), zoom))
        bounds = (x0, x1, y0, y1, (x1 - x0 + 1) * 256, (y1 - y0 + 1) * 256)
        grid = np.full((21, 21), 750.0, dtype=np.float32)
        with tempfile.TemporaryDirectory() as directory:
            terrain_builder.make_mesh(grid, args, bounds, Path(directory))
            mesh = np.load(Path(directory) / "terrain_mesh.npz")
            vertices, uvs = mesh["vertices"], mesh["uvs"]

        width_px, height_px = bounds[4], bounds[5]
        for (east, north, _), (u, v) in zip(vertices, uvs):
            lat, lon = tile_pixel_to_latlon(
                x0, y0, float(u) * width_px, (1.0 - float(v)) * height_px, zoom)
            expected = local_to_latlon(float(east), float(north), lat0, lon0)
            self.assertAlmostEqual(lat, expected[0], delta=1e-7)  # ~1 cm
            self.assertAlmostEqual(lon, expected[1], delta=1e-7)


if __name__ == "__main__":
    unittest.main()
