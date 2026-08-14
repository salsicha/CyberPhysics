import importlib.util
import json
from pathlib import Path
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
                mission=str(mission_path), origin_alt=80.0, area_km=8.0)

            count, home = terrain_builder.write_behavior_tree(grid, args, output)
            xml = (output / "wilderness_mission.xml").read_text(encoding="utf-8")

        self.assertEqual(count, 1)
        self.assertEqual(home, (-720.0, -260.0))
        self.assertIn('pose="20.000;20.000;34.800"', xml)
        self.assertGreaterEqual(
            xml.count('<Delay delay_msec="2000"><AlwaysSuccess/></Delay>'), 2)
        self.assertNotIn('pose="-700.000;-240.000', xml)


if __name__ == "__main__":
    unittest.main()
