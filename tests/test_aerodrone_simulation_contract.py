import importlib.util
from pathlib import Path
import struct
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def load_module(name, relative_path):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


ardupilot = load_module(
    "ardupilot_json", "applications/ardupilot_sitl/scripts/ardupilot_json.py")
terrain = load_module(
    "terrain_collisions", "applications/demnav/scripts/terrain_collisions.py")


class ArduPilotJSONTest(unittest.TestCase):
    def test_decodes_16_and_32_channel_packets(self):
        packet16 = struct.pack("<HHI16H", 18458, 400, 42, *range(1000, 1016))
        packet32 = struct.pack("<HHI32H", 29569, 200, 99, *range(1100, 1132))
        frame16 = ardupilot.decode_servo_packet(packet16)
        frame32 = ardupilot.decode_servo_packet(packet32)
        self.assertEqual((frame16.frame_rate_hz, frame16.frame_count), (400, 42))
        self.assertEqual(frame16.pwm[:2], (1000, 1001))
        self.assertEqual((frame32.frame_rate_hz, frame32.frame_count), (200, 99))
        self.assertEqual(len(frame32.pwm), 32)

    def test_rejects_invalid_packets(self):
        self.assertIsNone(ardupilot.decode_servo_packet(b"not a servo packet"))
        bad_magic = struct.pack("<HHI16H", 1, 400, 1, *([1000] * 16))
        self.assertIsNone(ardupilot.decode_servo_packet(bad_magic))

    def test_pwm_mapping_preserves_thrust_curve_and_order(self):
        speeds = ardupilot.pwm_to_rotor_speed(
            [1000, 1250, 2000, 1500], maximum_speed=800.0,
            order=(2, 0, 3, 1))
        expected = 800.0 * np.sqrt([1.0, 0.0, 0.5, 0.25])
        np.testing.assert_allclose(speeds, expected)

    def test_enu_flu_state_becomes_ned_frd(self):
        state = ardupilot.state_from_enu(
            [10.0, 20.0, 30.0], [1.0, 2.0, 3.0], [1.0, 0.0, 0.0, 0.0],
            [0.1, 0.2, 0.3], [1.0, 2.0, 3.0],
            37.0, -122.0, 100.0, timestamp=12.5)
        self.assertEqual(state["timestamp"], 12.5)
        self.assertEqual(state["position"], [20.0, 10.0, -30.0])
        self.assertEqual(state["velocity"], [2.0, 1.0, -3.0])
        self.assertEqual(state["imu"]["gyro"], [0.1, -0.2, -0.3])
        relative = ardupilot.state_from_enu(
            [10.0, 20.0, 30.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0], [0.0, 0.0, 9.80665], 37.0, -122.0, 100.0,
            timestamp=1.0, reference_enu=[4.0, 5.0, 6.0])
        self.assertEqual(relative["position"], [15.0, 6.0, -24.0])
        rotation = ardupilot.quaternion_to_matrix(state["quaternion"])
        np.testing.assert_allclose(
            rotation,
            np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]),
            atol=1e-8)


class TerrainCollisionTest(unittest.TestCase):
    def test_collision_height_is_conservative(self):
        grid = np.arange(81, dtype=np.float32).reshape(9, 9)
        height = terrain.terrain_collision_height(
            grid, origin_alt=20.0, area_km=8.0, east=0.0, north=0.0)
        self.assertGreaterEqual(height, float(grid[4, 4] - 20.0))


class ComposeContractTest(unittest.TestCase):
    def test_all_drone_backends_use_real_stack_contract(self):
        for filename, simulator in (
            ("aerostack2_sim.yaml", "gazebo_ardupilot_backend"),
            ("aerostack2_isaac.yaml", "isaacsim"),
            ("aerostack2_genesis.yaml", "genesis"),
        ):
            text = (ROOT / "compositions" / filename).read_text(encoding="utf-8")
            with self.subTest(filename=filename):
                self.assertIn(simulator, text)
                self.assertIn("navigator_sitl:", text)
                self.assertIn("cyberphysics/navigator:", text)
                self.assertIn("as2_platform_blueos", text)
                self.assertIn("flight_ready:", text)
                self.assertIn("wilderness_mission.xml", text)
                self.assertIn("build_georeferenced_terrain.py", text)
                self.assertNotIn("aerodrone_sensor_sim.py", text)

    def test_boat_and_submarine_use_real_ardupilot_firmware(self):
        boat = (ROOT / "compositions/boat_sim.yaml").read_text(encoding="utf-8")
        sub = (ROOT / "compositions/submarine_sim.yaml").read_text(encoding="utf-8")
        self.assertIn("cyberphysics/navigator:", boat)
        self.assertIn("ArduRover", boat)
        self.assertNotIn("blueboat_sim.py", boat)
        self.assertIn("cyberphysics/navigator:", sub)
        self.assertIn("ArduSub", sub)
        self.assertNotIn("bluerov2_sim.py", sub)


    def test_gazebo_runtime_scripts_are_owned_by_the_gazebo_application(self):
        compose = (ROOT / "compositions/aerostack2_sim.yaml").read_text(
            encoding="utf-8")
        self.assertIn("../applications/gazebo/scripts", compose)
        self.assertNotIn("../systems/aerostack2_gazebo/scripts", compose)
        for filename in ("drone_bridges_isolated.py", "gazebo_ardupilot_backend.py"):
            self.assertTrue((ROOT / "applications/gazebo/scripts" / filename).is_file())


    def test_isaac_compositions_use_the_owned_isaac_image(self):
        dockerfile = (ROOT / "applications/isaac/Dockerfile").read_text(
            encoding="utf-8")
        self.assertIn(
            "FROM nvcr.io/nvidia/isaac-sim:${ISAAC_SIM_VERSION}", dockerfile)
        self.assertIn("ARG ISAAC_SIM_VERSION=6.0.1", dockerfile)
        for filename in (
            "aerostack2_isaac.yaml", "isaacsim.yaml",
            "so101_isaacsim.yaml", "so101_groot_isaac.yaml",
        ):
            compose = (ROOT / "compositions" / filename).read_text(
                encoding="utf-8")
            with self.subTest(filename=filename):
                self.assertIn(
                    "image: cyberphysics/isaac:${CYBERPHYSICS_TAG:-latest}",
                    compose)
                self.assertNotIn("image: nvcr.io/nvidia/isaac-sim", compose)


    def test_systems_contains_one_canonical_aerodrone(self):
        systems = ROOT / "systems"
        drone_systems = sorted(
            path.name for path in systems.iterdir()
            if path.is_dir() and ("drone" in path.name or "aerostack2" in path.name))
        self.assertEqual(drone_systems, ["aerodrone"])
        self.assertTrue((systems / "aerodrone" / "config" / "arducopter.params").is_file())
        for pseudo_system in (
            "aerostack2_common", "aerostack2_gazebo",
            "aerostack2_genesis", "aerostack2_isaac",
        ):
            self.assertFalse((systems / pseudo_system).exists())
        for filename in (
            "aerostack2_sim.yaml", "aerostack2_genesis.yaml",
            "aerostack2_isaac.yaml",
        ):
            compose = (ROOT / "compositions" / filename).read_text(encoding="utf-8")
            with self.subTest(filename=filename):
                self.assertIn("../systems/aerodrone", compose)
                self.assertIn("namespace:=aerodrone", compose)
                self.assertNotIn("drone_sim_0", compose)


if __name__ == "__main__":
    unittest.main()
