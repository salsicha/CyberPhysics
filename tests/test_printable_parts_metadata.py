"""Printable parts: PART metadata schema and datasheet footprints.

Pure Python (no CadQuery): runs in every test environment.
"""
import importlib
import inspect
import re
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "tools"))

import build_printable_parts as bpp  # noqa: E402

from hardware.parts.common import fasteners, footprints, printing, schema  # noqa: E402

PLATFORM_PARTS = [(p, s) for p in bpp.platforms() for s in bpp.part_stems(p)]


def valid_part(**changes):
    part = {
        "name": "Fixture",
        "quantity": 1,
        "material": "PETG",
        "material_reason": "test",
        "material_fallback": None,
        "flight_critical": False,
        "print": printing.settings("PETG", "flat"),
        "mates_with": [{"part": "x", "dimensions": "y", "source": "z", "verified": True}],
        "inserts": [],
        "fasteners": [],
        "fit_test": False,
        "notes": [],
    }
    part.update(changes)
    return part


def test_there_are_parts():
    assert "aerodrone" in bpp.platforms()
    assert len(PLATFORM_PARTS) >= 10


@pytest.mark.parametrize("platform,stem", PLATFORM_PARTS)
def test_every_part_metadata_is_valid(platform, stem):
    mod = importlib.import_module(f"hardware.parts.{platform}.{stem}")
    assert schema.validate_part(mod.PART, stem) == []
    # a CadQuery part has build(); a vendored one has a pinned file instead
    assert callable(getattr(mod, "build", None)) != ("vendored" in mod.PART)
    # CadQuery must be imported inside build(), not at module level.
    assert not any(obj.__name__.startswith("cadquery") or obj.__name__.endswith("common.cad")
                   for obj in vars(mod).values() if inspect.ismodule(obj))
    src = Path(mod.__file__).read_text()
    assert "\nimport cadquery" not in src and "\nfrom cadquery" not in src


def test_schema_accepts_a_valid_part():
    assert schema.validate_part(valid_part()) == []


@pytest.mark.parametrize("change,needle", [
    ({"material": "PLA"}, "unknown material"),
    ({"quantity": 0}, "quantity"),
    ({"quantity": True}, "'quantity' must be int"),
    ({"material": "PA-CF", "print": printing.settings("PA-CF", "flat")}, "hardened nozzle"),
    ({"material": "PA-CF", "print": printing.settings("PA-CF", "flat"), "material_fallback": "PETG-CF"},
     "non-CF"),
    ({"mates_with": []}, "mates_with is empty"),
    ({"mates_with": [{"part": "x", "dimensions": "y", "source": "z", "verified": False}]}, "fit_test"),
    ({"mates_with": [{"part": "x", "dimensions": "y", "source": "", "verified": True}]}, "no source"),
    ({"inserts": [{"size": "M7", "qty": 1, "where": "x"}]}, "insert size"),
    ({"fasteners": [{"item": "M3 x 8", "qty": 0, "use": "x"}]}, "qty"),
    ({"print": {**printing.settings("PETG", "flat"), "supports": "everywhere"}}, "supports"),
    ({"print": {**printing.settings("PETG", "flat"), "layer_height_mm": 0.5}}, "layer_height"),
    ({"print": {**printing.settings("PETG", "flat"), "rotate": [["Q", 90]]}}, "rotate"),
    ({"print": printing.settings("ASA", "flat")}, "filament_profile"),
])
def test_schema_rejects(change, needle):
    errors = schema.validate_part(valid_part(**change))
    assert any(needle in e for e in errors), errors


VENDORED_PARTS = [(p, s) for p, s in PLATFORM_PARTS
                  if "vendored" in importlib.import_module(f"hardware.parts.{p}.{s}").PART]


@pytest.mark.parametrize("platform,stem", VENDORED_PARTS)
def test_vendored_parts_are_pinned(platform, stem):
    """The committed upstream file is the pinned one, its licence is beside
    it, and the source URL names a commit, not a branch."""
    import hashlib
    v = importlib.import_module(f"hardware.parts.{platform}.{stem}").PART["vendored"]
    path = bpp.vendored_path(platform, {"vendored": v})
    assert path.exists(), path
    assert hashlib.sha256(path.read_bytes()).hexdigest() == v["sha256"]
    data = path.read_bytes()
    assert not data.startswith(b"version https://git-lfs"), f"{path} is a Git LFS pointer"
    assert (bpp.PARTS / platform / v["licence_file"]).exists()
    assert re.search(r"/blob/[0-9a-f]{40}/", v["source"]), v["source"]


def test_vendored_parts_exist():
    assert ("so101", "base_so101") in VENDORED_PARTS


VENDORED = {"file": "upstream/a.stl", "sha256": "0" * 64, "source": "https://example.org/blob/" + "a" * 40 + "/a.stl",
            "licence": "Apache-2.0", "licence_file": "upstream/LICENSE", "author": "someone"}


@pytest.mark.parametrize("change,needle", [
    ({"sha256": "abc"}, "sha256"),
    ({"file": "../elsewhere.stl"}, "inside the platform folder"),
    ({"file": "upstream/a.step"}, "must be an STL"),
    ({"source": "http://example.org/a.stl"}, "https URL"),
    ({"licence": ""}, "misses 'licence'"),
    ({"licence_file": ""}, "misses 'licence_file'"),
])
def test_schema_rejects_bad_vendored_entries(change, needle):
    errors = schema.validate_part(valid_part(vendored={**VENDORED, **change}))
    assert any(needle in e for e in errors), errors


def test_schema_accepts_a_vendored_part():
    assert schema.validate_part(valid_part(vendored=dict(VENDORED))) == []


def test_so101_footprints_cite_sources():
    from hardware.parts.so101 import _footprints
    for fp in _footprints.ALL:
        assert "http" in fp.SOURCE or "Requirement" in fp.SOURCE or "measured" in fp.SOURCE, fp
        for name in fp.UNVERIFIED:
            assert hasattr(fp, name), (fp.__name__, name)


def test_so101_station_matches_the_scene():
    """The station is built from the scene: riser height from the spawn and
    the scene's mount, the camera RGB centre at the scene camera's position
    at its pitch (48.5 deg, the head's recommended trim), and the bins and
    objects standing on the table where the scene puts them."""
    from hardware.parts.so101 import _station as S
    assert S.TABLE_Z == pytest.approx(-53.5)
    assert S.PLATE_TOP - S.PLATE_BOTTOM == pytest.approx(31.0)
    assert S.SCENE["arm_base_mount"]["size"][2] * 1000.0 == pytest.approx(31.0)
    assert S.CAM_XYZ == pytest.approx((169.756, -521.991, 464.613), abs=1e-3)
    assert S.PITCH_DEG == pytest.approx(48.5, abs=1e-3)
    assert S.rgb_centre(S.PITCH_DEG) == pytest.approx(S.CAM_XYZ)
    for name in S.TASK_ITEMS:
        (_, _, cz), (_, _, sz) = S.scene_box(name)
        assert cz - sz / 2 == pytest.approx(S.TABLE_Z), name
    lo, hi = S.PITCH_RANGE_DEG
    assert lo < S.PITCH_DEG < hi
    # turning about the pivot keeps the RGB centre's distance to it
    d0 = sum((a - b) ** 2 for a, b in zip(S.rgb_centre(lo), S.PIVOT))
    d1 = sum((a - b) ** 2 for a, b in zip(S.rgb_centre(hi), S.PIVOT))
    assert d0 == pytest.approx(d1)


def test_lunar_rover_footprints_cite_sources():
    from hardware.parts.lunar_rover import _footprints
    for fp in _footprints.ALL:
        assert "http" in fp.SOURCE or "Requirement" in fp.SOURCE, fp
        for name in fp.UNVERIFIED:
            assert hasattr(fp, name), (fp.__name__, name)


def test_lunar_rover_matches_the_simulator():
    """Wheel radius, top speed and battery of the rover are the simulator's."""
    from hardware.parts.lunar_rover import _frame as F
    from hardware.parts.lunar_rover._footprints import CubeMarsAK70_10, LiFePO4Pack24V
    sim = (REPO / "systems" / "lunar_rover" / "config" / "simulation.yaml").read_text()

    def value(key):
        return float(re.search(rf"^\s*{key}:\s*([0-9.]+)", sim, re.MULTILINE).group(1))
    assert F.SIM_WHEEL_RADIUS == pytest.approx(1000.0 * value("wheel_radius_m"))
    assert F.GROUSER_TIP_R == F.SIM_WHEEL_RADIUS
    assert F.SIM_MAX_SPEED == pytest.approx(value("max_speed_mps"))
    assert F.SIM_BATTERY_WH == pytest.approx(value("battery_capacity_wh"))
    assert abs(LiFePO4Pack24V.CAPACITY_WH - F.SIM_BATTERY_WH) / F.SIM_BATTERY_WH < 0.05
    # the six wheels, their names and equal loads from the pivot positions
    assert [n for n, _, _ in F.wheel_positions()] == [
        "front_left", "front_right", "middle_left", "middle_right", "rear_left", "rear_right"]
    front, middle, rear = F.WHEEL_X
    assert F.BOGIE_PIVOT[0] == pytest.approx((middle + rear) / 2)
    # rocker: front load * arm = bogie (two wheels) load * arm
    assert (front - F.ROCKER_PIVOT[0]) == pytest.approx(2 * (F.ROCKER_PIVOT[0] - F.BOGIE_PIVOT[0]))
    # the AK70-10 at the simulator's top speed and its worst slip (0.65)
    import math
    rpm = F.SIM_MAX_SPEED / (1 - 0.65) / (F.SIM_WHEEL_RADIUS / 1000.0) * 60 / (2 * math.pi)
    assert rpm < CubeMarsAK70_10.RATED_RPM_24V


def test_cf_part_with_fallback_is_valid():
    part = valid_part(material="PETG-CF", print=printing.settings("PETG-CF", "flat"), material_fallback="PETG")
    assert schema.validate_part(part) == []


def test_printer_and_bed():
    assert printing.BED_MM == (250.0, 250.0, 250.0)
    assert all(b <= v for b, v in zip(printing.BED_MM, printing.PRINTER_BUILD_VOLUME_MM))
    assert printing.PRINTER_BUILD_VOLUME_MM == (256.0, 256.0, 256.0)
    assert set(printing.CF_MATERIALS) == {"PETG-CF", "PA-CF"}
    s = printing.settings("ASA", "flat", wall_loops=4, infill_pct=35, rotate=[("X", 180)])
    assert s["filament_profile"] == "Bambu ASA" and s["wall_loops"] == 4
    assert s["sparse_infill_density_pct"] == 35 and s["rotate"] == [["X", 180]]
    assert s["process"] == "0.20mm Standard"


def test_footprints_cite_sources_and_list_real_attributes():
    for fp in footprints.ALL:
        assert "http" in fp.SOURCE or "Requirement" in fp.SOURCE or "No manufacturer" in fp.SOURCE, fp
        for name in fp.UNVERIFIED:
            assert hasattr(fp, name), (fp.__name__, name)


def test_raspberry_pi5_footprint_matches_the_drawing():
    pi = footprints.RaspberryPi5
    assert pi.BOARD == (85.0, 56.0)
    assert pi.HOLE_SPACING == (58.0, 49.0)
    assert pi.HOLE_INSET == 3.5 and pi.HOLE_D == 2.7
    xs = sorted({x for x, _ in pi.HOLES})
    ys = sorted({y for _, y in pi.HOLES})
    assert xs == [3.5, 3.5 + 58.0] and ys == [3.5, 3.5 + 49.0]


def test_navigator_footprint():
    nav = footprints.BlueRoboticsNavigator
    assert nav.BOARD == (65.0, 56.0) and nav.HOLE_SPACING == (58.0, 49.0) and nav.HOLE_D == 2.75
    assert nav.HOLE_SPACING == footprints.RaspberryPi5.HOLE_SPACING
    assert nav.HEATSINK == (65.0, 56.0, 14.0)
    assert nav.SUPPLY_MIN_A == 4.5 and nav.PI_AND_PERIPHERALS_A == (4.0, 4.5)


def test_oak1_footprint():
    oak = footprints.LuxonisOAK1
    assert (oak.WIDTH, oak.HEIGHT, oak.DEPTH) == (36.0, 54.5, 27.8)
    assert oak.LENS_FROM_USB_END == 32.4 and oak.TRIPOD_FROM_USB_END == 14.5 and oak.TRIPOD_DEPTH == 10.0
    assert (oak.HFOV_DEG, oak.VFOV_DEG) == (69.0, 55.0)
    assert oak.MASS_G == 53.1


def test_oak1_fov_matches_the_navigation_stack():
    """FOV_DEG in the BlueOS composition is the OAK-1's horizontal FOV."""
    text = (REPO / "compositions" / "blueos_nav_core.yaml").read_text()
    assert footprints.LuxonisOAK1.HFOV_DEG == 69.0
    assert "fov_deg:=${FOV_DEG:-69.0}" in text


def test_benewake_footprints():
    tf = footprints.BenewakeTFminiS
    assert (tf.LENGTH, tf.WIDTH, tf.HEIGHT) == (42.0, 15.0, 16.0)
    assert tf.HOLE_SPACING == 33.0 and tf.HOLE_D == 2.35
    luna = footprints.BenewakeTFLuna
    assert (luna.LENGTH, luna.WIDTH) == (35.0, 21.25)
    assert luna.HOLE_SPACING == 30.5 and luna.HOLE_D == 2.2


def test_motor_gps_and_tube_footprints():
    m = footprints.Holybro2216KV920
    assert m.MOUNT_PATTERN == (16.0, 16.0) and m.MOUNT_SCREW == "M3"
    assert "MOUNT_LAYOUT" in m.UNVERIFIED
    assert footprints.Propeller1045.DIAMETER == 254.0
    assert footprints.CarbonTube16.OD == 16.0
    assert footprints.HolybroM10GPS.V1 == (50.0, 14.4) and footprints.HolybroM10GPS.V2 == (51.0, 18.5)


def test_insert_table_matches_cnc_kitchen():
    expected = {"M2": (3.0, 3.6, 3.2, 1.3), "M2.5": (4.0, 4.6, 4.0, 1.6), "M3": (5.7, 4.6, 4.0, 1.6),
                "M3-short": (3.0, 4.6, 4.0, 1.6), "M4": (8.1, 6.3, 5.7, 2.1)}
    for key, (length, d1, hole, wall) in expected.items():
        ins = fasteners.INSERTS[key]
        assert (ins.length, ins.diameter, ins.hole, ins.min_wall) == (length, d1, hole, wall)
    assert fasteners.INSERTS["M3"].hole_depth == pytest.approx(6.7)
    assert fasteners.INSERTS["M3"].boss_diameter == pytest.approx(7.2)


def test_iso_273_medium_clearances():
    assert fasteners.CLEARANCE == {"M2": 2.4, "M2.5": 2.9, "M3": 3.4, "M4": 4.5, "M5": 5.5}
    assert fasteners.SHCS_HEAD["M3"] == (5.5, 3.0)
    for size, d in fasteners.CLEARANCE.items():
        assert d > fasteners.MAJOR[size]


def test_aerodrone_layout_is_consistent():
    from hardware.parts.aerodrone import _frame as F
    assert F.WHEELBASE == 500.0 and F.MOTOR_R == 250.0
    assert F.TUBE_LENGTH == 180.0
    # ArduPilot quad X: motors 1/2 CCW, 3/4 CW, at the four diagonals
    assert {a for a, _ in F.MOTORS.values()} == {45.0, 135.0, -135.0, -45.0}
    assert F.MOTORS[1] == (-45.0, "CCW") and F.MOTORS[3] == (45.0, "CW")
    # the Pi holes in the body frame are the drawing's pattern
    holes = F.pi_holes_body()
    xs = sorted({round(x, 6) for x, _ in holes})
    ys = sorted({round(y, 6) for _, y in holes})
    assert xs[1] - xs[0] == pytest.approx(58.0) and ys[1] - ys[0] == pytest.approx(49.0)
    # OAK-1 camera centred so its image top points to the nose (yaw 0)
    assert F.CAMERA_LENS_Z < F.STAGE_Z[0] < F.NOSE_Z[0]
