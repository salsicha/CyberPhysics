"""Printable parts: the build and validation script, on fixture parts and on
the real ones (committed STLs fresh, every part valid, assembly checks).

Needs CadQuery, trimesh and rtree (applications/cad/requirements.txt):
skipped without them, run by the printable-parts job in ci.yml and in the
cad image.
"""
import math
import sys
import types
from pathlib import Path

import pytest

pytest.importorskip("cadquery")
pytest.importorskip("trimesh")
pytest.importorskip("rtree")

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "tools"))

import build_printable_parts as bpp  # noqa: E402

from hardware.parts.common import cad, printing  # noqa: E402


def fixture_module(build, **part_changes):
    part = {
        "name": "Fixture",
        "quantity": 2,
        "material": "PETG",
        "material_reason": "test",
        "material_fallback": None,
        "flight_critical": False,
        "print": printing.settings("PETG", "flat"),
        "mates_with": [{"part": "x", "dimensions": "y", "source": "test", "verified": True}],
        "inserts": [],
        "fasteners": [],
        "fit_test": False,
        "notes": [],
    }
    part.update(part_changes)
    mod = types.ModuleType("fixture")
    mod.PART = part
    mod.build = build
    return mod


def plate_with_hole(t=4.0, size=(40.0, 30.0)):
    def build():
        plate = cad.box(-size[0] / 2, size[0] / 2, -size[1] / 2, size[1] / 2, 0.0, t)
        return plate.cut(cad.cyl(3.4, t + 2, (0, 0, -1)))
    return build


def run(tmp_path, mod, stem="fixture"):
    return bpp.build_part("fixture", stem, mod, tmp_path / f"{stem}.stl", tmp_path / f"{stem}.step")


def test_fixture_part_builds_and_validates(tmp_path):
    entry, _ = run(tmp_path, fixture_module(plate_with_hole()))
    v = entry["validation"]
    assert v["passed"], v["errors"]
    assert v["closed_manifold"] and v["bodies"] == 1 and v["fits_bed"]
    assert v["min_wall_mm"] == pytest.approx(4.0, abs=0.05)
    assert entry["print_size_mm"] == [40.0, 30.0, 4.0]
    assert entry["volume_cm3"] == pytest.approx((40 * 30 * 4 - 3.14159 * 1.7 ** 2 * 4) / 1000, abs=0.01)
    assert (tmp_path / "fixture.stl").exists() and (tmp_path / "fixture.step").exists()
    est = entry["estimate"]
    assert 0 < est["mass_g"] < 40 * 30 * 4 / 1000 * 1.27 + 0.01
    assert est["mass_g_all"] == pytest.approx(2 * est["mass_g"], abs=0.11)
    assert v["bed_contact_mm2"] > 1000


def test_print_rotation_is_applied(tmp_path):
    mod = fixture_module(plate_with_hole(), print=printing.settings("PETG", "on edge", rotate=[("Y", 90)]))
    entry, _ = run(tmp_path, mod)
    assert entry["print_size_mm"] == [4.0, 30.0, 40.0]


@pytest.mark.parametrize("build,changes,needle", [
    (plate_with_hole(t=0.8), {}, "thinnest wall"),
    (plate_with_hole(size=(300.0, 30.0)), {}, "does not fit"),
    (lambda: cad.box(0, 10, 0, 10, 0, 10).fuse(cad.box(20, 30, 0, 10, 0, 10)), {}, "one solid"),
    (plate_with_hole(), {"material": "PLA"}, "unknown material"),
    (plate_with_hole(), {"print": printing.settings("PETG", "on a corner", rotate=[("X", 45), ("Y", 45)])},
     "on the build plate"),
])
def test_fixture_failures_are_reported(tmp_path, build, changes, needle):
    entry, _ = run(tmp_path, fixture_module(build, **changes))
    assert not entry["validation"]["passed"]
    assert any(needle in e for e in entry["validation"]["errors"]), entry["validation"]["errors"]


def test_min_wall_finds_thin_ribs_and_knife_edges_but_not_corners(tmp_path):
    def rib():          # 1 mm rib standing on a plate
        return cad.box(0, 30, 0, 30, 0, 3).fuse(cad.box(10, 11, 0, 30, 3, 13))

    def bar(angle):     # 10 mm bar whose end is cut at this angle
        run_x = 10.0 / math.tan(math.radians(angle))
        return lambda: cad.profile_xz([(0, 0), (60, 0), (60, 10), (run_x, 10)], -10, 10)

    entry, _ = run(tmp_path, fixture_module(rib), "rib")
    assert entry["validation"]["min_wall_mm"] == pytest.approx(1.0, abs=0.05)
    # a 60 deg corner is not a thin wall ...
    entry, _ = run(tmp_path, fixture_module(bar(60)), "corner")
    assert entry["validation"]["passed"], entry["validation"]["errors"]
    # ... a 30 deg knife edge is
    entry, _ = run(tmp_path, fixture_module(bar(30)), "knife")
    assert any("thinnest wall" in e for e in entry["validation"]["errors"])


# --------------------------------------------------------------------- vendored parts

def vendored_fixture(tmp_path, monkeypatch, build, rotate=(), **part_changes):
    """A vendored part whose pinned STL is ``build()`` exported in its
    design frame under a temporary platform folder."""
    import hashlib
    monkeypatch.setattr(bpp, "PARTS", tmp_path)
    src = tmp_path / "fixture" / "upstream" / "part.stl"
    src.parent.mkdir(parents=True)
    bpp.export_stl(build(), src)
    (src.parent / "LICENSE").write_text("test")
    vendored = {"file": "upstream/part.stl", "sha256": hashlib.sha256(src.read_bytes()).hexdigest(),
                "source": "https://example.org/blob/" + "a" * 40 + "/part.stl", "licence": "MIT",
                "licence_file": "upstream/LICENSE", "author": "test"}
    mod = fixture_module(None, vendored=vendored,
                         print=printing.settings("PETG", "as published", rotate=list(rotate)), **part_changes)
    del mod.build
    return mod


def test_vendored_part_builds_and_validates(tmp_path, monkeypatch):
    mod = vendored_fixture(tmp_path, monkeypatch, plate_with_hole(), rotate=[("Y", 90)])
    entry, mesh = bpp.build_part("fixture", "fixture", mod, tmp_path / "out" / "fixture.stl", None)
    v = entry["validation"]
    assert v["passed"], v["errors"]
    assert v["valid_brep"] is None and v["voids"] == 0 and entry["step"] is None
    assert entry["print_size_mm"] == [4.0, 30.0, 40.0]          # turned to its print orientation
    assert entry["vendored"]["licence"] == "MIT"
    data = (tmp_path / "out" / "fixture.stl").read_bytes()
    assert data[:8] == b"part.stl" and b"MIT" in data[:80]      # notice in the STL header
    assert mesh.bounds[0][2] == pytest.approx(0.0, abs=1e-6)


def test_vendored_part_with_a_changed_file_fails(tmp_path, monkeypatch):
    mod = vendored_fixture(tmp_path, monkeypatch, plate_with_hole())
    (tmp_path / "fixture" / "upstream" / "part.stl").write_bytes(b"x" * 84)
    entry, _ = bpp.build_part("fixture", "fixture", mod, tmp_path / "out" / "fixture.stl", None)
    assert not entry["validation"]["passed"]
    assert "not the pinned" in entry["validation"]["errors"][0]


def test_vendored_mesh_checks(tmp_path, monkeypatch):
    def two_bodies():
        return cad.box(0, 10, 0, 10, 0, 10).fuse(cad.box(20, 30, 0, 10, 0, 10))

    mod = vendored_fixture(tmp_path, monkeypatch, two_bodies)
    entry, _ = bpp.build_part("fixture", "fixture", mod, tmp_path / "out" / "a.stl", None)
    assert any("2 bodies" in e for e in entry["validation"]["errors"])


def test_vendored_mesh_with_a_void_is_one_body(tmp_path, monkeypatch):
    def hollow():
        return cad.box(0, 30, 0, 30, 0, 30).cut(cad.box(10, 20, 10, 20, 10, 20))

    mod = vendored_fixture(tmp_path, monkeypatch, hollow)
    entry, _ = bpp.build_part("fixture", "fixture", mod, tmp_path / "out" / "a.stl", None)
    v = entry["validation"]
    assert v["passed"], v["errors"]
    assert v["bodies"] == 1 and v["voids"] == 1


def test_vendored_wall_check_uses_the_percentile(tmp_path, monkeypatch):
    def thin():               # a 3 mm plate is fine; a 0.6 mm fin on it is a real thin wall
        return cad.box(0, 40, 0, 40, 0, 3).fuse(cad.box(10, 10.6, 0, 40, 3, 23))

    mod = vendored_fixture(tmp_path, monkeypatch, thin)
    entry, _ = bpp.build_part("fixture", "fixture", mod, tmp_path / "out" / "a.stl", None)
    assert any("thinner than" in e for e in entry["validation"]["errors"]), entry["validation"]


def test_stl_comparison_detects_changes(tmp_path):
    run(tmp_path, fixture_module(plate_with_hole()), "a")
    run(tmp_path, fixture_module(plate_with_hole()), "b")
    run(tmp_path, fixture_module(plate_with_hole(t=4.5)), "c")
    assert bpp.compare_stl(tmp_path / "a.stl", tmp_path / "b.stl") == []
    assert bpp.compare_stl(tmp_path / "a.stl", tmp_path / "c.stl")


def test_plate_packing():
    def entry(stem, size, qty, material="PETG"):
        return {"id": stem, "name": stem, "quantity": qty, "material": material,
                "print": printing.settings(material, "flat"), "print_size_mm": list(size),
                "footprint_mm": list(size[:2]),
                "estimate": {"time_min": 10, "mass_g": 5.0}}
    plates = bpp.pack_plates([entry("big", (200, 200, 5), 2), entry("small", (40, 40, 5), 10),
                              entry("soft", (20, 20, 5), 4, "TPU95A")])
    petg = [p for p in plates if p["material"] == "PETG"]
    assert len(petg) == 2
    assert sum(n for p in petg for e, n in p["parts"] if e["id"] == "small") == 10
    assert [p["material"] for p in plates].count("TPU95A") == 1
    assert [p["name"] for p in plates] == ["A", "B", "C"]


def test_committed_parts_are_valid_and_up_to_date():
    """Every part builds and validates, the assembly checks pass, and the
    committed STLs and generated document blocks match the CAD sources."""
    assert bpp.main(["--check", "--no-step"]) == 0


def test_interference_check_catches_a_misplaced_hole():
    from hardware.parts.aerodrone import _hub, assembly, hub_bottom_plate
    plate = hub_bottom_plate.build()
    x, y = _hub.clamp_bolt_xy()[0]
    good = assembly._screw((x, y, 19.0), (0, 0, -1), 25.0, "M3")
    bad = assembly._screw((x + 0.8, y, 19.0), (0, 0, -1), 25.0, "M3")
    assert plate.intersect(good).Volume() < assembly.OVERLAP_TOLERANCE_MM3
    assert plate.intersect(bad).Volume() > assembly.OVERLAP_TOLERANCE_MM3


def test_camera_view_check_sees_an_obstacle():
    from hardware.parts.aerodrone import assembly
    lx, ly, lz = assembly.lens_point()
    fov = assembly._frustum((lx, ly, lz), 27.5, 34.5, 1500.0)
    below = cad.cbox(20, 20, 20, center=(lx, ly, lz - 100))
    beside = cad.cbox(20, 20, 20, center=(lx, ly + 200, lz - 100))
    assert below.intersect(fov).Volume() > 1.0
    assert beside.intersect(fov).Volume() == pytest.approx(0.0, abs=1e-6)
