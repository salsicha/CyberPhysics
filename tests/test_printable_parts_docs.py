"""Printable parts: the committed outputs and documents follow the template.

Pure Python (no CadQuery): every platform with parts has a README with the
required sections, every part appears in a build plate and an assembly
step, its STL is committed (binary, plain git) and the BOM has the
generated tables. Freshness of the STLs against the CAD is checked by
tests/test_printable_parts_build.py, which needs CadQuery.
"""
import json
import re
import struct
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "tools"))

import build_printable_parts as bpp  # noqa: E402

PLATFORMS = bpp.platforms()
README_SECTIONS = [
    "## Printing", "### Plates", "### Material and printer notes", "### Post-processing",
    "### Bambu Studio project files", "## Assembly", "### Tools and consumables", "### Fasteners",
    "### Steps", "### Wiring", "### Checks",
]
STL_BUDGET_BYTES = 15_000_000


def section(text, heading):
    """The text under a heading, up to the next heading of the same or a
    higher level."""
    level = heading.split()[0]
    start = text.index(heading + "\n")
    rest = text[start + len(heading) + 1:]
    stop = re.search(rf"^#{{1,{len(level)}}} ", rest, re.MULTILINE)
    return rest[:stop.start()] if stop else rest


def generated(text, name):
    m = re.search(rf"<!-- BEGIN GENERATED: {name} .*?-->(.*?)<!-- END GENERATED: {name} -->", text, re.DOTALL)
    assert m, f"no generated block '{name}'"
    return m.group(1)


@pytest.mark.parametrize("platform", PLATFORMS)
def test_readme_has_the_template_sections_in_order(platform):
    text = (bpp.PARTS / platform / "README.md").read_text()
    assert text.startswith("# ")
    positions = [text.find(h + "\n") for h in README_SECTIONS]
    missing = [h for h, p in zip(README_SECTIONS, positions) if p < 0]
    assert not missing, f"{platform}/README.md lacks {missing}"
    assert positions == sorted(positions), "sections out of order"


@pytest.mark.parametrize("platform", PLATFORMS)
def test_every_part_is_on_a_plate_and_in_an_assembly_step(platform):
    text = (bpp.PARTS / platform / "README.md").read_text()
    plates = generated(text, "plates")
    steps = section(text, "### Steps")
    for stem in bpp.part_stems(platform):
        assert f"`{stem}`" in plates, f"{stem} is on no build plate (run tools/build_printable_parts.py)"
        assert f"`{stem}`" in steps, f"{stem} is in no assembly step"


@pytest.mark.parametrize("platform", PLATFORMS)
def test_readme_images_exist(platform):
    text = (bpp.PARTS / platform / "README.md").read_text()
    images = re.findall(r"!\[[^\]]*\]\(([^)]+)\)", text)
    assert images, "the assembly instructions need figures"
    total = 0
    for rel in images:
        path = bpp.PARTS / platform / rel
        assert path.exists(), rel
        assert path.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n", f"{rel} is not a PNG (Git LFS pointer?)"
        total += path.stat().st_size
    assert total < 2_000_000


@pytest.mark.parametrize("platform", PLATFORMS)
def test_committed_stls_and_manifest(platform):
    stl_dir = bpp.PARTS / platform / "stl"
    stems = bpp.part_stems(platform)
    manifest = json.loads((stl_dir / "manifest.json").read_text())
    assert [p["id"] for p in manifest["parts"]] == stems
    assert all(p["validation"]["passed"] for p in manifest["parts"])
    assert sorted(p.stem for p in stl_dir.glob("*.stl")) == stems
    total = 0
    for stem in stems:
        data = (stl_dir / f"{stem}.stl").read_bytes()
        assert not data.startswith(b"version https://git-lfs"), f"{stem}.stl is a Git LFS pointer"
        (count,) = struct.unpack("<I", data[80:84])
        assert len(data) == 84 + 50 * count, f"{stem}.stl is not a binary STL"
        total += len(data)
    assert total < STL_BUDGET_BYTES


def test_parts_outputs_are_not_in_git_lfs():
    lines = (REPO / ".gitattributes").read_text().splitlines()
    rules = [ln for ln in lines if ln.startswith("hardware/parts/")]
    assert "hardware/parts/*/stl/*.stl -filter binary" in rules
    assert "hardware/parts/*/images/*.png -filter binary" in rules
    # after the repository-wide LFS rules, so they win
    assert lines.index(rules[0]) > lines.index("*.stl filter=lfs diff=lfs merge=lfs -text")
    assert lines.index(rules[0]) > lines.index("*.png filter=lfs diff=lfs merge=lfs -text")


@pytest.mark.parametrize("platform", PLATFORMS)
def test_bom_has_purchased_and_printed_sections(platform):
    bom = (REPO / "hardware" / f"{platform}.md").read_text()
    assert "## Purchased parts" in bom and "## 3D-printed parts" in bom
    assert bom.index("## Purchased parts") < bom.index("## 3D-printed parts")
    table = generated(bom, "printed-parts")
    for stem in bpp.part_stems(platform):
        assert f"{stem}.stl" in table
    assert "brass heat-set insert" in generated(bom, "fasteners")
    # links from the BOM to the parts resolve
    for rel in re.findall(r"\]\((parts/[^)#]+)", bom):
        assert (REPO / "hardware" / rel).exists(), rel


def test_no_stale_links_to_the_old_aerodrone_list():
    for path in ("TODO.md", "compositions/aerodrone_hardware.yaml", "applications/demnav/README.md"):
        assert "hardware/aerodrone.txt" not in (REPO / path).read_text(), path
