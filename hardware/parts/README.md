# Printable parts

Parametric, 3D-printable parts for the CyberPhysics robots, written in
[CadQuery](https://cadquery.readthedocs.io/) (Python). Every part is built,
validated and exported by `tools/build_printable_parts.py`; the STLs are
committed so they can be downloaded straight from GitHub.

| Platform | Parts | Printing and assembly | Bill of materials |
|---|---|---|---|
| aerodrone | [`aerodrone/`](aerodrone/) | [aerodrone/README.md](aerodrone/README.md) | [../aerodrone.md](../aerodrone.md) |
| airplane | [`airplane/`](airplane/) | [airplane/README.md](airplane/README.md) | [../airplane.md](../airplane.md) |
| boat | [`boat/`](boat/) | [boat/README.md](boat/README.md) | [../boat.md](../boat.md) |
| submarine | [`submarine/`](submarine/) | [submarine/README.md](submarine/README.md) | [../submarine.md](../submarine.md) |

The other platforms (lunar_rover, racecarneo, so101, turret) follow the
same layout when their parts are added.

## Reference printer

Bambu Lab **P1S** (enclosed, 256 x 256 x 256 mm, stock 0.4 mm stainless
steel nozzle), sliced in Bambu Studio with its P1S system profiles.

- Every part must fit **250 x 250 x 250 mm** in its print orientation
  (room for the purge/wipe area and brims), or be split into pieces that
  bolt together.
- Materials (`common/printing.py`):
  - **PETG** (Bambu PETG Basic): general structure out of the sun.
  - **ASA** (Bambu ASA): anything in the sun or near heat: airframe
    exteriors, motor and ESC mounts, parts touching warm electronics. Print
    with the P1S door closed and the lid on.
  - **TPU 95A** (Bambu TPU 95A HF): dampers, grommets, feet.
  - **PETG-CF / PA-CF**: only where the load needs it. They need the
    hardened steel nozzle, so a CF part must give a non-CF
    `material_fallback`.

## Build

```bash
make -C applications build_cad            # the pinned CadQuery image, once
docker run --rm -u "$(id -u):$(id -g)" -v "$PWD":/repo -w /repo \
  salsicha/cyberphysics:cad-latest python3 tools/build_printable_parts.py
```

Run from the repository root. Variants:

| Command (after `python3 tools/build_printable_parts.py`) | Does |
|---|---|
| *(nothing)* | every platform: STLs and manifests into `hardware/parts/<platform>/stl/`, STEPs into `generated/parts/<platform>/` (git-ignored), and the generated blocks of the docs |
| `aerodrone` | one platform |
| `aerodrone --part motor_mount` | one part (no docs, no assembly check) |
| `--out DIR` | write STL/STEP/manifest to `DIR/<platform>/`; the repository is not touched |
| `--check` | rebuild in a temporary directory and fail if a committed STL or generated doc block is stale (CI runs this) |
| `--previews DIR` | also render a PNG of each part in its print orientation |
| `--figures` | regenerate the assembly figures in `hardware/parts/<platform>/images/` |
| `--list` | list the platforms and parts |

Exit status 1 if any part fails validation, the assembly check fails, or
(with `--check`) anything is stale. A source change without regenerated
STLs fails CI: rebuild and commit the STLs, manifests and docs together.

CI (`.github/workflows/ci.yml`, job `printable-parts`) installs
`applications/cad/requirements.txt`, runs the CadQuery tests and
`--check`, and uploads the STLs, STEPs and manifests as the
`printable-parts` workflow artifact. `.github/workflows/images.yml` builds
the `cad` image.

## Layout

```
hardware/
  <platform>.md                  bill of materials: purchased parts, generated printed-parts
                                 and fastener tables, assembly notes
  parts/
    README.md                    this file
    common/
      printing.py                printer, bed, materials, fits, settings() for Bambu Studio
      fasteners.py               heat-set inserts (CNC Kitchen), ISO 273 clearances, screw heads
      footprints.py              datasheet footprints of purchased parts, with sources
      schema.py                  the PART schema and its validator
      cad.py                     CadQuery helpers (imported inside build())
      render.py                  headless PNG renders (matplotlib)
    <platform>/
      README.md                  printing and assembly instructions (template below)
      _frame.py, _*.py           shared layout numbers and helpers (not parts)
      <part>.py                  one printed part per module
      assembly.py                optional: placement, interference and platform checks, figures
      stl/<part>.stl             generated, committed (binary STL)
      stl/manifest.json, .md     generated, committed
      images/*.png               generated assembly figures, committed
generated/parts/<platform>/      STEP files (git-ignored)
```

STLs and figures are in plain git, not Git LFS (`.gitattributes`
overrides the repository's LFS rule for these folders). Keep them compact:
binary STL at 0.05 mm / 0.3 rad tessellation, figures about 800 px wide
and palette-compressed. The aerodrone's 13 STLs are about 1.6 MB.

## Part modules

```python
"""Motor mount: clamps the outer end of a 16 mm arm tube ..."""
from hardware.parts.aerodrone import _frame as F
from hardware.parts.common import printing
from hardware.parts.common.footprints import Holybro2216KV920

PART = {
    "name": "Motor mount",
    "quantity": 4,
    "material": "ASA",
    "material_reason": "next to the motor, in the sun",
    "material_fallback": None,          # required (non-CF) when material is PETG-CF or PA-CF
    "flight_critical": True,
    "print": printing.settings("ASA", "upside down: motor seat on the plate",
                               wall_loops=5, infill_pct=50, rotate=[("X", 180)]),
    "min_wall_mm": 1.2,                 # optional
    "mates_with": [{"part": "Holybro 2216 KV920 motor",
                    "dimensions": "M3 holes on a 16 x 16 mm cross",
                    "source": Holybro2216KV920.SOURCE,
                    "verified": True}],
    "inserts": [{"size": "M3", "qty": 2, "where": "pinch clamp"}],
    "fasteners": [{"item": "M3 x 25 socket head cap screw", "qty": 2, "use": "pinch clamp"}],
    "fit_test": True,                   # required when any mates_with entry is unverified
    "notes": ["..."],
}

def build():
    from hardware.parts.common import cad   # CadQuery is imported here, not at module level
    ...
    return shape                            # exactly one solid, in the part's design frame
```

Rules:

- **One module per printed part**, named for the part (`motor_mount.py`);
  the STL gets the same name. Modules starting with `_` and
  `assembly.py` are not parts.
- **Import CadQuery only inside `build()`**, so `PART` can be read and
  checked without CadQuery (the plain test environment does).
- **`build()` returns one solid in the part's design frame** (the vehicle
  frame for parts that sit in one place; a local frame for parts used
  several times, which `assembly.py` places). `PART["print"]["rotate"]`
  turns it into the print orientation; the exported STL is already rotated
  and sits on z = 0, so the slicer only places it.
- **Every mating dimension comes from `common/footprints.py`** (or from
  another printed part), and each `mates_with` entry cites its source. If
  a dimension is not in the cited document, put it in the footprint's
  `UNVERIFIED`, mark the entry `"verified": False`, and the part gets
  `"fit_test": True`: it is printed first to check the fit.
- **Fasteners are listed on the part their head bears on**, so the BOM's
  generated fastener table counts each one once. Quantities are per printed
  piece. Consumables (thread locker, solder) go in the README.
- **Print settings map onto Bambu Studio**: `settings()` fills filament and
  plate type, process (layer height), wall loops, top/bottom shell layers,
  sparse infill density and pattern, supports (none, or tree/normal on the
  build plate only), brim, and orientation.
- Shared numbers (bolt positions, heights) live in the platform's
  `_frame.py` so mating parts cannot drift apart.

## Validation

For each part `tools/build_printable_parts.py` checks:

| Check | How |
|---|---|
| metadata | `common/schema.py` |
| one valid solid | the B-rep has exactly one solid and passes OCCT's `isValid()` |
| closed, manifold mesh | the exported STL is watertight, consistently wound, one body, positive volume |
| fits the bed | print-orientation size within 250 x 250 x 250 mm |
| minimum wall | ray-cast from about 12,000 surface samples along the inward normal to an opposed face; thinnest must be at least `min_wall_mm` (default 1.2 mm, three lines) |
| print orientation | at least 20 mm2 of flat face on the build plate |
| estimates | filament mass and print time from the solid's volume and face areas: walls and top/bottom shells solid, the rest at the infill density, at the material's average flow. Rough; slice for real numbers |

`assembly.py` (optional, per platform) defines `check(entries)` and
`figures(out_dir, entries)`. The aerodrone's places every printed part
with simple datasheet-sized models of the purchased parts and screws, then
checks pairwise interference (a misplaced hole shows up as a screw
overlapping a part), the camera's field of view, the rangefinder beam,
propeller clearance and the centre of gravity.

## Platform README template

Each `hardware/parts/<platform>/README.md` has these sections, in this
order (`tests/test_printable_parts_docs.py` checks the headings and that
every part appears under **Plates** and **Steps**):

1. `# <Platform> printed parts`: one paragraph on what is printed and what
   stays purchased; link the BOM.
2. `## Printing`
   - `### Plates`: the generated block
     `<!-- BEGIN GENERATED: plates (tools/build_printable_parts.py) -->` ...
     `<!-- END GENERATED: plates -->`: build plates grouped by material and
     layer height, with filament, plate type, per-object settings,
     orientation, estimated time and filament.
   - `### Material and printer notes`: drying, enclosure, nozzle.
   - `### Post-processing`: support removal, heat-set inserts (size, which
     holes, iron temperature), reaming, and the **fit-test parts to print
     first** (every part with `fit_test`).
   - `### Bambu Studio project files`: provided, or why not.
3. `## Assembly`
   - `### Tools and consumables`
   - `### Fasteners`: link to the BOM's generated fastener table.
   - `### Steps`: numbered, in build order. Each step names the parts (as
     `part_id`), fasteners, orientation cues, torque ("snug" is fine),
     thread locker, and what to check before moving on; figures from
     `images/`.
   - `### Wiring`: routing, strain relief, connectors; consistent with the
     BOM's wiring and power sections.
   - `### Checks`: the platform's pre-flight / power-up checks; link to the
     existing setup and calibration docs instead of copying them.

The BOM `hardware/<platform>.md` has the sections **Purchased parts**
(with why each safety-critical item is not printed) and **3D-printed
parts**, the latter holding the generated blocks `printed-parts` and
`fasteners`.

## Licences

The parts here are original designs under the repository's MIT licence. If
you adapt a published design, check its licence allows it and add it to
`THIRD_PARTY_NOTICES.md`.
