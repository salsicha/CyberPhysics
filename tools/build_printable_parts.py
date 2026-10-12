#!/usr/bin/env python3
"""Build, validate and export the 3D-printable parts in hardware/parts/.

Every module hardware/parts/<platform>/<part>.py (names not starting with
"_", and not assembly.py) defines PART metadata and build(); see
hardware/parts/README.md. For each part this script

  * checks PART against hardware/parts/common/schema.py,
  * builds it with CadQuery and checks it is one valid solid,
  * turns it into its print orientation and exports a binary STL
    (0.05 mm / 0.3 rad tessellation) and a STEP,
  * checks the mesh is closed and manifold, fits the usable print volume
    (printing.BED_MM), has no wall thinner than its minimum (ray-cast from
    sampled surface points), and stands on a flat face,
  * estimates filament mass and print time from the solid's volume and area,

then writes a manifest (JSON and Markdown) per platform, updates the
generated blocks of hardware/<platform>.md (printed-parts and fastener
tables) and hardware/parts/<platform>/README.md (build plates), and runs
the platform's assembly checks (assembly.py) when there is one.

A vendored part (PART["vendored"], no build()) is a published STL committed
under hardware/parts/<platform>/upstream/ and pinned by SHA-256: the script
checks the hash, turns the mesh into its print orientation and runs the
same mesh checks (closed, one outer body, fits the bed with its brim, wall
thickness on the 1st percentile, bed contact). It has no B-rep or STEP.

  build_printable_parts.py                    every platform, into the repository
  build_printable_parts.py aerodrone          one platform
  build_printable_parts.py --out DIR          scratch output; the repository is not touched
  build_printable_parts.py --check            are the committed STLs and generated
                                              docs up to date? (writes nothing)
  build_printable_parts.py --previews DIR     also render a PNG per part
  build_printable_parts.py --figures          regenerate hardware/parts/<platform>/images/

Repository outputs: STLs and manifests in hardware/parts/<platform>/stl/
(committed), STEPs in generated/parts/<platform>/ (git-ignored). Exit
status 1 if any part fails validation or (with --check) anything is stale.
Needs CadQuery, trimesh and rtree: run it in the cad image
(applications/cad/README.md).
"""
import argparse
import hashlib
import importlib
import json
import re
import struct
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PARTS = REPO / "hardware" / "parts"
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from hardware.parts.common import fasteners, printing, schema  # noqa: E402

STL_TOLERANCE_MM = 0.05
STL_ANGULAR_TOLERANCE_RAD = 0.3
WALL_SAMPLES = 12000
WALL_TOLERANCE_MM = 0.1        # tessellation chord error allowance
MIN_BED_CONTACT_MM2 = 20.0
PLATE_MARGIN_MM = 5.0          # gap between parts on a build plate
PLATE_OVERHEAD_MIN = 6         # heating, levelling, purge per plate
LAYER_OVERHEAD_S = 2.0         # layer change, wipe

# Freshness tolerances between a committed STL and a rebuild.
FRESH_VOLUME_REL = 0.005
FRESH_BBOX_MM = 0.05
FRESH_TRIANGLES_REL = 0.02

NOT_PARTS = {"assembly"}

# Vendored meshes: a published design has features the wall check reports
# that are not walls (edge chamfers meeting a face, embossed text, slivers
# where two faces nearly touch) and that cannot be changed here. Its walls
# are checked on this percentile of the sampled thicknesses instead of the
# minimum: features thinner than the minimum wall may cover at most 1 % of
# the sampled surface.
VENDORED_WALL_PERCENTILE = 1


# --------------------------------------------------------------------- discovery

def platforms():
    """Platforms with at least one part module."""
    out = []
    for d in sorted(PARTS.iterdir()):
        if d.is_dir() and d.name != "common" and not d.name.startswith(("_", ".")) and part_stems(d.name):
            out.append(d.name)
    return out


def part_stems(platform):
    d = PARTS / platform
    return sorted(p.stem for p in d.glob("*.py")
                  if not p.stem.startswith("_") and p.stem not in NOT_PARTS)


def load_module(platform, stem):
    return importlib.import_module(f"hardware.parts.{platform}.{stem}")


def load_parts(platform, only=None):
    mods = {}
    for stem in part_stems(platform):
        if only and stem not in only:
            continue
        mods[stem] = load_module(platform, stem)
    return mods


# --------------------------------------------------------------------- geometry

def print_oriented(shape, rotations):
    """Rotate into the print orientation, then sit on z = 0 centred in x/y."""
    from hardware.parts.common import cad
    for axis, deg in rotations:
        shape = cad.rotate(shape, axis, deg)
    bb = shape.BoundingBox()
    return cad.translate(shape, (-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))


def export_stl(shape, path):
    import cadquery as cq
    path.parent.mkdir(parents=True, exist_ok=True)
    cq.exporters.export(shape, str(path), tolerance=STL_TOLERANCE_MM,
                        angularTolerance=STL_ANGULAR_TOLERANCE_RAD)


def export_step(shape, path):
    import cadquery as cq
    path.parent.mkdir(parents=True, exist_ok=True)
    cq.exporters.export(shape, str(path))


def load_mesh(path):
    import trimesh
    return trimesh.load(str(path), force="mesh", process=True)


def wall_samples(mesh, samples=WALL_SAMPLES, seed=7):
    """Wall thickness at sampled surface points: the distance along the
    inward normal to the opposite surface. Only hits on a roughly opposed
    face count (normals more than ~135 deg apart), so the tips of acute
    corners, which are thin by nature, are not reported as thin walls.
    Returns (points, thickness) arrays."""
    import numpy as np
    import trimesh
    pts, fi = trimesh.sample.sample_surface(mesh, samples, seed=seed)
    pts = np.vstack([pts, mesh.triangles_center])
    fi = np.concatenate([fi, np.arange(len(mesh.faces))])
    n = mesh.face_normals[fi]
    origins = pts - n * 1e-3
    loc, ray_i, tri = mesh.ray.intersects_location(origins, -n, multiple_hits=False)
    if len(ray_i) == 0:
        return np.zeros((0, 3)), np.zeros(0)
    d = np.linalg.norm(loc - origins[ray_i], axis=1) + 1e-3
    opposed = np.einsum("ij,ij->i", n[ray_i], mesh.face_normals[tri]) < -0.7
    keep = opposed & (d > 0.02)
    return origins[ray_i][keep], d[keep]


def min_wall(mesh, samples=WALL_SAMPLES, seed=7):
    """Thinnest wall and its 1st percentile, in mm (see wall_samples)."""
    import numpy as np
    _, d = wall_samples(mesh, samples, seed)
    if len(d) == 0:
        return float("inf"), float("inf")
    return float(d.min()), float(np.percentile(d, 1))


def bed_contact_area(mesh):
    z = mesh.triangles[:, :, 2]
    on_bed = (z.max(axis=1) < 0.01) & (mesh.face_normals[:, 2] < -0.99)
    return float(mesh.area_faces[on_bed].sum())


# --------------------------------------------------------------------- vendored meshes

def is_vendored(part):
    return isinstance(part, dict) and "vendored" in part


def vendored_path(platform, part):
    """The pinned file of a vendored part (relative to its platform folder)."""
    return PARTS / platform / part["vendored"]["file"]


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def mesh_print_oriented(mesh, rotations):
    """Mesh counterpart of print_oriented(): rotate about the global axes in
    order (right-handed, degrees), then sit on z = 0 centred in x/y."""
    import numpy as np
    import trimesh
    m = mesh.copy()
    axes = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}
    for axis, deg in rotations:
        m.apply_transform(trimesh.transformations.rotation_matrix(np.radians(deg), axes[axis]))
    lo, hi = m.bounds
    m.apply_translation((-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]))
    return m


def write_binary_stl(mesh, path, header=""):
    """Binary STL with an ASCII header (at most 80 characters)."""
    import numpy as np
    path.parent.mkdir(parents=True, exist_ok=True)
    rec = np.zeros(len(mesh.faces), dtype=[("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    rec["n"] = mesh.face_normals
    rec["v"] = mesh.triangles
    head = header.encode("ascii", "replace")[:80].ljust(80, b" ")
    with open(path, "wb") as f:
        f.write(head)
        f.write(struct.pack("<I", len(rec)))
        f.write(rec.tobytes())


def mesh_face_areas(mesh):
    """Mesh area split like face_areas() for a mesh already in its print
    orientation: faces facing up or down (shells) and the rest (walls)."""
    import numpy as np
    flat = np.abs(mesh.face_normals[:, 2]) > 0.7
    return float(mesh.area_faces[flat].sum()), float(mesh.area_faces[~flat].sum())


def mesh_shells(mesh):
    """(outer bodies, enclosed voids) of a closed mesh: its connected shells
    with positive and with negative volume."""
    shells = mesh.split(only_watertight=False)
    outer = sum(1 for s in shells if s.volume > 0)
    return outer, len(shells) - outer


def build_vendored_part(platform, stem, module, stl_path):
    """Check, orient and validate one vendored part. Returns its manifest
    entry and mesh, like build_part()."""
    import numpy as np
    part = module.PART
    errors = schema.validate_part(part, stem)
    t0 = time.time()
    result = {"id": stem, "name": part.get("name", stem) if isinstance(part, dict) else stem}
    if errors:
        result["validation"] = {"passed": False, "errors": errors}
        return result, None
    v = part["vendored"]
    src = vendored_path(platform, part)
    if not src.exists():
        result["validation"] = {"passed": False, "errors": [f"{stem}: vendored file {v['file']} is missing"]}
        return result, None
    digest = sha256_file(src)
    if digest != v["sha256"]:
        result["validation"] = {"passed": False, "errors": [
            f"{stem}: {v['file']} has SHA-256 {digest}, not the pinned {v['sha256']}"]}
        return result, None
    oriented = mesh_print_oriented(load_mesh(src), part["print"]["rotate"])
    write_binary_stl(oriented, stl_path,
                     f"{v['file'].rsplit('/', 1)[-1]} ({v['licence']}), turned to print orientation")
    mesh = load_mesh(stl_path)

    ext = [float(e) for e in mesh.extents]
    watertight = bool(mesh.is_watertight and mesh.is_winding_consistent and mesh.is_volume)
    if not watertight:
        errors.append(f"{stem}: mesh is not closed and manifold")
    bodies, voids = mesh_shells(mesh) if watertight else (int(mesh.body_count), 0)
    if bodies != 1:
        errors.append(f"{stem}: mesh has {bodies} bodies")
    brim = brim_width(part["print"]["brim"])
    footprint = [ext[0] + 2 * brim, ext[1] + 2 * brim, ext[2]]
    fits = all(e <= b for e, b in zip(footprint, printing.BED_MM))
    if not fits:
        errors.append(f"{stem}: {footprint[0]:.1f} x {footprint[1]:.1f} x {ext[2]:.1f} mm (with brim) does not fit "
                      f"{printing.BED_MM} mm")
    required = part.get("min_wall_mm", printing.DEFAULT_MIN_WALL_MM)
    _, d = wall_samples(mesh)
    wall_min = float(d.min()) if len(d) else float("inf")
    wall_p1 = float(np.percentile(d, VENDORED_WALL_PERCENTILE)) if len(d) else float("inf")
    thin_pct = float((d < required).mean() * 100.0) if len(d) else 0.0
    if wall_p1 + WALL_TOLERANCE_MM < required:
        errors.append(f"{stem}: {thin_pct:.1f} % of the sampled walls are thinner than {required} mm "
                      f"(1st percentile {wall_p1:.2f} mm)")
    contact = bed_contact_area(mesh)
    if contact < MIN_BED_CONTACT_MM2:
        errors.append(f"{stem}: only {contact:.0f} mm2 on the build plate; check the print orientation")

    volume, area = float(mesh.volume), float(mesh.area)
    mass, minutes = estimate(volume, mesh_face_areas(mesh), ext[2], part)
    qty = part["quantity"]
    result.update({
        "quantity": qty,
        "material": part["material"],
        "material_fallback": part.get("material_fallback"),
        "material_reason": part["material_reason"],
        "flight_critical": part["flight_critical"],
        "fit_test": part["fit_test"],
        "print": part["print"],
        "stl": None,
        "step": None,
        "vendored": {key: v[key] for key in sorted(v)},
        "volume_cm3": round(volume / 1000.0, 2),
        "area_cm2": round(area / 100.0, 1),
        "print_size_mm": [round(e, 1) for e in ext],
        "footprint_mm": [round(e, 1) for e in footprint[:2]],
        "triangles": len(mesh.faces),
        "estimate": {
            "mass_g": round(mass, 1),
            "time_min": round(minutes),
            "mass_g_all": round(mass * qty, 1),
            "time_min_all": round(minutes * qty),
        },
        "validation": {
            "passed": not errors,
            "valid_brep": None,
            "closed_manifold": watertight,
            "bodies": bodies,
            "voids": voids,
            "fits_bed": fits,
            "min_wall_mm": round(wall_min, 2),
            "min_wall_p1_mm": round(wall_p1, 2),
            "min_wall_required_mm": required,
            "min_wall_rule": f"percentile {VENDORED_WALL_PERCENTILE} (vendored mesh)",
            "thinner_than_required_pct": round(thin_pct, 2),
            "bed_contact_mm2": round(contact),
            "errors": errors,
        },
        "mates_with": part["mates_with"],
        "inserts": part["inserts"],
        "fasteners": part["fasteners"],
        "notes": part["notes"],
    })
    result["_build_s"] = round(time.time() - t0, 1)
    return result, mesh


def face_areas(solid, up=(0.0, 0.0, 1.0)):
    """B-rep area split into faces facing up/down in the print orientation
    (top and bottom shells) and the rest (walls), in mm2."""
    flat = side = 0.0
    for f in solid.Faces():
        a = f.Area()
        try:
            n = f.normalAt()
            nz = abs(n.x * up[0] + n.y * up[1] + n.z * up[2])
        except Exception:  # noqa: BLE001 - degenerate face: count it as wall
            nz = 0.0
        if nz > 0.7:
            flat += a
        else:
            side += a
    return flat, side


def print_up(rotations):
    """The build-frame direction that ends up as +z when printing."""
    import numpy as np
    m = np.eye(3)
    for axis, deg in rotations:
        a = np.radians(deg)
        c, s = np.cos(a), np.sin(a)
        r = {"X": [[1, 0, 0], [0, c, -s], [0, s, c]],
             "Y": [[c, 0, s], [0, 1, 0], [-s, 0, c]],
             "Z": [[c, -s, 0], [s, c, 0], [0, 0, 1]]}[axis]
        m = np.array(r) @ m
    return tuple(float(v) for v in m.T @ np.array([0.0, 0.0, 1.0]))


def brim_width(brim):
    """Brim width (mm) a brim setting adds around the part."""
    m = re.search(r"(\d+(?:\.\d+)?) mm", brim)
    return float(m.group(1)) if m else 0.0


def estimate(volume_mm3, areas_mm2, height_mm, part):
    """Filament mass (g) and print time (min) of one piece: a rough model,
    shells (walls on side faces, top/bottom layers on flat faces) plus
    sparse infill, at the material's average flow."""
    p = part["print"]
    m = printing.MATERIALS[part["material"]]
    flat, side = areas_mm2
    wall_t = p["wall_loops"] * printing.LINE_WIDTH_MM
    cap_t = 0.5 * (p["top_shell_layers"] + p["bottom_shell_layers"]) * p["layer_height_mm"]
    shell = min(volume_mm3, side * wall_t + flat * cap_t)
    printed = shell + (volume_mm3 - shell) * p["sparse_infill_density_pct"] / 100.0
    mass = printed / 1000.0 * m["density_g_cm3"]
    layers = height_mm / p["layer_height_mm"]
    seconds = printed / m["flow_mm3_s"] + layers * LAYER_OVERHEAD_S
    return mass, seconds / 60.0


# --------------------------------------------------------------------- one part

def build_part(platform, stem, module, stl_path, step_path):
    """Build and validate one part. Returns its manifest entry and mesh."""
    if is_vendored(getattr(module, "PART", None)):
        return build_vendored_part(platform, stem, module, stl_path)
    from hardware.parts.common import cad
    part = module.PART
    errors = schema.validate_part(part, stem)
    t0 = time.time()
    result = {"id": stem, "name": part.get("name", stem) if isinstance(part, dict) else stem}
    if errors:
        # The geometry checks need valid metadata (material, print settings).
        result["validation"] = {"passed": False, "errors": errors}
        return result, None
    try:
        shape = module.build()
        solid = cad.single_solid(shape)
    except Exception as exc:  # noqa: BLE001 - report any build failure
        result["validation"] = {"passed": False, "errors": [f"{stem}: build failed: {exc}"]}
        return result, None
    if not solid.isValid():
        errors.append(f"{stem}: B-rep is not a valid solid")
    volume = solid.Volume()
    area = solid.Area()
    areas = face_areas(solid, print_up(part["print"]["rotate"]))
    if volume <= 0:
        errors.append(f"{stem}: non-positive volume")

    oriented = print_oriented(solid, part["print"]["rotate"])
    export_stl(oriented, stl_path)
    if step_path is not None:
        export_step(solid, step_path)
    mesh = load_mesh(stl_path)

    ext = [float(v) for v in mesh.extents]
    watertight = bool(mesh.is_watertight and mesh.is_winding_consistent and mesh.is_volume)
    bodies = int(mesh.body_count)
    if not watertight:
        errors.append(f"{stem}: mesh is not closed and manifold")
    if bodies != 1:
        errors.append(f"{stem}: mesh has {bodies} bodies")
    brim = brim_width(part["print"]["brim"])
    footprint = [ext[0] + 2 * brim, ext[1] + 2 * brim, ext[2]]
    fits = all(e <= b for e, b in zip(footprint, printing.BED_MM))
    if not fits:
        errors.append(f"{stem}: {footprint[0]:.1f} x {footprint[1]:.1f} x {ext[2]:.1f} mm (with brim) does not fit "
                      f"{printing.BED_MM} mm; split it into joinable pieces")
    required = part.get("min_wall_mm", printing.DEFAULT_MIN_WALL_MM)
    wall_min, wall_p1 = min_wall(mesh)
    if wall_min + WALL_TOLERANCE_MM < required:
        errors.append(f"{stem}: thinnest wall {wall_min:.2f} mm < {required} mm")
    contact = bed_contact_area(mesh)
    if contact < MIN_BED_CONTACT_MM2:
        errors.append(f"{stem}: only {contact:.0f} mm2 on the build plate; check the print orientation")

    mass, minutes = estimate(volume, areas, ext[2], part)
    qty = part["quantity"]
    result.update({
        "quantity": qty,
        "material": part["material"],
        "material_fallback": part.get("material_fallback"),
        "material_reason": part["material_reason"],
        "flight_critical": part["flight_critical"],
        "fit_test": part["fit_test"],
        "print": part["print"],
        "stl": None,   # filled by the caller (relative path)
        "step": None,
        "volume_cm3": round(volume / 1000.0, 2),
        "area_cm2": round(area / 100.0, 1),
        "print_size_mm": [round(e, 1) for e in ext],
        "footprint_mm": [round(e, 1) for e in footprint[:2]],
        "triangles": len(mesh.faces),
        "estimate": {
            "mass_g": round(mass, 1),
            "time_min": round(minutes),
            "mass_g_all": round(mass * qty, 1),
            "time_min_all": round(minutes * qty),
        },
        "validation": {
            "passed": not errors,
            "valid_brep": bool(solid.isValid()),
            "closed_manifold": watertight,
            "bodies": bodies,
            "fits_bed": fits,
            "min_wall_mm": round(wall_min, 2),
            "min_wall_p1_mm": round(wall_p1, 2),
            "min_wall_required_mm": required,
            "bed_contact_mm2": round(contact),
            "errors": errors,
        },
        "mates_with": part["mates_with"],
        "inserts": part["inserts"],
        "fasteners": part["fasteners"],
        "notes": part["notes"],
    })
    result["_build_s"] = round(time.time() - t0, 1)   # console only, not in the manifest
    return result, mesh


# --------------------------------------------------------------------- plates

def pack_plates(entries):
    """Group pieces into build plates by material and layer height, then
    shelf-pack their print footprints (either way round) on the usable bed.
    Bounding boxes only: Bambu Studio's arrange nests parts tighter."""
    groups = {}
    for e in entries:
        if "print_size_mm" not in e:
            continue
        key = (e["material"], e["print"]["layer_height_mm"])
        for _ in range(e["quantity"]):
            groups.setdefault(key, []).append(e)
    W, H = printing.BED_MM[0], printing.BED_MM[1]
    gap = PLATE_MARGIN_MM

    def place(plate, w, h):
        for shelf in plate["shelves"]:
            for a, b in ((w, h), (h, w)):
                if shelf["x"] + a <= W and b <= shelf["h"]:
                    shelf["x"] += a + gap
                    return True
        for a, b in sorted(((w, h), (h, w)), key=lambda wh: wh[1]):
            if a <= W and plate["y"] + b <= H:
                plate["shelves"].append({"x": a + gap, "h": b})
                plate["y"] += b + gap
                return True
        return False

    plates = []
    for (material, layer), items in sorted(groups.items()):
        items = sorted(items, key=lambda e: (-e["footprint_mm"][0] * e["footprint_mm"][1], e["id"]))
        group_plates = []
        for e in items:
            w, h = e["footprint_mm"]
            for plate in group_plates:
                if place(plate, w, h):
                    plate["items"].append(e)
                    break
            else:
                plate = {"shelves": [], "y": 0.0, "items": [e]}
                if not place(plate, w, h):
                    raise ValueError(f"{e['id']} does not fit the bed")
                group_plates.append(plate)
        for plate in group_plates:
            counts = {}
            for e in plate["items"]:
                counts.setdefault(e["id"], [e, 0])[1] += 1
            plates.append({
                "material": material,
                "layer_height_mm": layer,
                "parts": [(e, n) for e, n in counts.values()],
                "time_min": round(sum(e["estimate"]["time_min"] * n for e, n in counts.values())
                                  + PLATE_OVERHEAD_MIN),
                "mass_g": round(sum(e["estimate"]["mass_g"] * n for e, n in counts.values())),
            })
    for i, p in enumerate(plates):
        p["name"] = chr(ord("A") + i) if i < 26 else f"P{i + 1}"
    return plates


def fmt_mass(grams):
    return f"{grams:.1f} g" if grams < 10 else f"{grams:.0f} g"


def fmt_time(minutes):
    h, m = divmod(round(minutes), 60)
    return f"{h} h {m:02d} min" if h else f"{m} min"


# --------------------------------------------------------------------- documents

def stl_rel(platform, stem):
    return f"parts/{platform}/stl/{stem}.stl"


def printed_parts_table(platform, entries):
    rows = ["| Part | Qty | Material | Bambu Studio settings (P1S, 0.4 mm) | Est. time (each) | Est. mass (each) | File |",
            "|---|---|---|---|---|---|---|"]
    total_t = total_m = 0.0
    for e in entries:
        if "estimate" not in e:
            continue
        mat = e["material"]
        if e.get("material_fallback"):
            mat += f" (or {e['material_fallback']})"
        flags = []
        if e["flight_critical"]:
            flags.append("flight-critical")
        if e["fit_test"]:
            flags.append("fit test first")
        name = e["name"] + (f" ({', '.join(flags)})" if flags else "")
        rows.append(f"| {name} | {e['quantity']} | {mat} | {printing.settings_summary(e['print'])} | "
                    f"{fmt_time(e['estimate']['time_min'])} | {fmt_mass(e['estimate']['mass_g'])} | "
                    f"[{e['id']}.stl]({stl_rel(platform, e['id'])}) |")
        total_t += e["estimate"]["time_min_all"]
        total_m += e["estimate"]["mass_g_all"]
    rows.append(f"| **Total, all pieces** | | | | **{fmt_time(total_t)}** | **{total_m:.0f} g** | |")
    return "\n".join(rows)


def fastener_table(entries):
    """Every fastener and insert, totalled over the vehicle."""
    items = {}
    for e in entries:
        if "estimate" not in e:
            continue
        q = e["quantity"]
        for f in e["fasteners"]:
            row = items.setdefault(f["item"], {"qty": 0, "uses": []})
            row["qty"] += f["qty"] * q
            row["uses"].append(f"{f['use']} ({e['id']}, {f['qty'] * q})")
        for i in e["inserts"]:
            ins = fasteners.INSERTS[i["size"]]
            label = f"{ins.thread} x {ins.length:g} mm brass heat-set insert (CNC Kitchen type, {ins.hole:g} mm hole)"
            row = items.setdefault(label, {"qty": 0, "uses": []})
            row["qty"] += i["qty"] * q
            row["uses"].append(f"{i['where']} ({e['id']}, {i['qty'] * q})")
    rows = ["| Item | Qty | Where |", "|---|---|---|"]
    for item in sorted(items):
        rows.append(f"| {item} | {items[item]['qty']} | {'; '.join(items[item]['uses'])} |")
    return "\n".join(rows)


def plates_section(platform, plates):
    out = []
    for p in plates:
        m = printing.MATERIALS[p["material"]]
        out.append(f"#### Plate {p['name']}: {p['material']}, {p['layer_height_mm']:.2f} mm layers")
        out.append("")
        out.append(f"Filament **{m['filament_profile']}**, **{m['plate']}**, process "
                   f"**{p['layer_height_mm']:.2f}mm Standard**. Estimated "
                   f"{fmt_time(p['time_min'])} and {p['mass_g']} g of filament (estimates; "
                   "slice for real numbers).")
        out.append("")
        out.append("| Part | Qty | Per-object settings | Orientation | File |")
        out.append("|---|---|---|---|---|")
        for e, n in p["parts"]:
            pr = e["print"]
            s = (f"{pr['wall_loops']} wall loops, {pr['top_shell_layers']}/{pr['bottom_shell_layers']} top/bottom "
                 f"shells, {pr['sparse_infill_density_pct']}% {pr['sparse_infill_pattern']}, supports: "
                 f"{pr['supports']}, brim: {pr['brim']}")
            out.append(f"| `{e['id']}` | {n} | {s} | {pr['orientation']} | [stl](stl/{e['id']}.stl) |")
        out.append("")
    return "\n".join(out).rstrip() + "\n"


BEGIN = "<!-- BEGIN GENERATED: {name} (tools/build_printable_parts.py) -->"
END = "<!-- END GENERATED: {name} -->"


def replace_block(text, name, body):
    """Replace a generated block; raises if the markers are missing."""
    b, e = BEGIN.format(name=name), END.format(name=name)
    pattern = re.compile(re.escape(b) + r".*?" + re.escape(e), re.DOTALL)
    if not pattern.search(text):
        raise KeyError(f"missing generated block markers for '{name}'")
    return pattern.sub(lambda _: f"{b}\n{body.rstrip()}\n{e}", text)


def assembly_block(report):
    """The assembly check results as a Markdown list (platforms may add
    their own keys; the known ones get readable labels)."""
    if not report:
        return "No assembly check for this platform."
    labels = {
        "interferences": "Interference between parts (screws modelled through their holes)",
        "intended_press_fits": "Intended press fits",
        "camera_view_clear": "Camera view clear",
        "rangefinder_beam_clear": "Rangefinder beam clear",
        "propeller_clearance": "Propeller discs clear",
        "battery_centre_x_mm": "Battery centre for a centred CG (mm along x; negative = behind the hub centre)",
        "battery_on_tray": "Battery within the tray",
        "cg_mm": "Centre of gravity with the battery there (x, y, z mm from the hub centre)",
        "takeoff_mass_g": "Estimated take-off mass (g)",
        "bodies": "Bodies in the check",
        # so101 station
        "pitch_sweep": "Camera cradle swept through its pitch range against the head",
        "scene_camera_is_the_d435i_colour_stream": "Scene camera models the D435i's colour stream",
        "task_area_in_view_at_scene_pitch": "Bins and objects in the D435i's image at the scene's pitch",
        "scene_pitch_is_the_recommended_pitch": "Scene camera pitched at the recommended pitch",
        "recommended_pitch_deg": "Recommended camera pitch (deg down)",
        "task_area_in_view_at_recommended_pitch": "Bins and objects in the image at the recommended pitch",
        "camera_view_clear_of_station": "Camera's view clear of the station's own parts",
        "out_of_arm_reach": "Camera, head and post out of the arm's reach",
        "on_the_station_board": "Everything on the station board",
        "camera_rgb_at_recommended_pitch_mm": "RGB optical centre at the recommended pitch (x, y, z mm from base_link)",
        # lunar rover
        "wheel_radius_mm": "Wheel radius at the grouser tips (mm)",
        "wheel_gaps_mm": "Gaps between neighbouring wheels (mm)",
        "ground_clearance_mm": "Ground clearance under the body (mm, lowest part)",
        "mass_kg": "Estimated mass (kg)",
        "centre_of_gravity_mm": "Centre of gravity (x, y, z mm; z above the ground)",
        "tip_over_deg": "Static tip-over angles (deg)",
        "wheel_torque_nm": "Wheel torque (N m per wheel)",
        "max_payload_at_rated_torque_kg": "Largest payload on the steepest slope at rated torque (kg)",
        "wheel_speed_rpm": "Wheel speed",
        "battery_wh": "Battery energy (Wh)",
        "hazcam_view_clear": "Hazard camera's depth view clear of the rover",
        "terrain_camera_view_clear": "Terrain camera's view clear of the rover",
        "lidar_view_clear": "Lidar's field of view clear of the rover",
        "lux_sky_view": "Lux sensor's view of the sky",
        "passed": "All assembly checks passed",
    }
    lines = ["Generated by the assembly check (`assembly.py`), from the CAD and datasheet masses:", ""]
    for key, value in report.items():
        if isinstance(value, list):
            value = "; ".join(str(v) for v in value)
        elif isinstance(value, bool):
            value = "yes" if value else "**no**"
        lines.append(f"- {labels.get(key, key)}: {value}")
    return "\n".join(lines)


def generated_docs(platform, entries, plates, assembly=None):
    """{path: new text} for the documents with generated blocks."""
    out = {}
    bom = REPO / "hardware" / f"{platform}.md"
    if bom.exists():
        text = bom.read_text()
        for name, body in (("printed-parts", printed_parts_table(platform, entries)),
                           ("fasteners", fastener_table(entries))):
            if BEGIN.format(name=name) in text:
                text = replace_block(text, name, body)
        out[bom] = text
    readme = PARTS / platform / "README.md"
    if readme.exists():
        text = readme.read_text()
        if BEGIN.format(name="plates") in text:
            text = replace_block(text, "plates", plates_section(platform, plates))
        if BEGIN.format(name="assembly") in text and assembly is not None:
            text = replace_block(text, "assembly", assembly_block(assembly))
        out[readme] = text
    return out


def manifest_markdown(platform, entries, plates, assembly):
    lines = [f"# {platform} printed parts manifest", "",
             ("Generated by `tools/build_printable_parts.py`; do not edit. Estimates are rough "
              "(shells plus sparse infill at an average flow); slice for real numbers."), "",
             "| Part | Qty | Material | Print size (mm) | Volume | Est. mass | Est. time | Min wall | Valid | STL |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for e in entries:
        v = e.get("validation", {})
        if "estimate" not in e:
            lines.append(f"| {e['name']} | | | | | | | | FAILED | |")
            continue
        size = " x ".join(f"{s:g}" for s in e["print_size_mm"])
        lines.append(f"| {e['name']} | {e['quantity']} | {e['material']} | {size} | {e['volume_cm3']} cm3 | "
                     f"{e['estimate']['mass_g']} g | {fmt_time(e['estimate']['time_min'])} | "
                     f"{v['min_wall_mm']} mm | {'yes' if v['passed'] else 'NO'} | [stl]({e['id']}.stl) |")
    lines += ["", f"{len(plates)} build plates; see ../README.md.", ""]
    if assembly:
        lines.append("## Assembly check")
        lines.append("")
        for k, v in assembly.items():
            lines.append(f"- {k}: {v}")
        lines.append("")
    return "\n".join(lines)


# --------------------------------------------------------------------- freshness

def compare_stl(committed, rebuilt):
    """Problems between a committed STL and a fresh build (empty if fresh)."""
    import numpy as np
    problems = []
    a, b = load_mesh(committed), load_mesh(rebuilt)
    if abs(a.volume - b.volume) > FRESH_VOLUME_REL * abs(b.volume):
        problems.append(f"volume {a.volume:.0f} vs rebuilt {b.volume:.0f} mm3")
    if np.abs(a.bounds - b.bounds).max() > FRESH_BBOX_MM:
        problems.append(f"bounding box {a.extents.round(2).tolist()} vs rebuilt {b.extents.round(2).tolist()}")
    if abs(len(a.faces) - len(b.faces)) > FRESH_TRIANGLES_REL * len(b.faces):
        problems.append(f"{len(a.faces)} triangles vs rebuilt {len(b.faces)}")
    # Equal bounds, volume and triangle count do not detect a moved hole.
    # Compare both surfaces, without relying on STL vertex/face ordering.
    def triangles(mesh):
        points = np.round(mesh.triangles, 5)
        points = np.sort(points.view([('x', 'f8'), ('y', 'f8'), ('z', 'f8')]), axis=1)
        data = points.tobytes()
        return sorted(data[i:i + 72] for i in range(0, len(data), 72))

    if triangles(a) != triangles(b):
        import trimesh
        for source, target in ((a, b), (b, a)):
            points = np.vstack((source.vertices, source.triangles_center))
            maximum = 0.0
            for offset in range(0, len(points), 512):
                _, distances, _ = trimesh.proximity.closest_point(target, points[offset:offset + 512])
                maximum = max(maximum, float(distances.max()))
                if maximum > FRESH_BBOX_MM:
                    break
            if maximum > FRESH_BBOX_MM:
                problems.append(f"surface differs by at least {maximum:.3f} mm")
                break
    return problems


# --------------------------------------------------------------------- main

def run_platform(platform, args, report):
    # Clean up also on partial checks, validation failure and exceptions.
    with tempfile.TemporaryDirectory(prefix=f"parts-{platform}-") as work:
        return _run_platform(platform, args, report, Path(work))


def _run_platform(platform, args, report, work):
    only = set(args.part) if args.part else None
    mods = load_parts(platform, only)
    if not mods:
        return True
    if args.check:
        stl_dir, step_dir = work / "stl", None
    elif args.out:
        stl_dir, step_dir = Path(args.out) / platform / "stl", Path(args.out) / platform / "step"
    else:
        stl_dir, step_dir = PARTS / platform / "stl", REPO / "generated" / "parts" / platform
    if args.no_step:
        step_dir = None
    elif step_dir is not None:
        try:
            step_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            print(f"  warning: cannot write STEP files to {step_dir} ({exc.strerror}); skipping them "
                  "(use --out DIR, or make the directory writable)")
            step_dir = None

    entries, meshes, ok = [], {}, True
    for stem, mod in mods.items():
        stl_path = stl_dir / f"{stem}.stl"
        vendored = is_vendored(getattr(mod, "PART", None))
        step_path = step_dir / f"{stem}.step" if step_dir and not vendored else None
        entry, mesh = build_part(platform, stem, mod, stl_path, step_path)
        entry["stl"] = str((PARTS / platform / "stl" / f"{stem}.stl").relative_to(REPO))
        entry["step"] = None if vendored else str((REPO / "generated" / "parts" / platform / f"{stem}.step").relative_to(REPO))
        if args.out:
            # Downloaded artifacts must resolve without a repository checkout.
            # Paths in this manifest are relative to its own stl/ directory.
            entry["stl"] = stl_path.name
            entry["step"] = f"../step/{step_path.name}" if step_path is not None else None
        entries.append(entry)
        meshes[stem] = mesh
        v = entry["validation"]
        status = "ok" if v["passed"] else "FAILED"
        if v["passed"]:
            print(f"  {platform}/{stem}: {status}  {entry['print_size_mm']} mm, "
                  f"{entry['estimate']['mass_g']} g, {fmt_time(entry['estimate']['time_min'])}, "
                  f"min wall {v['min_wall_mm']} mm, {entry['triangles']} tris ({entry.pop('_build_s')} s)")
        else:
            ok = False
            entry.pop("_build_s", None)
            print(f"  {platform}/{stem}: {status}")
            for e in v["errors"]:
                print(f"    - {e}")
        if args.previews and mesh is not None:
            from hardware.parts.common import render
            Path(args.previews).mkdir(parents=True, exist_ok=True)
            render.render([(mesh, render.MATERIAL_COLORS[entry["material"]])],
                          Path(args.previews) / f"{platform}_{stem}.png",
                          title=f"{entry['name']} (print orientation)", width_px=640, height_px=480)

    if any(mesh is None for mesh in meshes.values()):
        report[platform] = False
        return False  # incomplete entries cannot be packed or assembled

    stale = []
    if args.check:
        for stem in mods:
            committed = PARTS / platform / "stl" / f"{stem}.stl"
            if not committed.exists():
                stale.append(f"{committed.relative_to(REPO)} is missing")
                continue
            for problem in compare_stl(committed, stl_dir / f"{stem}.stl"):
                stale.append(f"{committed.relative_to(REPO)}: {problem}")
    if only:
        for problem in stale:
            print(f"    - {problem}")
        report[platform] = ok and not stale
        return report[platform]

    plates = pack_plates(entries)
    assembly_report = {}
    asm_path = PARTS / platform / "assembly.py"
    if asm_path.exists() and not args.no_assembly and not only:
        asm = importlib.import_module(f"hardware.parts.{platform}.assembly")
        print(f"  {platform}: assembly checks ...")
        assembly_report = asm.check({e["id"]: e for e in entries})
        for k, v in assembly_report.items():
            print(f"    {k}: {v}")
        if not assembly_report.get("passed", True):
            ok = False
        if args.figures:
            images = Path(args.out) / platform / "images" if args.out else PARTS / platform / "images"
            asm.figures(images, {e["id"]: e for e in entries})
            print(f"  {platform}: figures written to {images}")

    manifest = {
        "platform": platform,
        "generator": "tools/build_printable_parts.py",
        "printer": printing.PRINTER,
        "usable_volume_mm": printing.BED_MM,
        "stl_tolerance_mm": STL_TOLERANCE_MM,
        "stl_angular_tolerance_rad": STL_ANGULAR_TOLERANCE_RAD,
        "parts": entries,
        "plates": [{"name": p["name"], "material": p["material"], "layer_height_mm": p["layer_height_mm"],
                    "parts": [{"id": e["id"], "qty": n} for e, n in p["parts"]],
                    "time_min": p["time_min"], "mass_g": p["mass_g"]} for p in plates],
        "assembly": assembly_report,
    }
    if args.out:
        manifest['path_base'] = 'manifest_directory'
    docs = generated_docs(platform, entries, plates, assembly_report if asm_path.exists() and not args.no_assembly else None)

    if args.check:
        committed_dir = PARTS / platform / "stl"
        for extra in sorted({p.stem for p in committed_dir.glob("*.stl")} - set(mods)):
            stale.append(f"{(committed_dir / extra).relative_to(REPO)}.stl has no source module")
        pinned = {vendored_path(platform, m.PART).resolve() for m in mods.values() if is_vendored(m.PART)}
        for extra in sorted((PARTS / platform / "upstream").glob("*.stl")):
            if extra.resolve() not in pinned:
                stale.append(f"{extra.relative_to(REPO)} is vendored but no part uses it")
        for path, text in docs.items():
            if path.read_text() != text:
                stale.append(f"{path.relative_to(REPO)}: generated blocks are out of date")
        if not args.no_assembly:
            manifest_dir = PARTS / platform / "stl"
            manifest_path = manifest_dir / "manifest.json"
            try:
                fresh_manifest = json.loads(manifest_path.read_text()) == json.loads(json.dumps(manifest))
            except (OSError, ValueError):
                fresh_manifest = False
            if not fresh_manifest:
                stale.append(f"{manifest_path.relative_to(REPO)}: manifest is missing or out of date")
            markdown = manifest_dir / "manifest.md"
            if not markdown.exists() or markdown.read_text() != manifest_markdown(platform, entries, plates, assembly_report):
                stale.append(f"{markdown.relative_to(REPO)}: manifest is missing or out of date")
        if stale:
            ok = False
            print(f"  {platform}: STALE, run tools/build_printable_parts.py {platform}")
            for s in stale:
                print(f"    - {s}")
        else:
            print(f"  {platform}: committed STLs and generated docs are up to date")
    else:
        mdir = stl_dir
        mdir.mkdir(parents=True, exist_ok=True)
        (mdir / "manifest.json").write_text(json.dumps(manifest, indent=1, sort_keys=False) + "\n")
        markdown = manifest_markdown(platform, entries, plates, assembly_report)
        if args.out:
            markdown = markdown.replace('see ../README.md.',
                f'assembly instructions are in hardware/parts/{platform}/README.md in the source repository.')
        (mdir / "manifest.md").write_text(markdown)
        if not args.out and not args.no_docs:
            for path, text in docs.items():
                if path.read_text() != text:
                    path.write_text(text)
                    print(f"  updated {path.relative_to(REPO)}")
        total = sum(p.stat().st_size for p in mdir.glob("*.stl"))
        print(f"  {platform}: {len(entries)} parts, {len(plates)} plates, STLs {total / 1e6:.2f} MB in {mdir}")
    report[platform] = ok
    return ok


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("platforms", nargs="*", help="platforms to build (default: all)")
    ap.add_argument("--part", action="append", help="only this part (repeatable); skips docs and assembly")
    ap.add_argument("--out", help="write STL/STEP/manifest to this directory instead of the repository")
    ap.add_argument("--check", action="store_true", help="check committed STLs and docs are up to date")
    ap.add_argument("--no-step", action="store_true", help="skip STEP export")
    ap.add_argument("--no-docs", action="store_true", help="do not update the generated document blocks")
    ap.add_argument("--no-assembly", action="store_true", help="skip the assembly checks")
    ap.add_argument("--previews", help="render a PNG preview per part into this directory")
    ap.add_argument("--figures", action="store_true", help="regenerate the assembly figures (images/)")
    ap.add_argument("--list", action="store_true", help="list platforms and parts, then exit")
    args = ap.parse_args(argv)
    if args.check and (args.figures or args.previews or args.out):
        ap.error("--check cannot be combined with --figures, --previews or --out (check writes nothing)")

    names = args.platforms or platforms()
    unknown = [p for p in names if p not in platforms()]
    if unknown:
        ap.error(f"no parts for {unknown}; platforms with parts: {platforms()}")
    if args.list:
        for p in names:
            print(p + ": " + ", ".join(part_stems(p)))
        return 0
    report = {}
    for p in names:
        print(f"{p}:")
        run_platform(p, args, report)
    failed = [p for p, ok in report.items() if not ok]
    if failed:
        print(f"FAILED: {', '.join(failed)}")
        return 1
    print("all parts valid" + (" and up to date" if args.check else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
