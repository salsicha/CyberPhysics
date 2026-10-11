"""Submarine assembly: the printed parts under a model of the BlueROV2's
bottom panel (its 1/2 in thickness is from Blue Robotics; the 300 x 200 mm
clear area is an assumption) with the DVL, screws and trim washers, the
topside tether spool on its axle, and the checks run by
tools/build_printable_parts.py:

  * interference: no two bodies overlap (screws are modelled at their
    nominal diameter through the holes drilled with the parts as templates);
  * DVL beams: nothing enters the cone that holds the A50's four beams
    (22.5 deg tilt + half the 4.4 deg beam width + a margin) below its face;
  * skids: the DVL face and every screw head stay above the skid line;
  * buoyancy: the added weight in sea water of the printed parts (solid
    ASA), the DVL and the stainless hardware, how many of the BlueROV2's
    200 g ballast weights that replaces, and the same parts in PETG for
    comparison; water uptake of the plastic;
  * spool capacity for the standard and slim tethers.

``figures()`` renders the assembly-instruction images.
"""
import math

from hardware.parts.common import fasteners, printing
from hardware.parts.submarine import _frame as F
from hardware.parts.submarine._footprints import (
    BlueROV2,
    FathomTether,
    Iso7089M16Washer,
    Water,
    WaterLinkedDVLA50,
)

OVERLAP_TOLERANCE_MM3 = 0.5
SCREW_SLACK = 0.02
BEAM_MARGIN_DEG = 2.0
TRIM_WASHERS_EACH = 5             # in the check and the figures
M5_HEAD = (8.5, 5.0)              # ISO 4762 M5: dk, k
M5_SET_G = 8.0                    # screw, two washers and a nyloc nut (estimate)
WATER_UPTAKE = 0.0045             # Bambu ASA and PETG Basic TDS: saturated, 25 C / 55 % RH
SPOOL_OFFSET = (650.0, 0.0, 0.0)  # the spool sits beside the ROV in the model
WINDING_FILL = math.pi / 4        # square-packed round tether


def _screw(axis_point, direction, length, d, head):
    from hardware.parts.common import cad
    dx, dy, dz = direction
    x, y, z = axis_point
    hd, hh = head
    return cad.cyl(d - SCREW_SLACK, length, (x, y, z), direction).fuse(
        cad.cyl(hd, hh, (x - dx * hh, y - dy * hh, z - dz * hh), direction))


def _bolt_xy():
    xy = list(F.DVL_BOLTS)
    xy += [(x, y) for y in F.SKID_Y for x in F.SKID_BOLTS_X]
    xy += list(F.TRIM_XY)
    return xy


def panel():
    """The bottom panel's clear area, drilled for every screw."""
    from hardware.parts.common import cad
    sx, sy = BlueROV2.BOTTOM_CLEAR
    p = cad.box(-sx / 2, sx / 2, -sy / 2 - 20, sy / 2 + 20, 0.0, F.PANEL_T)
    return cad.cut_all(p, [cad.cyl(F.M5_CLEARANCE, F.PANEL_T + 2, (x, y, -1)) for x, y in _bolt_xy()])


def _nut_on_panel(x, y):
    from hardware.parts.common import cad
    z = F.PANEL_T + 1.0
    return cad.cyl(8.0, 5.0, (x, y, z)).cut(cad.cyl(5.0, 7.0, (x, y, z - 1)))


def _purchased_rov():
    """[(name, shape, in-water weight g)] under the panel."""
    from hardware.parts.common import cad
    from hardware.parts.submarine import skid
    out = [("bottom_panel", panel(), 0.0)]
    m5_wet = M5_SET_G * (1 - Water.SEA / Water.STAINLESS)
    cx, cy = F.DVL_XY
    out.append(("dvl_a50", cad.cyl(WaterLinkedDVLA50.DIAMETER, WaterLinkedDVLA50.HEIGHT, (cx, cy, F.DVL_FACE_Z)),
                WaterLinkedDVLA50.MASS_WATER_G))
    for k, (x, y) in enumerate(F.DVL_BOLTS):
        out.append((f"dvl_screw_{k}", _screw((x, y, F.DVL_BACK_Z), (0, 0, 1), 30.0, 5.0, M5_HEAD), m5_wet))
        out.append((f"dvl_nut_{k}", _nut_on_panel(x, y), 0.0))
    seat = -F.SKID[2] + skid.COUNTERBORE[1]
    for y in F.SKID_Y:
        for k, x in enumerate(F.SKID_BOLTS_X):
            out.append((f"skid_screw_{y:g}_{k}", _screw((x, y, seat - 1.0), (0, 0, 1), 35.0, 5.0, M5_HEAD), m5_wet))
            out.append((f"skid_nut_{y:g}_{k}", _nut_on_panel(x, y), 0.0))
    w = Iso7089M16Washer
    w_wet = w.mass_g() * (1 - Water.SEA / Water.STAINLESS)
    end = -F.TRIM_FLANGE[1] - F.TRIM_SPIGOT[1]
    for k, (x, y) in enumerate(F.TRIM_XY):
        for j in range(TRIM_WASHERS_EACH):
            z = -F.TRIM_FLANGE[1] - (j + 1) * w.H
            out.append((f"trim_washer_{k}_{j}", cad.cyl(w.D2, w.H - 0.02, (x, y, z)).cut(
                cad.cyl(w.D1, w.H + 2, (x, y, z - 1))), w_wet))
        out.append((f"trim_fender_{k}", cad.cyl(20.0, 1.5, (x, y, end - 1.5)).cut(cad.cyl(5.3, 3, (x, y, end - 2))), 0.0))
        out.append((f"trim_screw_{k}", _screw((x, y, end - 1.5), (0, 0, 1), 55.0, 5.0, M5_HEAD), m5_wet))
        out.append((f"trim_nut_{k}", _nut_on_panel(x, y), 0.0))
    return out


def _spool():
    """[(name, part id or None, shape)] of the spool in its own frame."""
    from hardware.parts.common import cad
    from hardware.parts.submarine import spool_drum, spool_flange, spool_handle
    length = F.SPOOL_DRUM[2]
    lower = spool_flange.build()
    upper = cad.translate(cad.rotate(lower, "X", 180), (0, 0, length))
    out = [("spool_drum", "spool_drum", spool_drum.build()),
           ("spool_flange_lower", "spool_flange", lower),
           ("spool_flange_upper", "spool_flange", upper),
           ("spool_handle", "spool_handle", spool_handle.build())]
    t = F.SPOOL_FLANGE[1]
    for a in F.SPOOL_SCREW_ANGLES:
        x, y = cad.polar(F.SPOOL_PCD / 2, a)
        out.append((f"spool_screw_lo_{a:g}", None, _screw((x, y, -t), (0, 0, 1), 10.0, fasteners.MAJOR["M4"],
                                                           fasteners.SHCS_HEAD["M4"])))
        out.append((f"spool_screw_hi_{a:g}", None, _screw((x, -y, length + t), (0, 0, -1), 10.0, fasteners.MAJOR["M4"],
                                                           fasteners.SHCS_HEAD["M4"])))
    cx, cy = cad.polar(F.CRANK_R, F.CRANK_ANGLE)
    z_head = -t - spool_flange.CRANK_BOSS[1] - 1.0 - spool_handle.SIZE[1] - 1.0
    out.append(("crank_screw", None, _screw((cx, cy, z_head), (0, 0, 1), spool_handle.SCREW_LENGTH - 0.5, 5.0,
                                            M5_HEAD)))
    out.append(("axle", None, cad.cyl(F.SPOOL_AXLE, 300.0, (0, 0, length / 2 - 150.0))))
    return out


def printed():
    """[(name, part id, shape)] for every printed piece in place (spool beside the ROV)."""
    from hardware.parts.common import cad
    from hardware.parts.submarine import dvl_mount, skid, trim_weight_holder
    out = [("dvl_mount", "dvl_mount", dvl_mount.build())]
    s = skid.build()
    for k, y in enumerate(F.SKID_Y):
        out.append((f"skid_{k}", "skid", cad.translate(s, (0, y, 0))))
    t = trim_weight_holder.build()
    for k, (x, y) in enumerate(F.TRIM_XY):
        out.append((f"trim_weight_holder_{k}", "trim_weight_holder", cad.translate(t, (x, y, 0))))
    for n, stem, shape in _spool():
        if stem:
            out.append((n, stem, cad.translate(shape, SPOOL_OFFSET)))
    return out


def _bbox_overlap(a, b, tol=0.01):
    return not (a.xmax < b.xmin - tol or b.xmax < a.xmin - tol or a.ymax < b.ymin - tol or
                b.ymax < a.ymin - tol or a.zmax < b.zmin - tol or b.zmax < a.zmin - tol)


def beam_cone(depth=1000.0):
    """Cone below the DVL face that holds all four beams, wherever they point."""
    from hardware.parts.common import cad
    half = WaterLinkedDVLA50.BEAM_TILT_DEG + WaterLinkedDVLA50.BEAM_WIDTH_DEG / 2 + BEAM_MARGIN_DEG
    r0 = WaterLinkedDVLA50.DIAMETER / 2
    cx, cy = F.DVL_XY
    return cad.cone(2 * r0, 2 * (r0 + depth * math.tan(math.radians(half))), depth,
                    (cx, cy, F.DVL_FACE_Z - 0.2), (0, 0, -1)), half


def check(entries=None):
    from hardware.parts.common import cad
    entries = entries or {}
    report = {}
    bodies = [(n, s) for n, _stem, s in printed()]
    rov = _purchased_rov()
    bodies += [(n, s) for n, s, _ in rov]
    bodies += [(n, cad.translate(s, SPOOL_OFFSET)) for n, stem, s in _spool() if not stem]
    boxes = [(n, s, s.BoundingBox()) for n, s in bodies]
    overlaps = []
    for i in range(len(boxes)):
        n1, s1, b1 = boxes[i]
        for j in range(i + 1, len(boxes)):
            n2, s2, b2 = boxes[j]
            if not _bbox_overlap(b1, b2):
                continue
            v = s1.intersect(s2).Volume()
            if v > OVERLAP_TOLERANCE_MM3:
                overlaps.append(f"{n1} / {n2}: {v:.1f} mm3")
    report["bodies"] = len(bodies)
    report["interferences"] = overlaps or "none"

    cone, half = beam_cone()
    cb = cone.BoundingBox()
    in_beams = [n for n, s, b in boxes if n not in ("dvl_a50",) and _bbox_overlap(cb, b)
                and s.intersect(cone).Volume() > OVERLAP_TOLERANCE_MM3]
    report["dvl_beams_clear"] = in_beams or f"yes (cone of {half:.1f} deg half-angle from the DVL face)"

    lowest = min(b.zmin for n, s, b in boxes if not n.startswith(("skid_", "spool", "crank", "axle"))
                 and b.xmax < SPOOL_OFFSET[0] - 200)
    skid_ok = lowest > F.SKID_BOTTOM_Z
    report["skid_clearance_mm"] = (f"{lowest - F.SKID_BOTTOM_Z:.1f} between the skid line and the lowest other "
                                   f"part; DVL face {F.DVL_FACE_Z - F.SKID_BOTTOM_Z:.0f} mm above it (the A50 needs "
                                   f"{WaterLinkedDVLA50.MIN_ALTITUDE_MM / 10:g} cm: no lock while sitting on the bottom)")

    # buoyancy in sea water
    def wet(material, mass):
        return mass * (1 - Water.SEA / printing.MATERIALS[material]["density_g_cm3"])

    printed_wet = petg_wet = printed_mass = 0.0
    for stem in ("dvl_mount", "skid", "trim_weight_holder"):
        e = entries.get(stem)
        if not e:
            continue
        m = e["estimate"]["mass_g_all"]
        printed_mass += m
        printed_wet += wet(e["material"], m)
        vol = m / printing.MATERIALS[e["material"]]["density_g_cm3"]
        petg_wet += wet("PETG", vol * printing.MATERIALS["PETG"]["density_g_cm3"])
    trim_wet = sum(wt for n, _s, wt in rov if n.startswith("trim_washer"))
    hardware_wet = sum(wt for n, _s, wt in rov if not n.startswith("trim_washer")) - WaterLinkedDVLA50.MASS_WATER_G
    added = printed_wet + hardware_wet + WaterLinkedDVLA50.MASS_WATER_G
    ballast_wet = BlueROV2.BALLAST_WEIGHT_G * (1 - Water.SEA / Water.STAINLESS)
    n_remove = math.ceil(added / ballast_wet)
    residual = added - n_remove * ballast_wet
    washer_wet = trim_wet / (len(F.TRIM_XY) * TRIM_WASHERS_EACH)
    report["added_weight_in_sea_water_g"] = (
        f"{added:.0f} (DVL {WaterLinkedDVLA50.MASS_WATER_G:g}, printed parts {printed_wet:.0f}, stainless screws and "
        f"nuts {hardware_wet:.0f}), before trim washers")
    report["ballast_to_remove"] = (f"{n_remove} of the 200 g ballast weights ({ballast_wet:.0f} g each in sea water): "
                                   f"{residual:+.0f} g, then about {round(-residual / washer_wet)} trim washers "
                                   f"({washer_wet:.1f} g each in sea water, up to {F.TRIM_WASHERS_MAX} per holder) "
                                   "restore the original trim")
    report["printed_parts_petg_instead_g"] = (f"{petg_wet:.0f} in sea water for the same parts in PETG, against "
                                              f"{printed_wet:.0f} in ASA")
    report["water_uptake_g"] = (f"under {printed_mass * WATER_UPTAKE * 2:.1f} (about {WATER_UPTAKE * 100:.2f} % at 55 % RH "
                                "per the filament data sheets, assumed doubled when soaked)")

    od, _idia, length = F.SPOOL_DRUM
    r_out = F.SPOOL_FLANGE[0] / 2 - 10.0
    vol = math.pi * (r_out ** 2 - (od / 2) ** 2) * length * WINDING_FILL
    per_m = {d: math.pi / 4 * d ** 2 * 1000.0 for d in (FathomTether.STANDARD_D, FathomTether.SLIM_D)}
    report["spool_capacity_m"] = (f"{vol / per_m[FathomTether.STANDARD_D]:.0f} of standard ({FathomTether.STANDARD_D} mm) "
                                  f"or {vol / per_m[FathomTether.SLIM_D]:.0f} of slim ({FathomTether.SLIM_D} mm) tether, "
                                  "10 mm inside the rims")
    report["passed"] = not overlaps and not in_beams and skid_ok
    return report


def _mesh(shape, tol=0.25):
    import numpy as np
    import trimesh
    verts, tris = shape.tessellate(tol, 0.4)
    return trimesh.Trimesh(np.array([v.toTuple() for v in verts]), np.array(tris), process=False)


def figures(out_dir, entries=None):
    from pathlib import Path

    from hardware.parts.common import render
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    report = check(entries)
    materials = {stem: e.get("material", "ASA") for stem, e in (entries or {}).items()}
    rov_printed = [(n, stem, s) for n, stem, s in printed() if not n.startswith("spool")]
    rov_buy = [(n, s) for n, s, _ in _purchased_rov()]
    spool = _spool()

    def color(stem):
        return render.MATERIAL_COLORS.get(materials.get(stem, "ASA"), "#e8833a")

    def items(pieces, buy, offsets=None, skip=()):
        offsets = offsets or {}
        out = []
        for n, stem, s in pieces:
            m = _mesh(s)
            off = next((v for k, v in offsets.items() if n.startswith(k)), None)
            out.append((m.apply_translation(off) if off else m, color(stem)))
        for n, s in buy:
            if any(n.startswith(p) for p in skip):
                continue
            m = _mesh(s)
            off = next((v for k, v in offsets.items() if n.startswith(k)), None)
            out.append((m.apply_translation(off) if off else m, render.PURCHASED_COLOR))
        return out

    render.render(items(rov_printed, rov_buy), out_dir / "overview.png", elev=-28, azim=-55, zoom=1.1,
                  title="Submarine: DVL mount, skids and trim holders (colour) under the BlueROV2 bottom panel (from below)")
    render.render(items(rov_printed, rov_buy,
                        {"dvl_mount": (0, 0, -40), "dvl_a50": (0, 0, -80), "dvl_screw": (0, 0, -60),
                         "skid_": (0, 0, -50), "trim_weight_holder": (0, 0, -40), "trim_washer": (0, 0, -60),
                         "trim_fender": (0, 0, -80), "trim_screw": (0, 0, -90), "dvl_nut": (0, 0, 30),
                         "skid_nut": (0, 0, 30), "trim_nut": (0, 0, 30)}),
                  out_dir / "exploded.png", elev=-20, azim=-60, zoom=0.95, title="Exploded view (from below)")
    sp = [(n, stem, s) for n, stem, s in spool if stem]
    sb = [(n, s) for n, stem, s in spool if not stem]
    render.render(items(sp, sb, {"spool_flange_upper": (0, 0, 60), "spool_screw_hi": (0, 0, 80),
                                 "spool_flange_lower": (0, 0, -60), "spool_screw_lo": (0, 0, -80),
                                 "spool_handle": (0, 0, -90), "crank_screw": (0, 0, -110)}),
                  out_dir / "step_spool.png", elev=12, azim=-50, zoom=0.95, title="Topside tether spool")
    return report
