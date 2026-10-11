"""The ``PART`` metadata schema every printed-part module must follow.

Pure Python, so tests can check metadata without CadQuery. A part module
``hardware/parts/<platform>/<part>.py`` defines::

    PART = {
        "name": "Motor mount",                 # human-readable name
        "quantity": 4,                         # per vehicle
        "material": "ASA",                     # a key of printing.MATERIALS
        "material_reason": "next to the motor, in the sun",
        "material_fallback": None,             # required for CF materials: a non-CF choice
        "flight_critical": True,               # failure can bring the vehicle down
        "print": printing.settings(...),       # Bambu Studio settings and orientation
        "min_wall_mm": 1.2,                    # optional, default printing.DEFAULT_MIN_WALL_MM
        "mates_with": [                        # every dimension taken from a purchased part
            {"part": "Holybro 2216 KV920 motor",
             "dimensions": "M3 holes on a 16 x 16 mm cross",
             "source": footprints.Holybro2216KV920.SOURCE,
             "verified": True},
        ],
        "inserts": [{"size": "M3", "qty": 2, "where": "pinch clamp"}],
        "fasteners": [{"item": "M3 x 20 socket head cap screw", "qty": 2, "use": "pinch clamp"}],
        "fit_test": False,                     # True when any mates_with entry is unverified
        "notes": ["..."],
    }

    def build():
        import cadquery as cq                  # imported here, not at module level
        ...
        return shape                           # one solid, in the part's design frame

Quantities in ``inserts`` and ``fasteners`` are per printed piece.

A vendored part reuses a published STL instead of a CadQuery model: it has
no ``build()`` and instead of it a ``"vendored"`` entry naming the pinned
file and where it comes from::

        "vendored": {
            "file": "upstream/Base_SO101.stl",     # relative to the platform folder
            "sha256": "19f3c6ba...",               # of that file, checked at every build
            "source": "https://github.com/<owner>/<repo>/blob/<commit>/<path>",
            "licence": "Apache-2.0",               # SPDX identifier
            "licence_file": "upstream/LICENSE",    # the licence text, committed beside it
            "author": "TheRobotStudio and Hugging Face",
        },
"""
import re

from . import fasteners, printing

VENDORED_KEYS = ("file", "sha256", "source", "licence", "licence_file", "author")

REQUIRED = {
    "name": str,
    "quantity": int,
    "material": str,
    "material_reason": str,
    "flight_critical": bool,
    "print": dict,
    "mates_with": list,
    "inserts": list,
    "fasteners": list,
    "fit_test": bool,
    "notes": list,
}
PRINT_KEYS = (
    "printer", "filament_profile", "plate", "process", "layer_height_mm", "wall_loops",
    "top_shell_layers", "bottom_shell_layers", "sparse_infill_density_pct",
    "sparse_infill_pattern", "supports", "brim", "orientation", "rotate",
)


def validate_part(part, stem="part"):
    """Return a list of problems with a PART dict (empty when valid)."""
    errors = []

    def err(msg):
        errors.append(f"{stem}: {msg}")

    if not isinstance(part, dict):
        return [f"{stem}: PART must be a dict"]
    for key, kind in REQUIRED.items():
        if key not in part:
            err(f"missing '{key}'")
        elif not isinstance(part[key], kind) or (kind is int and isinstance(part[key], bool)):
            err(f"'{key}' must be {kind.__name__}")
    if errors:
        return errors

    if not part["name"].strip():
        err("empty name")
    if part["quantity"] < 1:
        err("quantity must be >= 1")
    material = part["material"]
    if material not in printing.MATERIALS:
        err(f"unknown material '{material}' (one of {sorted(printing.MATERIALS)})")
    fallback = part.get("material_fallback")
    if material in printing.CF_MATERIALS:
        if not fallback:
            err(f"{material} needs the hardened nozzle: give a non-CF 'material_fallback'")
        elif fallback in printing.CF_MATERIALS or fallback not in printing.MATERIALS:
            err(f"material_fallback '{fallback}' must be a known non-CF material")
    elif fallback is not None and fallback not in printing.MATERIALS:
        err(f"unknown material_fallback '{fallback}'")

    p = part["print"]
    for key in PRINT_KEYS:
        if key not in p:
            err(f"print settings miss '{key}'")
    if not errors:
        if not 0.08 <= p["layer_height_mm"] <= 0.32:
            err("layer_height_mm outside 0.08-0.32 (0.4 mm nozzle)")
        if not 1 <= p["wall_loops"] <= 12:
            err("wall_loops outside 1-12")
        if not 0 <= p["sparse_infill_density_pct"] <= 100:
            err("sparse_infill_density_pct outside 0-100")
        if p["sparse_infill_pattern"] not in printing.INFILL_PATTERNS:
            err(f"unknown infill pattern '{p['sparse_infill_pattern']}'")
        if p["supports"] not in printing.SUPPORT_CHOICES:
            err(f"supports must be one of {printing.SUPPORT_CHOICES}")
        if p["brim"] not in printing.BRIM_CHOICES:
            err(f"brim must be one of {printing.BRIM_CHOICES}")
        if not str(p["orientation"]).strip():
            err("print orientation not described")
        for r in p["rotate"]:
            if len(r) != 2 or r[0] not in ("X", "Y", "Z") or not isinstance(r[1], (int, float)):
                err(f"rotate entries are [axis, degrees], got {r!r}")
        if material in printing.MATERIALS and p["filament_profile"] != printing.MATERIALS[material]["filament_profile"]:
            err("filament_profile does not match the material")

    if "min_wall_mm" in part and not 0.4 <= part["min_wall_mm"] <= 10:
        err("min_wall_mm outside 0.4-10")

    if not part["mates_with"]:
        err("mates_with is empty: name what the part fits onto")
    unverified = False
    for m in part["mates_with"]:
        for key in ("part", "dimensions", "source", "verified"):
            if key not in m:
                err(f"mates_with entry {m.get('part', '?')!r} misses '{key}'")
        if not isinstance(m.get("verified"), bool):
            err(f"mates_with entry {m.get('part', '?')!r}: 'verified' must be bool")
        elif not m["verified"]:
            unverified = True
        if not str(m.get("source", "")).strip():
            err(f"mates_with entry {m.get('part', '?')!r} has no source")
    if unverified and not part["fit_test"]:
        err("an unverified mating dimension requires fit_test = True")

    for i in part["inserts"]:
        if i.get("size") not in fasteners.INSERTS:
            err(f"insert size {i.get('size')!r} not in fasteners.INSERTS")
        if not isinstance(i.get("qty"), int) or i["qty"] < 1:
            err("insert qty must be a positive int")
        if not i.get("where"):
            err("insert entry needs 'where'")
    for f in part["fasteners"]:
        if not f.get("item") or not f.get("use"):
            err("fastener entries need 'item' and 'use'")
        if not isinstance(f.get("qty"), int) or f["qty"] < 1:
            err(f"fastener {f.get('item')!r}: qty must be a positive int")
    for n in part["notes"]:
        if not isinstance(n, str):
            err("notes must be strings")
    if "vendored" in part:
        errors += validate_vendored(part["vendored"], stem)
    return errors


def validate_vendored(v, stem="part"):
    """Problems with a PART's "vendored" entry (empty when valid)."""
    if not isinstance(v, dict):
        return [f"{stem}: 'vendored' must be a dict"]
    errors = [f"{stem}: vendored entry misses '{k}'" for k in VENDORED_KEYS if not str(v.get(k, "")).strip()]
    if errors:
        return errors
    for key in ("file", "licence_file"):
        if v[key].startswith("/") or ".." in v[key].split("/"):
            errors.append(f"{stem}: vendored '{key}' must be a path inside the platform folder")
    if not v["file"].lower().endswith(".stl"):
        errors.append(f"{stem}: vendored file must be an STL")
    if not re.fullmatch(r"[0-9a-f]{64}", v["sha256"]):
        errors.append(f"{stem}: vendored sha256 must be 64 lower-case hex digits")
    if not v["source"].startswith("https://"):
        errors.append(f"{stem}: vendored source must be an https URL (pinned to a commit)")
    return errors
