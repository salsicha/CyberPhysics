"""Printer, materials, tolerances and Bambu Studio print settings.

Pure Python (no CadQuery), so part metadata can be read and checked
anywhere. Every printed part's ``PART["print"]`` comes from
:func:`settings`, which fills the Bambu Studio fields with the defaults
below; a part only states what differs.

The reference printer is a Bambu Lab P1S (enclosed, 256 x 256 x 256 mm,
stock 0.4 mm stainless steel nozzle). Parts are validated
against a smaller usable volume, ``BED_MM``, to leave room for the purge
and wipe area and for brims.
"""

PRINTER = "Bambu Lab P1S, 0.4 mm stainless steel nozzle (stock)"
PRINTER_BUILD_VOLUME_MM = (256.0, 256.0, 256.0)
# Usable volume every printed part must fit, in its print orientation.
BED_MM = (250.0, 250.0, 250.0)

NOZZLE_MM = 0.4
LINE_WIDTH_MM = 0.42
# Thinnest wall a part may have (measured on the mesh) unless its PART
# states a different "min_wall_mm"; three 0.42 mm lines.
DEFAULT_MIN_WALL_MM = 1.2

# Fits between a printed part and a purchased one, per side (radial for
# holes). Printed holes come out ~0.1 mm small on a P1S, which the
# clearance values in fasteners.py already allow for.
CLOSE_FIT_MM = 0.1    # locating features: board edges, sensor bodies
SLIDING_FIT_MM = 0.2  # parts that slide or rotate: cradle walls, pivots
PRESS_FIT_MM = -0.15  # TPU caps pushed onto printed pegs (interference)

# Bambu Studio filament system profiles (P1S) and what they need.
# density_g_cm3 are typical datasheet values used only for the mass
# estimate; flow_mm3_s is a conservative average extrusion rate (walls,
# infill and travel together) used only for the print-time estimate.
MATERIALS = {
    "PETG": {
        "filament_profile": "Bambu PETG Basic",
        "plate": "Textured PEI Plate",
        "density_g_cm3": 1.27,
        "flow_mm3_s": 8.0,
        "enclosure": "door may stay closed; open the lid if the part warps less with it",
        "hardened_nozzle": False,
        "drying": "dry 8 h at 65 C if the spool has been open for weeks (stringing, bubbles)",
        "use": "general structure in the shade: trays, plates inside the airframe",
    },
    "ASA": {
        "filament_profile": "Bambu ASA",
        "plate": "Textured PEI Plate",
        "density_g_cm3": 1.07,
        "flow_mm3_s": 9.0,
        "enclosure": "P1S door closed and lid on for the whole print (warping, fumes)",
        "hardened_nozzle": False,
        "drying": "dry 8 h at 80 C if the spool is damp",
        "use": "sun- and heat-exposed parts: airframe exterior, near motors, ESCs and hot sensors",
    },
    "TPU95A": {
        "filament_profile": "Bambu TPU 95A HF (or Bambu TPU 95A)",
        "plate": "Textured PEI Plate",
        "density_g_cm3": 1.22,
        "flow_mm3_s": 3.5,
        "enclosure": "door open or closed",
        "hardened_nozzle": False,
        "drying": "always dry 8 h at 70 C before printing; load from the external spool holder, not an AMS",
        "use": "dampers, grommets, feet and other flexible parts",
    },
    "PETG-CF": {
        "filament_profile": "Bambu PETG-CF",
        "plate": "Textured PEI Plate",
        "density_g_cm3": 1.25,
        "flow_mm3_s": 7.0,
        "enclosure": "door may stay closed",
        "hardened_nozzle": True,
        "drying": "dry 8 h at 65 C",
        "use": "stiffer PETG for loaded parts (needs the hardened steel nozzle)",
    },
    "PA-CF": {
        "filament_profile": "Bambu PA6-CF",
        "plate": "Textured PEI Plate with glue stick",
        "density_g_cm3": 1.15,
        "flow_mm3_s": 6.0,
        "enclosure": "P1S door closed and lid on",
        "hardened_nozzle": True,
        "drying": "always dry 12 h at 80 C; print straight from a dry box",
        "use": "highly loaded or hot parts (needs the hardened steel nozzle)",
    },
}

# Carbon-filled filaments wear through the stock stainless steel nozzle.
CF_MATERIALS = tuple(name for name, m in MATERIALS.items() if m["hardened_nozzle"])

SUPPORT_CHOICES = (
    "none",
    "tree(auto), on build plate only",
    "normal(auto), on build plate only",
    "tree(auto), everywhere",
)
BRIM_CHOICES = ("auto", "no-brim", "outer brim only, 5 mm", "mouse ear")
INFILL_PATTERNS = ("gyroid", "grid", "cubic", "rectilinear", "honeycomb", "triangles")


def settings(material, orientation, *, layer_height_mm=0.2, wall_loops=3,
             top_shell_layers=5, bottom_shell_layers=4, infill_pct=25,
             infill_pattern="gyroid", supports="none", brim="auto",
             rotate=(), plate=None):
    """Bambu Studio settings for one part, with this repository's defaults.

    ``orientation`` is a sentence for the reader ("flat face down, bosses
    up"); ``rotate`` is the list of (axis, degrees) rotations, applied in
    order about the global axes, that turn the part from its build() frame
    into that print orientation. The build script exports the STL already
    rotated, so a slicer only has to place it.
    """
    m = MATERIALS[material]
    return {
        "printer": PRINTER,
        "filament_profile": m["filament_profile"],
        "plate": plate or m["plate"],
        "process": f"{layer_height_mm:.2f}mm Standard",
        "layer_height_mm": layer_height_mm,
        "wall_loops": wall_loops,
        "top_shell_layers": top_shell_layers,
        "bottom_shell_layers": bottom_shell_layers,
        "sparse_infill_density_pct": infill_pct,
        "sparse_infill_pattern": infill_pattern,
        "supports": supports,
        "brim": brim,
        "orientation": orientation,
        "rotate": [list(r) for r in rotate],
    }


def settings_summary(p):
    """One line of the settings that differ between parts, for tables."""
    s = (f"{p['layer_height_mm']:.2f} mm, {p['wall_loops']} walls, "
         f"{p['sparse_infill_density_pct']}% {p['sparse_infill_pattern']}")
    if p["supports"] != "none":
        s += f", supports {p['supports']}"
    else:
        s += ", no supports"
    if p["brim"] not in ("auto",):
        s += f", brim {p['brim']}"
    return s
