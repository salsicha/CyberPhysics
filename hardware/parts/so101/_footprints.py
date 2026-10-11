"""Footprints of the parts the SO-101 station's printed parts mate with
(helper, not a part).

Same conventions as hardware/parts/common/footprints.py: millimetres, every
class cites its source and lists in ``UNVERIFIED`` the attributes the cited
document does not state; a printed part that uses one sets ``fit_test``.
They live here, not in common/footprints.py, because that shared file is
frozen while the platforms are added; ``ALL`` lists them for the footprint
test.
"""
from hardware.parts.so101 import _upstream as U

REALSENSE_DATASHEET = ("RealSense D400 Series datasheet 337029-017 (Mar 2026) "
                       "(https://www.realsenseai.com/wp-content/uploads/2026/03/RealSense-D400-Series-Datasheet-Mar-2026.pdf)")


class RealSenseD435i:
    """Intel RealSense D435i (D435 case). Camera frame: x out of the lens,
    y to the camera's left, z up; origin at the centre of the case."""
    SOURCE = (f"{REALSENSE_DATASHEET}: Table 3-52 (90 x 25 x 25 mm, 75 g), Figure 10-9 (two M3 on the back "
              "45 mm apart, max insertion 3 mm, 0.4 N m; 1/4\"-20 underneath), Figure 4-7 and Table 4-20 "
              "(left imager 17.5 mm from the 1/4\"-20 centre line, 50 mm baseline, RGB 15 mm beyond the left "
              "imager), Table 3-18/3-49 (OV2740 colour, 1920 x 1080, 69.4 x 42.5 deg), Table 4-17 (zero depth "
              "4.2 mm behind the front glass), Table 4-12 (min-Z 195 mm at 848 x 480); realsense-ros "
              "_d435.urdf.xacro for the side of the RGB camera")
    WIDTH = 90.0
    HEIGHT = 25.0
    DEPTH = 25.0
    END_RADIUS = 12.5          # the case ends are round in the front view
    MASS_G = 75.0
    BACK_M3_SPACING = 45.0
    BACK_M3_MAX_INSERTION = 3.0
    BACK_M3_TORQUE_NM = 0.4
    # RGB optical centre: on the 1/4"-20 centre line plus 17.5 + 15 mm, to
    # the camera's left (+y); at the zero-depth plane, 4.2 mm behind the
    # glass (the datasheet gives it for the depth imagers; assumed for RGB).
    RGB_Y = 32.5
    RGB_BEHIND_FRONT = 4.2
    COLOR_FOV_DEG = (69.4, 42.5)      # 16:9 streams (1280 x 720 default in realsense-ros)
    MIN_Z_MM = 195.0                  # depth at 848 x 480, the realsense-ros default
    USB = "USB-C on the left end of the case (seen from behind)"
    # Not dimensioned on Figure 10-9: the M3 pair and the 1/4"-20 drawn
    # centred on the case, and the lens row at mid-height.
    M3_CENTRED = True
    LENS_MID_HEIGHT = True
    UNVERIFIED = ("M3_CENTRED", "LENS_MID_HEIGHT", "RGB_BEHIND_FRONT")


class SO101Base:
    """The SO-101 base (upstream Base_SO101.stl) in the station frame: origin
    on the shoulder-pan axis, 22.5 mm above the underside of the base (the
    repository URDF's base_link), x toward the work area, y left, z up."""
    SOURCE = (f"measured on the vendored mesh upstream/Base_SO101.stl ({U.REPOSITORY} at {U.COMMIT[:7]}); "
              "pan axis from upstream Simulation/SO101/so101_new_calib.urdf (shoulder_pan at x 38.84 mm and the "
              "base mesh placement), which assumes that mesh is in the same frame as the published STL")
    FOOTPRINT_X = (-61.2, 25.8)
    FOOTPRINT_Y = (-55.5, 55.5)
    UNDERSIDE_Z = -22.5
    WING_THICKNESS = 16.5         # the two lattice wings the bolts go through
    # Four 5.0 mm through holes in the wings, a 45 deg chamfer on top.
    HOLE_D = 5.0
    HOLES = ((16.5, 27.3), (16.5, -27.3), (-52.7, 31.75), (-52.7, -31.75))
    UNVERIFIED = ("FOOTPRINT_X", "HOLES")   # x positions depend on the pan-axis mapping


class Extrusion2020:
    """20 x 20 mm aluminium extrusion, slot 6 (Misumi HFS5-2020 or any
    20-series B-type profile)."""
    SOURCE = ("Misumi HFS5 series, 20 mm square, 1 slot (https://us.misumi-ec.com/pdf/fa/2012/p2_0511.pdf): "
              "20 x 20 mm, slot width 6 mm, centre hole 4.2 mm (M5 tap), I = 0.742e4 mm4")
    SIZE = 20.0
    SLOT_W = 6.0
    CENTRE_HOLE = 4.2
    I_MM4 = 0.742e4
    E_MPA = 69000.0
    UNVERIFIED = ()


class StationBoard:
    """Design envelope (a requirement): the panel the station is screwed to."""
    SOURCE = ("Requirement of this design: 18 mm plywood or MDF, at least 1200 x 800 mm; the arm 250 mm from "
              "the back edge as on the scenario's workbench")
    THICKNESS = 18.0
    SIZE = (1200.0, 800.0)
    UNVERIFIED = ()


ALL = (RealSenseD435i, SO101Base, Extrusion2020, StationBoard)
