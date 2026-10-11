"""Base plate (standalone turret): carries the pan base, the Arduino UNO
R4 WiFi or UNO Q, and the Waveshare Bus Servo Adapter (A), and screws onto
a tripod (1/4"-20 nut captive under the pan axis) or stands on a bench on
rubber feet."""
from hardware.parts.common import printing
from hardware.parts.turret import _frame as F
from hardware.parts.turret._footprints import ArduinoUnoR4 as U
from hardware.parts.turret._footprints import TripodNut as N
from hardware.parts.turret._footprints import WaveshareBusServoAdapterA as B

X = (-138.0, 40.0)
HALF_W = 56.0
T = 8.0
CORNER_R = 8.0
UNO_ORIGIN = (-133.0, -53.0)        # board corner at the USB/power edge, which faces the rear
UNO_BOSS = (7.2, U.BOTTOM_CLEARANCE + 1.0)
BSA_ORIGIN = (-122.0, 12.0)         # board corner; its 42 mm side along x
BSA_BOSS = (6.0, 6.0)
NUT_CLEAR = 0.15
TIE_SLOTS = ((-50.0, 38.0), (-50.0, 47.0), (-50.0, -38.0), (-50.0, -47.0), (25.0, 40.0), (25.0, 49.0))
WINDOWS = ((-50.0, -8.0, -20.0, 20.0),)


def uno_holes():
    return tuple((UNO_ORIGIN[0] + hx, UNO_ORIGIN[1] + hy) for hx, hy in U.HOLES)


def bsa_holes():
    dx = (B.BOARD[0] - B.HOLE_SPACING[0]) / 2
    dy = (B.BOARD[1] - B.HOLE_SPACING[1]) / 2
    x0, y0 = BSA_ORIGIN
    return tuple((x0 + dx + i * B.HOLE_SPACING[0], y0 + dy + j * B.HOLE_SPACING[1]) for i in (0, 1) for j in (0, 1))


PART = {
    "name": "Base plate (standalone turret, tripod adapter)",
    "quantity": 1,
    "material": "PETG",
    "material_reason": "a large flat part used indoors on a bench or tripod; PETG prints it flat (ASA outdoors)",
    "material_fallback": None,
    "flight_critical": False,
    "print": printing.settings("PETG", "flat: underside on the plate, bosses up", wall_loops=3, infill_pct=25,
                               brim="outer brim only, 5 mm"),
    "mates_with": [
        {"part": "pan_base (printed)", "dimensions": f"four M3 inserts on a {2 * F.BASE_BOLT:g} mm square",
         "source": "hardware/parts/turret/_frame.py", "verified": True},
        {"part": "Arduino UNO R4 WiFi / UNO Q", "dimensions": f"{U.BOARD[0]} x {U.BOARD[1]} mm, dia. {U.HOLE_D} holes "
                                                              f"at {U.HOLES}", "source": U.SOURCE, "verified": True},
        {"part": "Waveshare Bus Servo Adapter (A)", "dimensions": f"{B.BOARD[0]:g} x {B.BOARD[1]:g} mm, dia. {B.HOLE_D} holes "
                                                                  f"on {B.HOLE_SPACING[0]:g} x {B.HOLE_SPACING[1]:g} mm",
         "source": B.SOURCE, "verified": True},
        {"part": "1/4\"-20 hex nut (tripod thread)", "dimensions": f"{N.ACROSS_FLATS} mm across flats, {N.THICKNESS} mm thick",
         "source": N.SOURCE, "verified": True},
    ],
    "inserts": [
        {"size": "M3", "qty": 4, "where": "pan base screws, from the top"},
        {"size": "M3-short", "qty": 4, "where": "Arduino standoffs, from the top"},
        {"size": "M2", "qty": 4, "where": "Bus Servo Adapter standoffs, from the top"},
    ],
    "fasteners": [
        {"item": "M3 x 5 socket head cap screw", "qty": 4, "use": "Arduino UNO to the base plate standoffs"},
        {"item": "M2 x 5 socket head cap screw", "qty": 4, "use": "Bus Servo Adapter (A) to the base plate standoffs"},
        {"item": "1/4\"-20 UNC hex nut", "qty": 1, "use": "tripod thread, captive under the pan base"},
        {"item": "adhesive rubber foot, 10-12 mm", "qty": 4, "use": "base plate corners, for bench use"},
        {"item": "zip tie 2.5 mm", "qty": 3, "use": "servo bus and USB cables on the base plate"},
    ],
    "fit_test": False,
    "notes": [
        "Drop the 1/4\"-20 nut into its hex shaft before bolting the pan base on: the pan base keeps it captive.",
        "The Arduino's USB-C faces the rear edge. With the direct-from-host option (no Arduino) leave its standoffs "
        "empty and plug the adapter's USB-C (jumper on B) into the host.",
    ],
}


def build():
    from hardware.parts.common import cad
    z0, z1 = -T, 0.0
    plate = cad.rounded_plate(X[1] - X[0], 2 * HALF_W, T, CORNER_R, center=((X[0] + X[1]) / 2, 0.0, z0))
    bosses = [cad.cyl(UNO_BOSS[0], UNO_BOSS[1] + 0.01, (x, y, z1 - 0.01)) for x, y in uno_holes()]
    bosses += [cad.cyl(BSA_BOSS[0], BSA_BOSS[1] + 0.01, (x, y, z1 - 0.01)) for x, y in bsa_holes()]
    body = cad.fuse_all([plate] + bosses)
    cutters = []
    for bx in (F.BASE_BOLT, -F.BASE_BOLT):
        for by in (F.BASE_BOLT, -F.BASE_BOLT):
            cutters.append(cad.insert_hole("M3", bx, by, z1))
    for x, y in uno_holes():
        cutters.append(cad.insert_hole("M3-short", x, y, z1 + UNO_BOSS[1]))
    for x, y in bsa_holes():
        cutters.append(cad.insert_hole("M2", x, y, z1 + BSA_BOSS[1]))
    # Tripod: 1/4"-20 screw hole from below, nut in a hex shaft from the top.
    af = N.ACROSS_FLATS + 2 * NUT_CLEAR
    nut_bottom = z0 + 1.6
    hexagon = cad.polygon_prism([cad.polar(af / 3 ** 0.5, 30 + 60 * k) for k in range(6)], nut_bottom, z1 + 1)
    cutters.append(hexagon)
    cutters.append(cad.cyl(6.9, T + 2, (0, 0, z0 - 1)))
    for x, y in TIE_SLOTS:
        cutters.append(cad.cbox(2.6, 5.0, T + 2, center=(x, y, (z0 + z1) / 2)))
    for wx0, wx1, wy0, wy1 in WINDOWS:
        cutters.append(cad.rounded_plate(wx1 - wx0, wy1 - wy0, T + 2, 6.0, center=((wx0 + wx1) / 2, (wy0 + wy1) / 2, z0 - 1)))
    return cad.cut_all(body, cutters)
