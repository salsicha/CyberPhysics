"""Foam dart launcher core (helper, not a part): parametric geometry shared
by the turret's launcher parts and, later, the aerodrone's.

TOY POWER ONLY. A spring-plunger launcher for Elite-size foam darts: the
purchased spring of SPRING stores about 1.1 J, which at the most efficient
a springer reaches gives at most about 30 m/s (see muzzle_velocity()); the
velocity is confirmed with a chronograph at bring-up. Foam darts only; no
stronger spring.

Launcher frame (millimetres): origin on the bore axis at the chamber's rear
face, x along the bore toward the muzzle, z toward the plunger tube's side
(up on the turret), y = z x x. Along x:

  sear housing, rear cap | plunger tube | breech: rear plate, chamber
  (a sliding breech block, the "shuttle", with the clip on top), front
  wall | barrel clamp | barrel (purchased brass tube)

The plunger tube's axis is PT_Z above the bore; its outlet is off-centre,
on the bore. The priming drive is a rack along -y with a pinion on an
ST3215 (the actuator, bus ID 14) under it: pulling back by ACT_PRIME, the
rack's hook draws the plunger's rod until the sear drops into the rod's
groove, and the rack's cam slot moves the shuttle +y by TRAVEL, under the
clip, where the clip's spring pushes one dart into the shuttle's trough;
returning forward, the rack leaves the plunger on the sear and cams the
shuttle back onto the bore. A second ST3215 (the trigger, ID 15) at +y
lifts the sear with a pin on an arm: the shot. A manual safety slides
over the sear and blocks it.

Builders return CadQuery solids in this frame (CadQuery is imported inside
them); place() puts one into a vehicle frame. Mating dimensions of the
purchased parts are in hardware/parts/turret/_footprints.py.
"""
import math

from hardware.parts.turret import _gears as G
from hardware.parts.turret._footprints import ST3215 as S

HORN_FACE = S.HORN_FACES / 2
HALF_T = S.THICKNESS / 2

# --------------------------------------------------------------------- ammunition, purchased parts
# Elite-size foam dart: 70-73.5 mm long, 12.7 mm, about 1.0 g (community
# measurements: blasterhub.com, blastercentral.com; Hasbro publishes no
# drawing). Genuine Nerf magazines are not supported: their dimensions are
# not published.
DART_LEN = 73.5
DART_D = 12.7
DART_MASS_G = 1.0

# K&S Precision Metals 8140 round brass tube, 17/32" OD x 0.014" wall, ID
# 0.503" (+-0.002"), 12" long (ksmetals.com): cut to BARREL_LEN.
BARREL = {"item": "K&S Precision Metals 8140 brass tube, 17/32 in OD x 0.014 in wall (ID 0.503 in, 12.78 mm), 12 in",
          "od": 13.49, "id": 12.78, "length": 45.0, "mass_g": 7.5}

# Plunger spring, purchased: music wire (ASTM A228), closed and ground ends.
# Installed SPRING_INSTALLED long (8 mm preload); the sear holds the plunger
# drawn DRAW_CAUGHT; priming overtravels to DRAW_PRIME.
SPRING = {"wire_d": 2.0, "od": 28.0, "free_len": 80.0, "rate_n_mm": 1.5, "rate_tol": 0.10,
          "active_coils": 6, "total_coils": 8, "solid_len": 16.0, "mass_g": 16.0}
SPRING_INSTALLED = 72.0
DRAW_CAUGHT = 31.25
DRAW_PRIME = 35.0
# Clip spring: a light magazine spring.
CLIP_SPRING = {"wire_d": 0.6, "od": 12.0, "free_len": 110.0, "rate_n_mm": 0.05}
# Sear spring: small, between the sear's peg and the housing's roof.
SEAR_SPRING = {"wire_d": 0.4, "od": 4.5, "free_len": 16.0, "rate_n_mm": 0.3}
O_RINGS = (("NBR 70 O-ring 27 x 2.5 mm", 1, "plunger head"),
           ("NBR 70 O-ring 10 x 2 mm", 1, "plunger tube outlet spigot in the breech"))

# --------------------------------------------------------------------- breech and shuttle
CHAMBER = (0.0, 76.0)               # the chamber: a 73.5 mm dart with 2.5 mm to spare
REAR_PLATE_X = (-8.0, 0.0)
REAR_PLATE_UPPER_X1 = -1.5          # above the cover, clear of the clip
FRONT_WALL_X = (CHAMBER[1], CHAMBER[1] + 8.0)
TROUGH_R = 6.6
SHUTTLE_TOP = 6.75
SHUTTLE_FLOOR_T = 2.4
SHUTTLE_Z0 = -TROUGH_R - SHUTTLE_FLOOR_T          # -9.0
SHUTTLE_CLEAR = 0.15
ROOF = SHUTTLE_TOP + 0.1                          # the cover's underside: the chamber's roof
TRAVEL = 17.0                                     # shuttle +y from fire to load
SHUTTLE_Y = (-TROUGH_R - 4.0, TRAVEL + 6.7 + 3.3)  # at fire: -10.6..27
CHANNEL_Y = (SHUTTLE_Y[0] - SHUTTLE_CLEAR, SHUTTLE_Y[1] + TRAVEL + SHUTTLE_CLEAR)
WALL_T = 3.0
BREECH_Y = (CHANNEL_Y[0] - WALL_T, CHANNEL_Y[1] + WALL_T)
FLOOR_T = 3.0
BREECH_Z0 = SHUTTLE_Z0 - SHUTTLE_CLEAR - FLOOR_T  # -12.15
COVER_T = 4.5
COVER_TOP = ROOF + COVER_T                        # 12.85
COVER_SCREWS = ((30.0, -14.4), (60.0, -14.4), (30.0, 47.8), (60.0, 47.8))   # (x, y), M3 inserts in wall bosses
OUTLET = (CHAMBER[0] + 0.75, CHAMBER[1] - 1.4, TRAVEL - 6.7, TRAVEL + 6.7)    # x0, x1, y0, y1 in the cover: 73.85 long
TAB_X = (4.0, 12.0)
TAB_Z = (SHUTTLE_Z0, SHUTTLE_Z0 + 6.0)
PIN_D = 3.0                         # 3 x 10 mm steel dowel pin, pressed into the tab
PIN_HOLE_D = 2.9
PIN_Y_FIRE = -42.0                  # the cam plate stays at y < -20.7, outside the pan stage at any tilt
PIN_Z0 = -13.0
BARREL_STOP = 0.0                   # the barrel's rear face flush with the chamber's front face (set with a straight edge)
FRONT_SCREWS = ((10.0, -6.0), (-10.0, -6.0))      # (y, z) M3 inserts in the front wall: barrel clamp
UNDER_SCREWS = ((-4.0, 11.0), (-4.0, -11.0), (80.0, 11.0), (80.0, -11.0))   # (x, y) M3 inserts in the underside
# (the pair at x 80 meets the barrel clamp's inserts, FRONT_SCREWS: a breech
# without a pedestal passes its own under_screws, as the aerodrone's does)

# Clip on the cover, at the shuttle's load position.
CLIP_DARTS = 6
CLIP_WALL = 1.3
CLIP_IN_X = (OUTLET[0], OUTLET[1])
CLIP_IN_Y = (TRAVEL - 7.5, TRAVEL + 7.5)          # 15 wide: the 14.8 mm follower does not pass the 13.4 mm outlet
CLIP_TOP = 104.0
CLIP_PINS_X = (15.0, 60.0)                        # M3 x 25 pins over the clip cap
CLIP_PIN_Z = CLIP_TOP - 3.5
FOLLOWER_T = 4.0
CLIP_CAP_T = 3.0

# --------------------------------------------------------------------- plunger tube, plunger, sear
PT_Z = 10.0
PT_BORE = 32.0
PT_WALL = 2.4
PT_X = (REAR_PLATE_X[0] - 89.0, REAR_PLATE_X[0])  # -97..-8 (front wall x -13..-8)
PT_FRONT_T = 5.0
PT_FLANGE_HALF = 22.0               # square front flange, y +-22
PT_FLANGE_Z = (-9.0, 33.0)          # clear of the rack's cam plate below
PT_FLANGE_SCREWS = ((17.0, -5.0), (-17.0, -5.0), (17.0, 27.0), (-17.0, 27.0))   # (y, z)
PT_REAR_FLANGE_D = 48.0
PT_REAR_FLANGE_T = 8.0
PT_REAR_SCREW_R = 19.6
PT_REAR_SCREWS_DEG = (180.0, 270.0, 330.0)  # clear of the sear housing, its lug and the trigger arm
PT_LUGS_X = (-78.0, -46.0)          # -y lugs for the actuator servo mount (M3 inserts from below)
PT_LUG = (8.0, -25.0, -14.0, 2.0, 14.0)            # x width, y0, y1, z0, z1
PT_LUG_SCREW_Y = -20.5
SPIGOT_D = 13.6
SPIGOT_GROOVE = (10.6, 2.6)         # O-ring 10 x 2 groove: root diameter, width
OUTLET_D = 8.0
HEAD_T = 12.0
HEAD_D = PT_BORE - 0.6
HEAD_GROOVE = (PT_BORE - 4.4, 3.4)  # O-ring 27 x 2.5 groove: root diameter, width
ROD_D = 8.0
GROOVE_CORE_D = 5.0
GROOVE_FLAT = 4.6
GROOVE_RAMP = 1.5
COLLAR_D = 14.0
COLLAR_T = 3.0
CAP_X = (PT_X[0] - 6.0, PT_X[0])    # -103..-97: the spring's rear seat is x -97
HOUSING_X = (CAP_X[0] - 8.0, CAP_X[0])   # -111..-103
HOUSING_Y = (-10.0, 12.0)
HOUSING_Z = (2.0, 38.0)
SEAR_X = (-108.5, -105.5)
SEAR_Y = (-8.0, 10.0)
SEAR_SLOT_W = 5.4                   # straddles the groove's 5 mm core, not the 8 mm rod
SEAR_LEG = 4.0
SEAR_ABOVE = 10.0                   # plate height above the notch top
SEAR_LUG_Y = (10.0, 16.0)
SEAR_SPRING_Y = -5.0
SAFETY_Y = (0.0, 8.0)
SAFETY_GAP = 0.8                    # the tongue sits this far above the caught sear: lift 0.8 < 4.45 to release
SAFETY_T = 3.0
SAFETY_TRAVEL = 5.0
HOOK_GAP = 3.0                      # the collar stops 3 mm short of the rack's hook after a shot
HOOK_X = (HOUSING_X[0] - 4.5, HOUSING_X[0] - 0.5)   # the rack's hook at rest: -115.5..-111.5

# --------------------------------------------------------------------- priming drive (rack, pinion, actuator servo)
ACT_PRIME = HOOK_GAP + DRAW_PRIME   # 38 mm: actuator_joint at prime
ACT_CATCH = HOOK_GAP + DRAW_CAUGHT  # 34.25 mm: the sear drops in from here
ACT_OVERTRAVEL = 1.0
RACK_Y = (-31.0, -23.0)
RACK_ROOT_Z = -14.0                 # rack body underside: tooth roots (teeth down, pinion below)
RACK_TOP_Z = -6.0
CAM_Z = (-14.0, -9.5)
CAM_PLATE_Y = (PIN_Y_FIRE - (PIN_D + 0.4) / 2 - 2.0, PIN_Y_FIRE + TRAVEL + (PIN_D + 0.4) / 2 + 2.0)
CAM_PLATE_X = (3.0, 51.0)
CAM_SHELF_X = (0.0, 60.0)          # the breech's shelf under the cam plate (turret pedestal)
CAM_PATH = ((0.0, 0.0), (2.0, 0.0), (30.0, TRAVEL), (40.0, TRAVEL))   # (rack travel u, shuttle y)
SLOT_W = PIN_D + 0.4
RACK_PITCH_Z = RACK_ROOT_Z - G.DEDENDUM          # -15.875
ACT_SHAFT = (-57.0, RACK_Y[0] - HORN_FACE, RACK_PITCH_Z - G.PINION_R)   # x, servo mid-plane y, z
HOOK_PLATE = (RACK_Y[0], 5.0, RACK_ROOT_Z, 20.0)  # y0, y1, z0, z1

# Actuator servo mount (hangs from the plunger tube's lugs).
MOUNT_BAR_Z = (-2.0, PT_LUG[3])
MOUNT_BAR_Y1 = -17.5                # clear of the tube's outside
MOUNT_FLOOR_T = 3.5
MOUNT_WALL_T = 4.0

# --------------------------------------------------------------------- trigger
TRIG_SHAFT = (-121.0, 40.0, PT_Z + 1.0)           # x, servo mid-plane y, z; output face toward -y, far end rearward
TRIG_ARM_R = 14.0
TRIG_PIN_D = 4.0
TRIG_FIRE_RAD = 0.6                 # the pin lifts the sear 5.4 mm (4.45 releases it)
TRIG_ARM_T = 3.0

# --------------------------------------------------------------------- barrel clamp, muzzle cap
CLAMP_X = (FRONT_WALL_X[1], FRONT_WALL_X[1] + 12.0)
CLAMP_HALF_W = 15.0
PINCH_Z = 10.0
MUZZLE_X = CHAMBER[1] + BARREL_STOP + BARREL["length"]   # the barrel's front end
CAP_DEPTH = 10.0


# --------------------------------------------------------------------- derived positions

def notch_top(draw=None, lift=0.0):
    """z of the sear's notch top: in the groove (caught), else riding on the rod."""
    caught = PT_Z + GROOVE_CORE_D / 2
    riding = caught + SEAR_LEG + math.sqrt((ROD_D / 2) ** 2 - (SEAR_SLOT_W / 2) ** 2) - GROOVE_CORE_D / 2
    return (caught if draw is None else riding) + lift


def sear_release_lift():
    return notch_top(draw=0.0) - notch_top()


def groove_x(draw=0.0):
    """(catch face, ramp end) of the rod's groove at this draw: the catch
    face meets the sear's rear face at DRAW_CAUGHT."""
    catch = SEAR_X[0] + DRAW_CAUGHT - draw
    return catch, catch + GROOVE_FLAT + GROOVE_RAMP


def rod_x(draw=0.0):
    head_front = PT_X[1] - PT_FRONT_T - draw
    collar_front = HOOK_X[0] - HOOK_GAP - draw
    return {"head": (head_front - HEAD_T, head_front), "collar": (collar_front - COLLAR_T, collar_front),
            "tail": collar_front - COLLAR_T}


def draw_at(u):
    """Plunger draw at rack travel u (rising), before the sear catches."""
    return max(0.0, u - HOOK_GAP)


def shuttle_y(u):
    """Shuttle offset (+y) at rack travel u, from CAM_PATH."""
    pts = CAM_PATH
    if u <= pts[0][0]:
        return pts[0][1]
    for (u0, y0), (u1, y1) in zip(pts, pts[1:]):
        if u <= u1:
            return y0 + (y1 - y0) * (u - u0) / (u1 - u0)
    return pts[-1][1]


def act_case_x(servo_x):
    """x of a point at servo X on the actuator servo (case toward -x)."""
    return ACT_SHAFT[0] + S.SHAFT_X - servo_x


def trig_case_x(servo_x):
    """x of a point at servo X on the trigger servo (case toward -x)."""
    return TRIG_SHAFT[0] + S.SHAFT_X - servo_x


def rack_teeth_x():
    """Rack tooth centres along x at rest: around the pinion over the stroke,
    one over the pinion centre at rest (the pinion has a gap at its top)."""
    p = math.pi * G.MODULE
    lo, hi = ACT_SHAFT[0] - 8.0, ACT_SHAFT[0] + ACT_PRIME + ACT_OVERTRAVEL + 6.0
    k0, k1 = math.ceil((lo - ACT_SHAFT[0]) / p), math.floor((hi - ACT_SHAFT[0]) / p)
    return [ACT_SHAFT[0] + k * p for k in range(k0, k1 + 1)]


def spring_energy_j(rate=SPRING["rate_n_mm"], draw=DRAW_CAUGHT):
    pre = SPRING["free_len"] - SPRING_INSTALLED
    return 0.5 * rate * ((pre + draw) ** 2 - pre ** 2) / 1000.0


def muzzle_velocity(efficiency, rate=SPRING["rate_n_mm"], dart_g=DART_MASS_G):
    """Dart speed (m/s) if ``efficiency`` of the spring's energy reaches the dart."""
    return math.sqrt(2 * efficiency * spring_energy_j(rate) / (dart_g / 1000.0))


def volumes_cm3():
    """(plunger swept volume, bore volume behind the dart at exit)."""
    swept = math.pi / 4 * PT_BORE ** 2 * DRAW_CAUGHT / 1000.0
    bore = math.pi / 4 * BARREL["id"] ** 2 * (DART_LEN + BARREL["length"]) / 1000.0
    return swept, bore


def prime_force_n(draw=DRAW_PRIME, rate=SPRING["rate_n_mm"]):
    """Spring force on the rod at this draw."""
    return rate * (SPRING["free_len"] - SPRING_INSTALLED + draw)


# --------------------------------------------------------------------- placing

def place(shape, origin=(0, 0, 0), rotations=()):
    """Launcher-frame shape -> a vehicle frame: rotations [(axis, deg)] in
    order, then translated to ``origin``."""
    from hardware.parts.common import cad
    for axis, deg in rotations:
        shape = cad.rotate(shape, axis, deg)
    return cad.translate(shape, origin)


def _yz_prism(points, x0, x1):
    """Prism of a (y, z) polygon from x0 to x1."""
    import cadquery as cq
    return cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(points).close().extrude(x1 - x0).val()


# --------------------------------------------------------------------- printed parts

def breech(pedestal=None, under_screws=UNDER_SCREWS):
    """Breech: the shuttle's channel (floor, side walls with the cover
    bosses, the tab opening at -y), the rear plate (the plunger tube's
    spigot socket and flange inserts) and the front wall (the barrel's seat
    and the clamp's inserts). ``pedestal``: (x0, x1, y0, y1, z0, screws)
    adds a mounting block underneath with M3 inserts from below at
    ``screws`` [(x, y)]; without one, M3 inserts at ``under_screws``
    [(x, y)] in the rear plate and the front wall (the floor between them
    is too thin for an insert)."""
    from hardware.parts.common import cad
    bx0, bx1 = REAR_PLATE_X[0], FRONT_WALL_X[1]
    y0, y1 = BREECH_Y
    parts = [cad.box(bx0, bx1, y0, y1, BREECH_Z0, ROOF),
             cad.box(bx0, REAR_PLATE_X[1], -PT_FLANGE_HALF, y0 + 0.01, PT_FLANGE_Z[0], ROOF),     # -y ear for the flange
             cad.box(bx0, REAR_PLATE_UPPER_X1, -PT_FLANGE_HALF, PT_FLANGE_HALF, ROOF - 0.01, PT_FLANGE_Z[1]),
             cad.box(bx0, REAR_PLATE_UPPER_X1, -PT_FLANGE_HALF, y1, ROOF - 0.01, COVER_TOP),
             cad.box(FRONT_WALL_X[0], bx1, y0, y1, ROOF - 0.01, COVER_TOP)]
    for x, y in COVER_SCREWS:
        ya, yb = (y0 - 5.75, y0 + 0.01) if y < 0 else (y1 - 0.01, y1 + 5.75)
        parts.append(cad.box(x - 4.0, x + 4.0, ya, yb, ROOF - 8.5, ROOF))
    if pedestal:
        px0, px1, py0, py1, pz0, _ = pedestal
        parts.append(cad.box(px0, px1, py0, py1, pz0, BREECH_Z0 + 0.01))
        # Shelf under the cam plate, on a 45 deg gusset from the pedestal's side.
        sy0 = CAM_PLATE_Y[1] - 6.0
        top = CAM_Z[0] - 0.2
        edge = top - 1.5
        parts.append(_yz_prism([(py0 + 0.01, pz0), (py0 + 0.01, top), (sy0, top), (sy0, edge),
                                (sy0 + (edge - pz0), pz0)], CAM_SHELF_X[0], CAM_SHELF_X[1]))
    body = cad.fuse_all(parts)
    cutters = [cad.box(CHAMBER[0], CHAMBER[1], CHANNEL_Y[0], CHANNEL_Y[1], SHUTTLE_Z0 - SHUTTLE_CLEAR, ROOF + 1.0),
               # tab opening in the -y wall
               cad.box(TAB_X[0] - 0.4, TAB_X[1] + 0.4, y0 - 1.0, CHANNEL_Y[0] + 0.01,
                       SHUTTLE_Z0 - SHUTTLE_CLEAR, ROOF + 1.0),          # open at the top: the shuttle drops in
               # spigot socket in the rear plate, on the bore, and the outlet into the chamber
               cad.cyl(SPIGOT_D + 0.3, REAR_PLATE_UPPER_X1 - bx0 + 0.01, (bx0 - 0.01, 0, 0), (1, 0, 0)),
               cad.cyl(OUTLET_D, REAR_PLATE_X[1] - REAR_PLATE_UPPER_X1 + 0.02, (REAR_PLATE_UPPER_X1 - 0.01, 0, 0), (1, 0, 0)),
               # barrel seat and the lip it stops on
               cad.cyl(BARREL["od"] + 0.2, bx1 - CHAMBER[1] + 0.02, (CHAMBER[1] - 0.01, 0, 0), (1, 0, 0))]
    for y, z in PT_FLANGE_SCREWS:
        size = "M3" if z < ROOF else "M3-short"
        cutters.append(cad.insert_hole_axis(size, (bx0, y, z), (1, 0, 0)))
    for x, y in COVER_SCREWS:
        cutters.append(cad.insert_hole("M3", x, y, ROOF))
    for y, z in FRONT_SCREWS:
        cutters.append(cad.insert_hole_axis("M3", (bx1, y, z), (-1, 0, 0)))
    if pedestal:
        px0, px1, py0, py1, pz0, screws = pedestal
        for x, y in screws:
            cutters.append(cad.insert_hole("M3", x, y, pz0, direction=1))
        # lightening pockets between the screws, from below
        cutters.append(cad.box(px0 + 14.0, px1 - 10.0, py0 + 3.0, py1 - 3.0, pz0 - 1.0, BREECH_Z0 - 3.0))
    else:
        for x, y in under_screws:
            cutters.append(cad.insert_hole("M3", x, y, BREECH_Z0, direction=1))
    return cad.cut_all(body, cutters)


def shuttle(u=0.0):
    """Sliding breech block at rack travel u: the trough (chamber) open at
    the top and at both ends, the tab out of the -y wall and its cam pin."""
    from hardware.parts.common import cad
    dy = shuttle_y(u)
    c = SHUTTLE_CLEAR
    x0, x1 = CHAMBER[0] + c, CHAMBER[1] - c
    block = cad.box(x0, x1, SHUTTLE_Y[0] + dy, SHUTTLE_Y[1] + dy, SHUTTLE_Z0, SHUTTLE_TOP)
    tab = cad.box(TAB_X[0], TAB_X[1], PIN_Y_FIRE - PIN_D / 2 - 2.5 + dy, SHUTTLE_Y[0] + dy + 0.01, TAB_Z[0], TAB_Z[1])
    xm = (TAB_X[0] + TAB_X[1]) / 2
    body = cad.fuse_all([block, tab])
    trough = cad.cyl(2 * TROUGH_R, x1 - x0 + 2, (x0 - 1, dy, 0), (1, 0, 0)).fuse(
        cad.box(x0 - 1, x1 + 1, dy - TROUGH_R, dy + TROUGH_R, 0, SHUTTLE_TOP + 1))
    hole = cad.cyl(PIN_HOLE_D, TAB_Z[1] - TAB_Z[0] + 2, (xm, PIN_Y_FIRE + dy, TAB_Z[0] - 1))
    pocket = cad.box(x0 + 2.0, x1 - 2.0, dy + TROUGH_R + 2.0, SHUTTLE_Y[1] + dy - 2.0, SHUTTLE_Z0 - 1, SHUTTLE_TOP - 2.0)
    return body.cut(trough).cut(hole).cut(pocket).clean()


def cam_pin(u=0.0):
    """The purchased dowel pin in the shuttle's tab (model)."""
    from hardware.parts.common import cad
    xm = (TAB_X[0] + TAB_X[1]) / 2
    return cad.cyl(PIN_D, 10.0, (xm, PIN_Y_FIRE + shuttle_y(u), PIN_Z0))


def clip():
    """Cover (the chamber's roof, with the outlet over the load position)
    and the clip box on it, one part: printed roof down."""
    from hardware.parts.common import cad
    y0, y1 = BREECH_Y
    cover = cad.box(REAR_PLATE_UPPER_X1 + 0.1, CHAMBER[1] - 0.1, y0 - 5.75, y1 + 5.75, ROOF, COVER_TOP)
    w = CLIP_WALL
    box = cad.box(CLIP_IN_X[0] - w, CLIP_IN_X[1] + w, CLIP_IN_Y[0] - w, CLIP_IN_Y[1] + w, COVER_TOP - 0.01, CLIP_TOP)
    body = cover.fuse(box).clean()
    cutters = [cad.box(OUTLET[0], OUTLET[1], OUTLET[2], OUTLET[3], ROOF - 1, COVER_TOP + 0.02),
               cad.box(CLIP_IN_X[0], CLIP_IN_X[1], CLIP_IN_Y[0], CLIP_IN_Y[1], COVER_TOP, CLIP_TOP + 1)]
    for x, y in COVER_SCREWS:
        cutters.append(cad.clearance_hole("M3", x, y, ROOF - 1, COVER_TOP + 1))
    # windows in the clip's side walls (8 mm: narrower than a dart), lightening
    for k in range(6):
        z0 = COVER_TOP + 8.0 + k * 13.0
        for x0 in (6.0, 41.0):
            cutters.append(cad.box(x0, x0 + 28.0, CLIP_IN_Y[0] - w - 1, CLIP_IN_Y[1] + w + 1, z0, z0 + 8.0))
    for x in CLIP_PINS_X:
        cutters.append(cad.cyl(3.4, CLIP_IN_Y[1] - CLIP_IN_Y[0] + 2 * w + 2, (x, CLIP_IN_Y[0] - w - 1, CLIP_PIN_Z), (0, 1, 0)))
    return cad.cut_all(body, cutters)


def follower(darts=CLIP_DARTS):
    """Clip follower on top of ``darts`` darts (0: resting on the cover)."""
    from hardware.parts.common import cad
    z0 = SHUTTLE_TOP + 0.2 + darts * DART_D if darts else COVER_TOP
    x0, x1 = CLIP_IN_X[0] + 0.3, CLIP_IN_X[1] - 0.3
    body = cad.box(x0, x1, CLIP_IN_Y[0] + 0.1, CLIP_IN_Y[1] - 0.1, z0, z0 + FOLLOWER_T)
    return body.fuse(cad.cyl(CLIP_SPRING["od"] - 2 * CLIP_SPRING["wire_d"] - 1.0, 6.0,
                             ((x0 + x1) / 2, TRAVEL, z0 + FOLLOWER_T - 0.01))).clean()


def clip_cap():
    """Clip cap under the two pins: the spring's top seat."""
    from hardware.parts.common import cad
    z1 = CLIP_PIN_Z - 1.7
    x0, x1 = CLIP_IN_X[0] + 0.3, CLIP_IN_X[1] - 0.3
    body = cad.box(x0, x1, CLIP_IN_Y[0] + 0.1, CLIP_IN_Y[1] - 0.1, z1 - CLIP_CAP_T, z1)
    return body.fuse(cad.cyl(CLIP_SPRING["od"] - 2 * CLIP_SPRING["wire_d"] - 1.0, 6.0,
                             ((x0 + x1) / 2, TRAVEL, z1 - CLIP_CAP_T - 6.0))).clean()


def tube():
    """Plunger tube: bore PT_BORE on y = 0, z = PT_Z, front wall with the
    off-centre outlet spigot on the bore, square front flange onto the
    breech, round rear flange for the cap, lugs for the actuator mount."""
    from hardware.parts.common import cad
    x0, x1 = PT_X
    od = PT_BORE + 2 * PT_WALL
    fz0, fz1 = PT_FLANGE_Z
    parts = [cad.cyl(od, x1 - x0, (x0, 0, PT_Z), (1, 0, 0)),
             cad.box(x1 - PT_FRONT_T, x1, -PT_FLANGE_HALF, PT_FLANGE_HALF, fz0, fz1),
             cad.cyl(PT_REAR_FLANGE_D, PT_REAR_FLANGE_T, (x0, 0, PT_Z), (1, 0, 0)),
             cad.cyl(SPIGOT_D, REAR_PLATE_UPPER_X1 - REAR_PLATE_X[0] - 0.3 + 0.01, (x1 - 0.01, 0, 0), (1, 0, 0))]
    wx, ly0, ly1, lz0, lz1 = PT_LUG
    for x in PT_LUGS_X:
        d = ly1 - ly0
        parts.append(cad.polygon_prism([(x - wx / 2 - d, ly1), (x - wx / 2, ly0), (x + wx / 2, ly0), (x + wx / 2, ly1)],
                                       lz0, lz1))
    body = cad.fuse_all(parts)
    sx1 = x1 + REAR_PLATE_UPPER_X1 - REAR_PLATE_X[0] - 0.3
    cutters = [cad.cyl(PT_BORE, x1 - PT_FRONT_T - x0 + 1, (x0 - 1, 0, PT_Z), (1, 0, 0)),
               cad.cyl(OUTLET_D, sx1 - (x1 - PT_FRONT_T) + 2, (x1 - PT_FRONT_T - 1, 0, 0), (1, 0, 0))]
    gd, gw = SPIGOT_GROOVE
    gx = x1 + 1.5
    cutters.append(cad.cyl(SPIGOT_D + 2, gw, (gx, 0, 0), (1, 0, 0)).cut(cad.cyl(gd, gw + 2, (gx - 1, 0, 0), (1, 0, 0))))
    for y, z in PT_FLANGE_SCREWS:
        cutters.append(cad.cyl(3.4, PT_FRONT_T + 2, (x1 - PT_FRONT_T - 1, y, z), (1, 0, 0)))
    for a in PT_REAR_SCREWS_DEG:
        y, z = PT_REAR_SCREW_R * math.cos(math.radians(a)), PT_Z + PT_REAR_SCREW_R * math.sin(math.radians(a))
        cutters.append(cad.insert_hole_axis("M3", (x0, y, z), (1, 0, 0)))
    for x in PT_LUGS_X:
        cutters.append(cad.insert_hole("M3", x, PT_LUG_SCREW_Y, lz0, direction=1))
    return cad.cut_all(body, cutters)


def plunger(draw=0.0):
    """Plunger: head (O-ring groove) and the rod behind it (the spring's
    guide), with the sear's groove (catch face, ramp) and the tail collar
    for the rack's hook, at this draw."""
    from hardware.parts.common import cad
    r = rod_x(draw)
    hx0, hx1 = r["head"]
    head = cad.cyl(HEAD_D, HEAD_T, (hx0, 0, PT_Z), (1, 0, 0))
    gd, gw = HEAD_GROOVE
    gx = hx0 + (HEAD_T - gw) / 2
    head = head.cut(cad.cyl(HEAD_D + 2, gw, (gx, 0, PT_Z), (1, 0, 0)).cut(cad.cyl(gd, gw + 2, (gx - 1, 0, PT_Z), (1, 0, 0))))
    cx0, cx1 = r["collar"]
    rod = cad.cyl(ROD_D, hx0 - cx1 + 0.01, (cx1, 0, PT_Z), (1, 0, 0))
    body = cad.fuse_all([head, rod]).cut(cad.insert_hole_axis("M3", (cx1, 0, PT_Z), (1, 0, 0)))
    catch, ramp_end = groove_x(draw)
    flat_end = catch + GROOVE_FLAT
    ring = cad.cyl(ROD_D + 2, flat_end - catch, (catch, 0, PT_Z), (1, 0, 0)).cut(
        cad.cyl(GROOVE_CORE_D, flat_end - catch + 2, (catch - 1, 0, PT_Z), (1, 0, 0)))
    ramp = cad.cyl(ROD_D + 2, ramp_end - flat_end, (flat_end, 0, PT_Z), (1, 0, 0)).cut(
        cad.cone(GROOVE_CORE_D, ROD_D, ramp_end - flat_end, (flat_end, 0, PT_Z), (1, 0, 0)))
    return body.cut(ring).cut(ramp).clean()


def collar(draw=0.0):
    """Collar on the rod's tail (an M3 x 8 into the rod's insert): the
    rack's hook pushes it back. Fitted after the rod is through the cap."""
    from hardware.parts.common import cad
    cx0, cx1 = rod_x(draw)["collar"]
    return cad.cyl(COLLAR_D, cx1 - cx0, (cx0, 0, PT_Z), (1, 0, 0)).cut(
        cad.cyl(3.4, cx1 - cx0 + 2, (cx0 - 1, 0, PT_Z), (1, 0, 0)))


def cap():
    """Rear cap on the tube's rear flange: the spring's seat and the rod's
    bushing; behind it the sear housing (the sear's slot, its spring, the
    safety's slot, the lug window at +y); at +y the trigger servo's cradle
    (floor, idler-side wall with two M2 into its far-end holes, stops)."""
    from hardware.parts.common import cad
    cx0, cx1 = CAP_X
    hx0, hx1 = HOUSING_X
    t = trig_box()
    parts = [cad.cyl(PT_REAR_FLANGE_D, cx1 - cx0, (cx0, 0, PT_Z), (1, 0, 0)),
             cad.box(hx0, hx1 + 0.01, HOUSING_Y[0], HOUSING_Y[1], HOUSING_Z[0], HOUSING_Z[1]),
             # web from the cap to the trigger cradle, and the cradle
             cad.box(cx0, cx1, HOUSING_Y[1] - 0.01, t["wall_y"][1], t["floor_z"][0], 30.0),
             cad.box(t["x"][0], cx1, 20.0, t["wall_y"][1], t["floor_z"][0], t["floor_z"][1]),
             cad.box(t["x"][0], t["x"][0] + 3.0, 20.0, t["wall_y"][1], t["floor_z"][0], TRIG_SHAFT[2]),   # far-end stop
             cad.box(trig_case_x(0.0) + 0.2, trig_case_x(0.0) + 3.2, TRIG_SHAFT[1] - S.THICKNESS_OUTPUT_END / 2 + 1.0,
                     t["wall_y"][1], t["floor_z"][0], TRIG_SHAFT[2]),                                       # output-end stop
             cad.box(t["x"][0], t["wall_x1"], t["wall_y"][0], t["wall_y"][1], t["floor_z"][0],
                     TRIG_SHAFT[2] + S.WIDTH / 2 + 1.5)]
    body = cad.fuse_all(parts)
    cutters = [cad.cyl(ROD_D + 0.6, hx1 - hx0 + (cx1 - cx0) + 2, (hx0 - 1, 0, PT_Z), (1, 0, 0)),
               # sear slot
               cad.box(SEAR_X[0] - 0.2, SEAR_X[1] + 0.2, SEAR_Y[0] - 0.2, SEAR_Y[1] + 0.2, HOUSING_Z[0] - 1.0,
                       HOUSING_Z[1] - 2.0),
               # lug window in the +y wall
               cad.box(SEAR_X[0] - 0.2, SEAR_X[1] + 0.2, SEAR_Y[1], HOUSING_Y[1] + 1,
                       notch_top() + 2.5, notch_top(draw=0.0) + 7.0 + sear_release_lift() + 2.0),
               # sear spring bore (to the top) and the safety's slot through the rear wall
               cad.cyl(SEAR_SPRING["od"] + 0.5, HOUSING_Z[1] - 2.0 - (notch_top() + SEAR_ABOVE - 0.5),
                       ((SEAR_X[0] + SEAR_X[1]) / 2, SEAR_SPRING_Y, notch_top() + SEAR_ABOVE - 0.5)),
               cad.box(hx0 - 1, SEAR_X[1] + 1.0, SAFETY_Y[0] - 0.2, SAFETY_Y[1] + 0.2,
                       safety_z()[0] - 0.2, safety_z()[1] + 0.2),
               # vents
               cad.cyl(5.0, cx1 - cx0 + 2, (cx0 - 1, 0, PT_Z + 9.0), (1, 0, 0)),
               cad.cyl(5.0, cx1 - cx0 + 2, (cx0 - 1, 0, PT_Z - 9.0), (1, 0, 0))]
    for a in PT_REAR_SCREWS_DEG:
        y, z = PT_REAR_SCREW_R * math.cos(math.radians(a)), PT_Z + PT_REAR_SCREW_R * math.sin(math.radians(a))
        cutters.append(cad.cyl(3.4, cx1 - cx0 + 2, (cx0 - 1, y, z), (1, 0, 0)))
    for x, z in t["screws"]:
        cutters.append(cad.cyl(2.4, MOUNT_WALL_T + 2, (x, t["wall_y"][0] - 1, z), (0, 1, 0)))
        cutters.append(cad.cyl(4.6, 3.0, (x, t["wall_y"][1] - 2.0, z), (0, 1, 0)))
    # plug window in the cradle wall
    px0, px1 = sorted((trig_case_x(S.CONNECTOR_X[0]), trig_case_x(S.CONNECTOR_X[1])))
    cutters.append(cad.box(px0 - 0.8, px1 + 0.8, t["wall_y"][0] - 1, t["wall_y"][1] + 1,
                           TRIG_SHAFT[2] - 11.5, TRIG_SHAFT[2] + 11.5))
    return cad.cut_all(body, cutters)


def trig_box():
    """Trigger servo cradle numbers."""
    far = trig_case_x(S.LENGTH)
    wall_y0 = TRIG_SHAFT[1] + HALF_T + 0.2
    floor_top = TRIG_SHAFT[2] - S.WIDTH / 2 - 0.2
    xh = trig_case_x(S.IDLER_FACE_HOLES_X[1])
    return {"x": (far - 0.2 - 3.0, CAP_X[1]), "wall_y": (wall_y0, wall_y0 + MOUNT_WALL_T),
            "floor_z": (floor_top - MOUNT_FLOOR_T, floor_top), "wall_x1": xh + 6.0,
            "screws": [(xh, TRIG_SHAFT[2] + dz) for dz in (S.FACE_HOLE_Y, -S.FACE_HOLE_Y)]}


def sear(lift=0.0, caught=True):
    """Sear: a plate sliding in z in the housing, its notch straddling the
    rod's groove when caught; a lug at +y for the trigger's pin and a peg
    for its spring on top."""
    from hardware.parts.common import cad
    nt = notch_top(None if caught else 0.0, lift)
    x0, x1 = SEAR_X
    plate = cad.box(x0, x1, SEAR_Y[0], SEAR_Y[1], nt - SEAR_LEG, nt + SEAR_ABOVE)
    lug = cad.box(x0, x1, SEAR_Y[1] - 0.01, SEAR_LUG_Y[1], nt + 3.0, nt + 7.0)
    peg = cad.cyl(2.5, 4.0, ((x0 + x1) / 2, SEAR_SPRING_Y, nt + SEAR_ABOVE - 0.01))
    body = cad.fuse_all([plate, lug, peg])
    return body.cut(cad.box(x0 - 1, x1 + 1, -SEAR_SLOT_W / 2, SEAR_SLOT_W / 2, nt - SEAR_LEG - 1, nt)).clean()


def safety_z():
    z0 = notch_top() + SEAR_ABOVE + SAFETY_GAP
    return z0, z0 + SAFETY_T


def safety(engaged=True):
    """Safety slider: a bar in the housing's slot; pushed in (SAFE) its
    tongue sits over the caught sear, pulled back (FIRE) it is clear."""
    from hardware.parts.common import cad
    dx = 0.0 if engaged else -SAFETY_TRAVEL
    z0, z1 = safety_z()
    x1 = SEAR_X[1] + 0.8 + dx
    bar = cad.box(HOUSING_X[0] - 14.0 + dx, x1, SAFETY_Y[0], SAFETY_Y[1], z0, z1)
    grip = cad.box(HOUSING_X[0] - 14.0 + dx, HOUSING_X[0] - 10.0 + dx, SAFETY_Y[0], SAFETY_Y[1], z1 - 0.01, z1 + 6.0)
    return bar.fuse(grip).clean()


def trigger_arm(angle=0.0):
    """Trigger arm on the trigger servo's horn: hub disc (four M3 x 5 into
    the horn), arm, and the pin under the sear's lug; ``angle`` rad lifts."""
    from hardware.parts.common import cad
    x, ys, z = TRIG_SHAFT
    yf = ys - HORN_FACE                    # horn face (the arm sits on it, at -y of it)
    hub = cad.cyl(22.0, TRIG_ARM_T, (x, yf, z), (0, -1, 0))
    arm = cad.box(x, x + TRIG_ARM_R, yf - TRIG_ARM_T, yf, z - 3.5, z + 3.5)
    pin = cad.cyl(TRIG_PIN_D, yf - TRIG_ARM_T - (SEAR_LUG_Y[0] + 2.5) + 0.01,
                  (x + TRIG_ARM_R, yf - TRIG_ARM_T + 0.01, z), (0, -1, 0))
    body = cad.fuse_all([hub, arm, pin])
    for hx, hz in _horn_holes():
        body = body.cut(cad.cyl(3.4, TRIG_ARM_T + 2, (x + hx, yf + 1, z + hz), (0, -1, 0)))
    return cad.rotate(body, (0, 1, 0), math.degrees(angle) * -1.0, about=(x, 0, z)) if angle else body


def _horn_holes(angle0=45.0):
    r = S.HORN_HOLE_CIRCLE / 2
    return [(r * math.cos(math.radians(angle0 + 90 * k)), r * math.sin(math.radians(angle0 + 90 * k))) for k in range(4)]


RACK_STEP_X = (CAM_PLATE_X[0] - 12.0, CAM_PLATE_X[0] + 0.5)  # the bar's front step over the cam plate
CAM_SCREWS_X = (-5.5, 0.5)


def rack(u=0.0):
    """Priming rack bar at travel u (back is +u): teeth down over the
    pinion, the hook plate (pushes the rod's collar back) at its rear, a
    step at its front onto the cam plate (two M3 into the plate)."""
    from hardware.parts.common import cad
    from hardware.parts.turret import _gears
    y0, y1 = RACK_Y
    hx0 = HOOK_X[0]
    body = cad.box(hx0, RACK_STEP_X[0] + 0.01, y0, y1, RACK_ROOT_Z, RACK_TOP_Z).fuse(
        cad.box(RACK_STEP_X[0], RACK_STEP_X[1], y0, y1, CAM_Z[1], RACK_TOP_Z))
    hy0, hy1, hz0, hz1 = HOOK_PLATE
    hook = cad.box(hx0, HOOK_X[1], hy0, hy1, hz0, hz1)
    prof = _gears.rack_profile([tx - hx0 for tx in rack_teeth_x()], 2.0, RACK_STEP_X[0] - hx0)
    pts = [(hx0 + a, RACK_ROOT_Z + 2.0 - h) for a, h in prof]
    teeth = cad.profile_xz(pts, y0, y1)
    bar = cad.fuse_all([body, hook, teeth])
    cutters = [cad.box(hx0 - 1, HOOK_X[1] + 1, -(ROD_D / 2 + 0.4), hy1 + 1, PT_Z - (ROD_D / 2 + 0.4),
                       PT_Z + ROD_D / 2 + 0.4)]
    for x in CAM_SCREWS_X:
        cutters.append(cad.clearance_hole("M3", x, (y0 + y1) / 2, CAM_Z[1] - 1, RACK_TOP_Z + 1))
    bar = cad.cut_all(bar, cutters)
    return cad.translate(bar, (-u, 0, 0)) if u else bar


def cam(u=0.0):
    """Cam plate under the rack bar's front: the slot that moves the
    shuttle's pin (CAM_PATH), two M3 x 3 inserts for the bar."""
    from hardware.parts.common import cad
    y0, y1 = RACK_Y
    plate = cad.box(RACK_STEP_X[0], CAM_PLATE_X[1], CAM_PLATE_Y[0], CAM_PLATE_Y[1], CAM_Z[0], CAM_Z[1])
    xm = (TAB_X[0] + TAB_X[1]) / 2
    cutters = []
    for (ua, ya), (ub, yb) in zip(CAM_PATH, CAM_PATH[1:]):
        cutters.append(cad.slot(xm + ua, PIN_Y_FIRE + ya, xm + ub, PIN_Y_FIRE + yb, SLOT_W, CAM_Z[0] - 1, CAM_Z[1] + 1))
    for x in CAM_SCREWS_X:
        cutters.append(cad.cyl(4.0, CAM_Z[1] - CAM_Z[0] + 2, (x, (y0 + y1) / 2, CAM_Z[0] - 1)))   # inserts pressed flush
    plate = cad.cut_all(plate, cutters)
    return cad.translate(plate, (-u, 0, 0)) if u else plate


def pinion(u=0.0):
    """Pinion on the actuator servo's horn (four M3 x 5 through 3 mm of
    gear), turned with the rack's travel u."""
    import cadquery as cq

    from hardware.parts.common import cad
    from hardware.parts.turret import _gears
    half_step = 180.0 / G.PINION_TEETH
    # local: axis z from the horn face, +y local -> -z placed: the gap at the
    # top (placed +z) is at local -y
    pts = _gears.pinion_outline(phase_deg=-90.0 - half_step)
    gear = cq.Workplane("XY").polyline(pts).close().extrude(G.FACE).val()
    cb = (6.0, 5.0)
    for hx, hy in _horn_holes():
        gear = gear.cut(cad.cyl(3.4, G.FACE + 2, (hx, hy, -1))).cut(cad.cyl(cb[0], cb[1] + 1, (hx, hy, G.FACE - cb[1])))
    gear = cad.rotate(gear, "X", -90)               # local z -> +y, local y -> -z
    x, ys, z = ACT_SHAFT
    gear = cad.translate(gear, (x, ys + HORN_FACE, z))
    # rack back (+u, toward -x): the top of the pinion moves -x
    return cad.rotate(gear, "Y", -math.degrees(u / G.PINION_R), about=(x, 0, z)) if u else gear


def servo_mount():
    """Actuator servo mount: a bar under the tube's two lugs, a shoe over
    the rack and guide lips at the pinion, the idler-side wall (two M2 into
    the servo's far-end holes, a window for its plugs) and the floor with
    end stops under the servo."""
    from hardware.parts.common import cad
    x, ys, z = ACT_SHAFT
    far, out = act_case_x(S.LENGTH), act_case_x(0.0)
    wy1 = ys - HALF_T - 0.2
    wy0 = wy1 - MOUNT_WALL_T
    fz1 = z - S.WIDTH / 2 - 0.2
    fz0 = fz1 - MOUNT_FLOOR_T
    bz0, bz1 = MOUNT_BAR_Z
    bx0, bx1 = PT_LUGS_X[0] - 8.0, PT_LUGS_X[1] + 8.0
    y0, y1 = RACK_Y
    parts = [cad.box(bx0, bx1, wy0, MOUNT_BAR_Y1, bz0, bz1),                                # bar
             cad.box(x - 33.0, x + 9.0, y0, y1 - 0.6, RACK_TOP_Z + 0.25, bz0 + 0.01),       # shoe over the rack
             cad.box(x - 33.0, x + 9.0, y0 - 2.25, y0 - 0.25, RACK_ROOT_Z + 2.0, bz0 + 0.01),    # -y lip
             cad.box(far - 3.2, bx1, wy0, wy1, fz0, bz0 + 0.01),                             # idler-side wall
             cad.box(far - 3.2, out + 3.2, wy0, ys + HALF_T + 0.2, fz0, fz1),                # floor
             cad.box(far - 3.2, far - 0.2, wy0, ys + HALF_T - 2.0, fz1 - 0.01, z),           # far-end stop
             cad.box(out + 0.2, out + 3.2, wy0, ys + S.THICKNESS_OUTPUT_END / 2 - 1.0, fz1 - 0.01, z)]   # output-end stop
    body = cad.fuse_all(parts)
    cutters = []
    for lx in PT_LUGS_X:
        cutters.append(cad.clearance_hole("M3", lx, PT_LUG_SCREW_Y, bz0 - 1, bz1 + 1))
    xh = act_case_x(S.IDLER_FACE_HOLES_X[1])
    for dz in (S.FACE_HOLE_Y, -S.FACE_HOLE_Y):
        cutters.append(cad.cyl(2.4, MOUNT_WALL_T + 2, (xh, wy1 + 1, z + dz), (0, -1, 0)))
        cutters.append(cad.cyl(4.6, 3.0, (xh, wy0 - 1, z + dz), (0, 1, 0)))
    px0, px1 = sorted((act_case_x(S.CONNECTOR_X[0]), act_case_x(S.CONNECTOR_X[1])))
    cutters.append(cad.box(px0 - 0.8, px1 + 0.8, wy0 - 1, wy1 + 1, z - 11.5, z + 11.5))
    return cad.cut_all(body, cutters)


def barrel_clamp():
    """Barrel clamp in front of the front wall: a split collar around the
    barrel (M3 pinch screw across the split) on two M3 into the wall."""
    from hardware.parts.common import cad
    x0, x1 = CLAMP_X
    body = cad.box(x0, x1, -CLAMP_HALF_W, CLAMP_HALF_W, BREECH_Z0, 15.0)
    xm = (x0 + x1) / 2
    cutters = [cad.cyl(BARREL["od"] + 0.15, x1 - x0 + 2, (x0 - 1, 0, 0), (1, 0, 0)),
               cad.box(x0 - 1, x1 + 1, -0.75, 0.75, 0, 16.0),
               cad.cyl(3.4, 2 * CLAMP_HALF_W + 2, (xm, -CLAMP_HALF_W - 1, PINCH_Z), (0, 1, 0)),
               cad.cyl(6.5, 4.0, (xm, -CLAMP_HALF_W - 0.01, PINCH_Z), (0, 1, 0)),
               cad.cyl(6.5, 4.0, (xm, CLAMP_HALF_W - 4.0 + 0.01, PINCH_Z), (0, 1, 0))]
    for y, z in FRONT_SCREWS:
        cutters.append(cad.cyl(3.4, x1 - x0 + 2, (x0 - 1, y, z), (1, 0, 0)))
        cutters.append(cad.cyl(6.5, 3.2, (x1 - 3.2, y, z), (1, 0, 0)))
    return cad.cut_all(body, cutters)


def muzzle_cap():
    """Muzzle cap, for transport: a cup over the barrel's end with a tether
    eye."""
    from hardware.parts.common import cad
    x0 = MUZZLE_X - CAP_DEPTH
    od = BARREL["od"] + 3.2
    body = cad.cyl(od, CAP_DEPTH + 2.0, (x0, 0, 0), (1, 0, 0)).fuse(
        cad.box(x0 + 2, x0 + 8, -1.5, 1.5, od / 2 - 1, od / 2 + 5))
    return cad.cut_all(body, [cad.cyl(BARREL["od"] + 0.25, CAP_DEPTH + 0.01, (x0 - 0.01, 0, 0), (1, 0, 0)),
                              cad.cyl(2.5, 5, (x0 + 5, -2.5, od / 2 + 2.5), (0, 1, 0))])


# --------------------------------------------------------------------- purchased models

def barrel():
    from hardware.parts.common import cad
    x0 = CHAMBER[1] + BARREL_STOP
    return cad.cyl(BARREL["od"], BARREL["length"], (x0, 0, 0), (1, 0, 0)).cut(
        cad.cyl(BARREL["id"], BARREL["length"] + 2, (x0 - 1, 0, 0), (1, 0, 0)))


def spring(draw=0.0):
    """The plunger spring as a tube of its outline (for interference)."""
    from hardware.parts.common import cad
    x0 = CAP_X[1]
    x1 = rod_x(draw)["head"][0]
    od, d = SPRING["od"], SPRING["wire_d"]
    return cad.cyl(od, x1 - x0 - 0.2, (x0 + 0.1, 0, PT_Z), (1, 0, 0)).cut(
        cad.cyl(od - 2 * d, x1 - x0 + 2, (x0 - 1, 0, PT_Z), (1, 0, 0)))


def darts(n=CLIP_DARTS, chambered=True, u=0.0):
    """Darts in the clip (stacked from the shuttle's top) and one in the
    chamber: [(name, shape)]."""
    from hardware.parts.common import cad
    x0 = (OUTLET[0] + OUTLET[1]) / 2 - DART_LEN / 2
    dy = shuttle_y(u)
    at_load = dy >= TRAVEL - 0.01
    base = DART_D + 0.2 if at_load and chambered else SHUTTLE_TOP + DART_D / 2 + 0.1
    out = []
    for k in range(n):
        out.append((f"dart_clip_{k}", cad.cyl(DART_D - 0.2, DART_LEN, (x0, TRAVEL, base + k * DART_D), (1, 0, 0))))
    if chambered:
        out.append(("dart_chamber", cad.cyl(DART_D - 0.2, DART_LEN, (x0, dy, 0.1), (1, 0, 0))))
    return out


# --------------------------------------------------------------------- parts list and assembly

PRINTED = ("breech", "shuttle", "clip", "follower", "clip_cap", "tube", "plunger", "collar", "cap", "sear", "safety",
           "trigger_arm", "rack", "cam", "pinion", "servo_mount", "barrel_clamp", "muzzle_cap")


def models(u=0.0, cocked=False, trigger=0.0, safe=False, darts_in_clip=CLIP_DARTS, chambered=True,
           pedestal=None, st3215=None, screw=None):
    """Everything of the launcher in the launcher frame, [(name, printed
    stem or None, shape)], at rack travel u (mm back; above ACT_CATCH the
    rack's hook holds the rod), ``cocked`` (below ACT_CATCH the sear holds
    the plunger at DRAW_CAUGHT), trigger angle (rad), safety, clip state.
    ``st3215`` is the servo model of hardware/parts/turret/assembly.py and
    ``screw`` hardware/parts/racecarneo/_checks.screw (both optional)."""
    from hardware.parts.common import cad
    if u >= ACT_CATCH or not cocked:
        rod_draw = min(draw_at(u), DRAW_PRIME)
    else:
        rod_draw = DRAW_CAUGHT
    catch, _ = groove_x(rod_draw)
    in_groove = catch <= SEAR_X[0] and catch + GROOVE_FLAT >= SEAR_X[1]
    rise = TRIG_ARM_R * math.sin(trigger) - 2.5
    lift = max(0.0, rise) if in_groove else max(0.0, rise - sear_release_lift())
    if safe:
        lift = min(lift, SAFETY_GAP if in_groove else 0.0)
    out = [("launcher_breech", "breech", breech(pedestal)),
           ("launcher_shuttle", "shuttle", shuttle(u)),
           ("launcher_clip", "clip", clip()),
           ("launcher_follower", "follower", follower(darts_in_clip)),
           ("launcher_clip_cap", "clip_cap", clip_cap()),
           ("launcher_tube", "tube", tube()),
           ("launcher_plunger", "plunger", plunger(rod_draw)),
           ("launcher_collar", "collar", collar(rod_draw)),
           ("launcher_cap", "cap", cap()),
           ("launcher_sear", "sear", sear(lift, in_groove)),
           ("launcher_safety", "safety", safety(safe)),
           ("launcher_trigger_arm", "trigger_arm", trigger_arm(trigger)),
           ("launcher_rack", "rack", rack(u)),
           ("launcher_cam", "cam", cam(u)),
           ("launcher_cam_pin", None, cam_pin(u)),
           ("launcher_pinion", "pinion", pinion(u)),
           ("launcher_servo_mount", "servo_mount", servo_mount()),
           ("launcher_barrel_clamp", "barrel_clamp", barrel_clamp()),
           ("launcher_barrel", None, barrel()),
           ("launcher_spring", None, spring(rod_draw))]
    out += [(f"launcher_{n}", None, s) for n, s in darts(darts_in_clip, chambered, u)]
    if st3215:
        turn = -math.degrees(u / G.PINION_R)
        x, ys, z = ACT_SHAFT
        for n, s in st3215([("X", 90), ("Z", 180)], (x, ys, z), 45.0, idler_horn=False):
            out.append((f"act_servo_{n}", None, cad.rotate(s, "Y", turn, about=(x, 0, z)) if n == "horn_out" and u else s))
        tx, tys, tz = TRIG_SHAFT
        for n, s in st3215([("X", 90), ("Y", 180)], (tx, tys, tz), 45.0, idler_horn=False):
            out.append((f"trig_servo_{n}", None,
                        cad.rotate(s, (0, 1, 0), -math.degrees(trigger), about=(tx, 0, tz)) if n == "horn_out" and trigger else s))
    if screw:
        out += [(n, None, s) for n, s in screws(u, trigger, screw, pedestal, rod_draw)]
    return out


def screws(u, trigger, screw, pedestal=None, rod_draw=0.0):
    """Screw models through their holes: [(name, shape)]."""
    from hardware.parts.common import cad
    out = [("launcher_collar_screw", screw((rod_x(rod_draw)["collar"][0], 0.0, PT_Z), (1, 0, 0), 8.0, "M3"))]
    for k, (x, y) in enumerate(COVER_SCREWS):
        out.append((f"launcher_cover_screw_{k}", screw((x, y, COVER_TOP), (0, 0, -1), 10.0, "M3")))
    for k, (y, z) in enumerate(PT_FLANGE_SCREWS):
        n = 10.0 if z < ROOF else 8.0
        out.append((f"launcher_flange_screw_{k}", screw((PT_X[1] - PT_FRONT_T, y, z), (1, 0, 0), n, "M3")))
    for k, a in enumerate(PT_REAR_SCREWS_DEG):
        y, z = PT_REAR_SCREW_R * math.cos(math.radians(a)), PT_Z + PT_REAR_SCREW_R * math.sin(math.radians(a))
        out.append((f"launcher_cap_screw_{k}", screw((CAP_X[0], y, z), (1, 0, 0), 12.0, "M3")))
    for k, lx in enumerate(PT_LUGS_X):
        out.append((f"launcher_mount_screw_{k}", screw((lx, PT_LUG_SCREW_Y, MOUNT_BAR_Z[0]), (0, 0, 1), 10.0, "M3")))
    for k, x in enumerate(CAM_SCREWS_X):
        out.append((f"launcher_cam_screw_{k}", cad.translate(screw((x, (RACK_Y[0] + RACK_Y[1]) / 2, RACK_TOP_Z), (0, 0, -1),
                                                                   6.0, "M3"), (-u, 0, 0))))
    for k, (y, z) in enumerate(FRONT_SCREWS):
        out.append((f"launcher_clamp_screw_{k}", screw((CLAMP_X[1] - 3.2, y, z), (-1, 0, 0), 14.0, "M3")))
    out.append(("launcher_pinch_screw", screw(((CLAMP_X[0] + CLAMP_X[1]) / 2, -CLAMP_HALF_W + 4.0, PINCH_Z), (0, 1, 0), 25.0, "M3")))
    # actuator servo: two M2 into its idler face; pinion: four M3 x 5
    x, ys, z = ACT_SHAFT
    wy0 = ys - HALF_T - 0.2 - MOUNT_WALL_T
    xh = act_case_x(S.IDLER_FACE_HOLES_X[1])
    for dz in (S.FACE_HOLE_Y, -S.FACE_HOLE_Y):
        out.append((f"launcher_act_wall_screw_{dz:g}", screw((xh, wy0 + 2.0, z + dz), (0, 1, 0), 6.0, "M2")))
    face = ys + HORN_FACE + G.FACE - 5.0
    turn = -math.degrees(u / G.PINION_R)
    for hx, hy in _horn_holes():
        s = screw((x + hx, face, z - hy), (0, -1, 0), 5.0, "M3")
        out.append((f"launcher_pinion_screw_{hx:.0f}_{hy:.0f}", cad.rotate(s, "Y", turn, about=(x, 0, z)) if u else s))
    t = trig_box()
    for xh, zh in t["screws"]:
        out.append((f"launcher_trig_wall_screw_{zh:.0f}", screw((xh, t["wall_y"][1] - 2.0, zh), (0, -1, 0), 6.0, "M2")))
    tx, tys, tz = TRIG_SHAFT
    yf = tys - HORN_FACE
    for hx, hz in _horn_holes():
        s = screw((tx + hx, yf - TRIG_ARM_T, tz + hz), (0, 1, 0), 5.0, "M3")
        out.append((f"launcher_arm_screw_{hx:.0f}_{hz:.0f}",
                    cad.rotate(s, (0, 1, 0), -math.degrees(trigger), about=(tx, 0, tz)) if trigger else s))
    if pedestal:
        for k, (px, py) in enumerate(pedestal[5]):
            out.append((f"launcher_pad_screw_{k}", screw((px, py, pedestal[4] - 4.0), (0, 0, 1), 10.0, "M3")))
    return out
