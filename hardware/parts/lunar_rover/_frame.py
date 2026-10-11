"""Lunar rover layout: the numbers every lunar rover part shares (helper,
not a part).

Pure Python. Rover frame FLU in millimetres: x forward, y left, z up;
origin on the ground under the body centre (the simulator's
``lunar_rover/base_link`` is not tied to a body point, so the ground is
used). Wheel-local frame: u along the wheel axle, outward, u = 0 at the
front face of the actuator's boss (the output flange); for the right
wheels the local frame is turned 180 deg about z.

The rover matches the simulator (systems/lunar_rover/config/simulation.yaml):
six wheels of 0.18 m radius, skid steer, 0.38 m/s, a 28 V class 650 Wh
battery. Rocker-bogie with a differential: the front wheel on the rocker,
middle and rear on the bogie, pivots placed so the three wheels carry
equal loads.
"""
from hardware.parts.lunar_rover._footprints import CubeMarsAK70_10 as AK

SIM_WHEEL_RADIUS = 180.0          # simulation.yaml wheel_radius_m 0.18
SIM_MAX_SLOPE_DEG = 21.0          # lunar_rover_sim.py: 2 + 9.5 + 5 + 4.5 deg at most
SIM_MAX_SPEED = 0.38              # m/s
SIM_BATTERY_WH = 650.0

# ----------------------------------------------------------------- wheels
WHEEL_X = (400.0, 0.0, -400.0)    # front, middle, rear
AXLE_Z = SIM_WHEEL_RADIUS
OUTPUT_Y = 221.0                  # |y| of the actuator's output face
GROUSER_TIP_R = SIM_WHEEL_RADIUS  # the rolling radius on firm ground
RIM_OUTER_R = 168.0
RIM_T = 3.0
RIM_U = (4.0, 124.0)              # rim edges, wheel-local u
WEB_U = (14.0, 20.0)              # segment web, bolted to the hub's outboard face
LIP_U = (119.0, 124.0)            # outboard stiffening lip
LIP_R = (156.0, RIM_OUTER_R - RIM_T + 0.5)
WEB_R = (98.0, RIM_OUTER_R - RIM_T + 0.5)
SEGMENTS = 8                      # per wheel: 45 deg each, prints on its end wall without supports
SEGMENT_DEG = 360.0 / SEGMENTS
END_WALL_T = 4.0
END_WALL_R = (152.0, RIM_OUTER_R - RIM_T + 0.5)
WEB_INNER_R = 117.0               # web: a solid ring to here (the hub bolts), then spokes
WEB_OUTER_R = 152.0               # and a solid band from here to the rim
SPOKE_W = 18.0
GROUSERS_PER_SEGMENT = 2
GROUSER_BASE_W = 12.0
GROUSER_TIP_W = 6.0
HUB_R = 113.0                     # hub disc radius (226 mm, fits the bed with a brim)
HUB_T = 14.0                      # hub disc, u = 0 .. 14
HUB_BOLT_R = 105.0                # segment web to hub: M4 into inserts in the hub
WHEEL_MID_U = (RIM_U[0] + RIM_U[1]) / 2   # 64
WHEEL_MID_Y = OUTPUT_Y + WHEEL_MID_U      # 285

# ----------------------------------------------------------------- actuator and mount
ACT_FACE_U = -AK.FRONT_BOSS[1]            # -11.2: body front face (the 8 x M3 ring)
ACT_REAR_U = -AK.LENGTH                   # -50.25
MOUNT_RING_T = 6.0                        # mount's front ring, u = -11.2 .. -5.2
CUP_WALL = 5.0
CUP_BORE = AK.DIAMETER + 0.6
CUP_REAR_U = ACT_REAR_U - 3.0             # the cup runs 3 mm past the actuator's rear
LEG_U = -28.0                             # leg centre, wheel-local u
LEG_Y = OUTPUT_Y + LEG_U                  # 193: rocker and bogie plane
LEG_SOCKET_Z = (AXLE_Z + 60.0, AXLE_Z + 105.0)   # the leg's end sits on the cup at the first, socket to the second

# ----------------------------------------------------------------- structure (2040 extrusions, purchased)
BEAM_H = 40.0
ROCKER_BEAM_Z = (400.0, 440.0)
ROCKER_BEAM_X = (-220.0, 420.0)
BOGIE_BEAM_Z = (310.0, 350.0)
BOGIE_BEAM_X = (-420.0, 20.0)
ROCKER_PIVOT = (0.0, 420.0)               # (x, z): body to rocker, equal wheel loads
BOGIE_PIVOT = (-200.0, 375.0)             # (x, z): rocker to bogie, half way between middle and rear
BODY_X = (-300.0, 300.0)
BODY_HALF_W = 150.0
BODY_Z = (260.0, 460.0)
DIFF_LEVER_X = 100.0                      # rocker-side rod ends, ahead of the rocker pivot
DIFF_BAR_Z = 230.0                        # differential bar (2020) centre, under the body on a pillow block
DIFF_CROSS_RAIL_X = (90.0, 110.0)         # extra 2040 cross rail under the floor that carries the pillow block

# ----------------------------------------------------------------- body (2040 frame, purchased)
RAIL = (20.0, 40.0)                       # body rails: 40 mm upright
FLOOR_Z = BODY_Z[0] + RAIL[1]             # 300: floor panel (3 mm ACP) on the bottom rails
FLOOR_T = 3.0
TRAY_FEET = 8.0                           # trays stand on feet (screw heads underneath)
COMPUTE_TRAY_X = (140.0, 270.0)
POWER_TRAY_X = (-270.0, -140.0)
TRAY_HALF_W = 125.0
BATTERY_X = 0.0                           # pack centre

# ----------------------------------------------------------------- sensors and top deck
MAST_XY = (280.0, 140.0)                  # front 2020 mast on the left top side rail (clear of the hazard camera)
MAST_LENGTH = 400.0
REAR_MAST_XY = (-280.0, -140.0)           # rear 2020 mast on the right top side rail: GNSS antenna above the panel
REAR_MAST_LENGTH = 200.0
MAST_FOOT_FLOOR = 6.0                     # mast bottom above the rail's top face
HAZCAM_PITCH_DEG = 20.0                   # D455 looks 20 deg down
HAZCAM_Z = 440.0                          # D455 centre height
HAZCAM_FORWARD = 45.0                     # D455 back face ahead of the body's front face
TERRAIN_CAM_X = BODY_X[1] - RAIL[0] / 2   # OAK-1 under the front bottom cross rail, looking down
PANEL_Z = 540.0                           # bottom of the solar panel frame, on the four posts (over the differential)
PANEL_XY = (0.0, 0.0)                     # panel centre: clear of both mast feet
PANEL_SIZE = (520.0, 400.0)
PANEL_CLAMPS = tuple((x, s * (BODY_HALF_W - RAIL[0] / 2)) for x in (-180.0, 180.0) for s in (1, -1))
ESTOP_X = BODY_X[0] - 30.0                # e-stop box behind the body, button up


def wheel_positions():
    """[(name, x, side)] for the six wheels; side +1 left, -1 right."""
    names = ("front", "middle", "rear")
    return [(f"{n}_{'left' if s > 0 else 'right'}", x, s) for n, x in zip(names, WHEEL_X) for s in (1, -1)]
