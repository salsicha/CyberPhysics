"""Spur gear and rack outlines for the launcher's priming drive (helper,
not a part).

Pure Python. Standard involute teeth (20 deg pressure angle) with stub
addenda (0.8 module, so the tooth tips stay over 1.2 mm wide in FDM) and
0.1 mm of backlash per flank; the rack is the basic rack of the same
module, so the two mesh at the pinion's pitch radius.
"""
import math

MODULE = 1.5                        # 4.71 mm pitch: teeth 5-6 lines wide at the pitch line
PINION_TEETH = 18                   # no undercut at 20 deg (17 is the limit)
PINION_R = MODULE * PINION_TEETH / 2   # 13.5 mm: 84.8 mm of rack per turn
PRESSURE_ANGLE_DEG = 20.0
ADDENDUM = 0.8 * MODULE
DEDENDUM = 1.25 * MODULE
BACKLASH = 0.1
FACE = 8.0


class F:                            # the names the outlines below use
    PRESSURE_ANGLE_DEG = PRESSURE_ANGLE_DEG
    GEAR_ADDENDUM = ADDENDUM
    GEAR_DEDENDUM = DEDENDUM
    GEAR_BACKLASH = BACKLASH
    PINION_TEETH = PINION_TEETH
    GEAR_MODULE = MODULE


def _inv(a):
    return math.tan(a) - a


def pinion_outline(teeth=F.PINION_TEETH, module=F.GEAR_MODULE, phase_deg=0.0, flank_points=8):
    """Closed (x, y) outline of a spur gear centred on the origin, one tooth
    centred at ``phase_deg``. Below the base circle the flanks are radial."""
    alpha = math.radians(F.PRESSURE_ANGLE_DEG)
    r = module * teeth / 2
    rb = r * math.cos(alpha)
    ra = r + F.GEAR_ADDENDUM
    rf = r - F.GEAR_DEDENDUM
    psi = math.pi / (2 * teeth) - F.GEAR_BACKLASH / r     # half tooth angle at the pitch circle

    def half(rho):
        rho = max(rho, rb)
        return psi + _inv(alpha) - _inv(math.acos(rb / rho))

    r0 = max(rb, rf)
    radii = [r0 + (ra - r0) * k / flank_points for k in range(flank_points + 1)]
    pts = []
    step = 2 * math.pi / teeth
    for k in range(teeth):
        c = math.radians(phase_deg) + k * step
        flank = []
        if rf < rb:
            flank.append((rf, half(rb)))
        flank += [(rho, half(rho)) for rho in radii]
        for rho, h in flank:                               # leading flank, root to tip
            pts.append((rho * math.cos(c - h), rho * math.sin(c - h)))
        for j in (1, 2):                                   # tip land
            a = c - half(ra) + 2 * half(ra) * j / 3
            pts.append((ra * math.cos(a), ra * math.sin(a)))
        for rho, h in reversed(flank):                     # trailing flank, tip to root
            pts.append((rho * math.cos(c + h), rho * math.sin(c + h)))
        a0 = c + half(rf)
        a1 = c + step - half(rf)
        for j in (1, 2):                                   # root arc
            a = a0 + (a1 - a0) * j / 3
            pts.append((rf * math.cos(a), rf * math.sin(a)))
    return pts


def rack_profile(u_teeth, root_h, u_len, module=F.GEAR_MODULE):
    """Closed (u, h) outline of the rack's tooth band: from h = 0 (the band's
    underside) to the roots at ``root_h`` and the teeth above them, tooth
    centres at ``u_teeth`` along u = 0..u_len."""
    alpha = math.radians(F.PRESSURE_ANGLE_DEG)
    p = math.pi * module
    w_root = p / 4 + F.GEAR_DEDENDUM * math.tan(alpha) - F.GEAR_BACKLASH
    w_tip = p / 4 - F.GEAR_ADDENDUM * math.tan(alpha) - F.GEAR_BACKLASH
    tip_h = root_h + F.GEAR_DEDENDUM + F.GEAR_ADDENDUM
    pts = [(0.0, 0.0), (0.0, root_h)]
    for u in sorted(u_teeth):
        pts += [(u - w_root, root_h), (u - w_tip, tip_h), (u + w_tip, tip_h), (u + w_root, root_h)]
    pts += [(u_len, root_h), (u_len, 0.0)]
    return pts
