"""Datasheet footprints of the airplane's purchased parts (helper, not a
part).

Same conventions as hardware/parts/common/footprints.py: millimetres, every
class cites its source and lists in ``UNVERIFIED`` the attributes the cited
document does not state; a printed part that uses one sets ``fit_test``.
They live here, not in common/footprints.py, only because that shared file
was frozen while these platforms were added; ``ALL`` lists them so the
footprint test can cover them when they move. The Raspberry Pi 5,
Navigator, OAK-1, GPS and damper balls come from common/footprints.py.
"""


class SkywalkerX8:
    """Skywalker X8 EPO flying wing (kit): the reference airframe."""
    SOURCE = ("Skywalker X8 kit listings: RMRC "
              "(https://www.readymaderc.com/products/details/skywalker-model-x8-wing) and UAV Garage "
              "(https://uavgarage.com/shop/skywalker-x8-2122mm-uav-fixed-wing-drone/): 2120-2122 mm span, "
              "790 mm fuselage, 80 dm2, flying weight 2500-3000 g, payload 1000-2000 g, CG 430-440 mm from "
              "the nose, 65-70 km/h, catapult or bungee launch; RMRC: 6S 5000-6000 mAh, 12x6E prop, "
              "60-100 A ESC, 2 x 17 g servos")
    SPAN = 2120.0
    FUSELAGE_LENGTH = 790.0
    WING_AREA_DM2 = 80.0
    FLYING_WEIGHT_G = (2500.0, 3000.0)
    PAYLOAD_G = (1000.0, 2000.0)
    CG_FROM_NOSE = (430.0, 440.0)
    AIRSPEED_KMH = (65.0, 70.0)
    SERVOS = 2
    SERVO_CLASS_G = 17.0
    # Not published by Skywalker or the resellers: the payload bay's inside
    # dimensions and the foam thickness of the bay floor (belly). These are
    # the envelope the printed parts are designed for (a requirement), to be
    # checked in the real bay before anything is glued.
    BAY_FLOOR_WIDTH = 110.0          # flat floor width the deck, frame and tray need
    BAY_HEIGHT = 60.0                # floor to hatch: the Pi + Navigator stack is ~50 mm
    BELLY_T = 10.0                   # design belly (floor) foam thickness
    BELLY_T_MAX = 14.0               # the camera cut-out keeps the view clear up to this
    UNVERIFIED = ("BAY_FLOOR_WIDTH", "BAY_HEIGHT", "BELLY_T", "BELLY_T_MAX")


class Lipo6SEnvelope:
    """Design envelope (a requirement, not a datasheet) for the 6S pack."""
    SOURCE = ("Requirement of this design: 6S 5000-6000 mAh LiPo (the X8 listing's recommendation), "
              "at most 170 x 52 x 62 mm, about 900 g")
    MAX_SIZE = (170.0, 52.0, 62.0)
    NOMINAL_SIZE = (160.0, 50.0, 58.0)
    MASS_G = 900.0
    UNVERIFIED = ()


class HolybroAirspeedMS4525DO:
    """Holybro Digital Air Speed Sensor MS4525DO with the PT60 pitot tube."""
    SOURCE = ("Holybro Digital Air Speed Sensor - MS4525DO "
              "(https://holybro.com/products/digital-air-speed-sensor-ms4525do): module 17 x 23 x 13 mm, "
              "4.1 g, 4.75-5.25 V, I2C, GH 4P cable, PT40/PT60 pitot tube with 40/100 cm tubing")
    MODULE = (17.0, 23.0, 13.0)
    MASS_G = 4.1
    SUPPLY_V = (4.75, 5.25)
    # PT60 pitot: reseller figures (3DXR, mybotshop), not a Holybro drawing:
    # 91.5 x 28 x 30 mm overall, 76 mm tube ahead of a 28 x 30 mm base.
    PT60_OVERALL = (91.5, 28.0, 30.0)
    PT60_TUBE_LENGTH = 76.0
    PT60_MASS_G = 12.0               # estimate
    UNVERIFIED = ("PT60_OVERALL", "PT60_TUBE_LENGTH", "PT60_MASS_G")


class BlueRoboticsPowerSense:
    """Blue Robotics Power Sense Module (senses only, no 5 V output)."""
    SOURCE = ("Blue Robotics Power Sense Module "
              "(https://bluerobotics.com/store/comm-control-power/control/psm-asm-r2-rp/): 25.2 V (6S) max, "
              "100 A non-continuous, 24 x 19 x 7 mm, 5.5 mm bullet in, #6 spades out")
    SIZE = (24.0, 19.0, 7.0)
    MAX_V = 25.2
    MAX_A = 100.0
    MASS_G = 15.0                    # estimate with leads
    UNVERIFIED = ("MASS_G",)


class BlueRobotics5V6A:
    """Blue Robotics 5 V 6 A power supply (Navigator main 5 V)."""
    SOURCE = ("Blue Robotics 5V 6A Power Supply "
              "(https://bluerobotics.com/store/comm-control-power/control/bec-5v6a-r1/): 7-26 V in, "
              "5 V 6 A out, 37 x 18 x 6 mm")
    SIZE = (37.0, 18.0, 6.0)
    INPUT_V = (7.0, 26.0)
    MASS_G = 15.0                    # estimate with leads
    UNVERIFIED = ("MASS_G",)


class RpSmaBulkhead:
    """RP-SMA bulkhead jack (antenna extension), SMA coupling thread."""
    SOURCE = ("IEC 61169-15 / MIL-STD-348 SMA interface: coupling thread 1/4\"-36 UNS-2 "
              "(6.35 mm major diameter); RP-SMA uses the same body and thread")
    THREAD_MAJOR = 6.35
    HOLE_D = 6.6                     # clearance for the thread, printed hole comes out small
    # Panel thickness the bulkhead's thread takes is not in the standard:
    # common extension cables take panels up to about 5 mm.
    MAX_PANEL = 5.0
    NUT_AF = 8.0
    UNVERIFIED = ("MAX_PANEL", "NUT_AF")


class ReceiverAntennaTube:
    """Generic RC receiver antenna (coax lead in a plastic tube, e.g. an
    ExpressLRS or FrSky dipole)."""
    SOURCE = "No manufacturer drawing: typical receiver antenna tubes are 2.5-4 mm in diameter"
    TUBE_D = (2.5, 4.0)
    UNVERIFIED = ("TUBE_D",)


ALL = (SkywalkerX8, Lipo6SEnvelope, HolybroAirspeedMS4525DO, BlueRoboticsPowerSense, BlueRobotics5V6A,
       RpSmaBulkhead, ReceiverAntennaTube)
