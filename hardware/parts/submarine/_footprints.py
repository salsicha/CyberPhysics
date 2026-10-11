"""Datasheet footprints of the submarine's purchased parts (helper, not a
part).

Same conventions as hardware/parts/common/footprints.py: millimetres, every
class cites its source and lists in ``UNVERIFIED`` the attributes the cited
document does not state; a printed part that uses one sets ``fit_test``.
They live here, not in common/footprints.py, only because that shared file
was frozen while these platforms were added; ``ALL`` lists them so the
footprint test can cover them when they move.
"""


class BlueROV2:
    """Blue Robotics BlueROV2 (standard 6-thruster vectored frame)."""
    SOURCE = ("Blue Robotics BlueROV2 (https://bluerobotics.com/store/rov/bluerov2/): 457 x 338 x 254 mm, "
              "11-12 kg in air with ballast and battery, acrylic enclosures 100 m / aluminium 300 m, 6 x T200 "
              "(4 vectored, 2 vertical), 4\" electronics and 3\" battery enclosures, Raspberry Pi 4 (2 GB) + "
              "Navigator, Bar30, low-light HD USB camera (110 deg horizontal), 9 x 200 g stainless ballast "
              "weights, R-3318 foam, payload 1.2-1.4 kg; BlueROV2 Frame "
              "(https://bluerobotics.com/store/rov/bluerov2-components-spares/brov2-asm-frame/): bottom panel "
              "1/2 in HDPE, side panels 3/8 in HDPE, M5 x 20 frame screws and M5 slot nuts; BlueROV2 assembly "
              "guide (https://bluerobotics.com/learn/bluerov2-assembly/): ballast under the bottom panel, "
              "accessory holes drilled in the panels")
    LENGTH = 457.0
    WIDTH = 338.0
    HEIGHT = 254.0
    MASS_KG = (11.0, 12.0)
    DEPTH_RATING_M = (("acrylic", 100.0), ("aluminium", 300.0))
    BOTTOM_PANEL_T = 12.7            # 1/2 in HDPE
    SIDE_PANEL_T = 9.525             # 3/8 in HDPE
    PANEL_SCREW = "M5"
    BALLAST_WEIGHT_G = 200.0         # stainless steel, each
    PAYLOAD_KG = (1.2, 1.4)
    # Not dimensioned in the documents: the bottom panel's outline and what
    # already hangs under it (ballast weights). The parts here assume a
    # clear 300 x 200 mm area under it; check before drilling.
    BOTTOM_CLEAR = (300.0, 200.0)
    UNVERIFIED = ("BOTTOM_CLEAR",)


class WaterLinkedDVLA50:
    """Water Linked DVL A50 Doppler velocity log."""
    SOURCE = ("Water Linked DVL A50 datasheet WL-21035-4 v5 (https://www.waterlinked.com/datasheets/dvl-a50): "
              "diameter 66 mm, height 25 mm, 0.17 kg in air, 0.105 kg in water, 300 or 600 m, 4-beam convex "
              "Janus array at 22.5 deg, 5 cm-50 m altitude, 10-30 V, 4 W, PEEK housing and 316 back plate, "
              "3 m cable; half-power beam width 4.4 deg (https://docs.waterlinked.com/dvl/dvl-a50/)")
    DIAMETER = 66.0
    HEIGHT = 25.0
    MASS_AIR_G = 170.0
    MASS_WATER_G = 105.0
    DEPTH_RATING_M = (300.0, 600.0)
    BEAM_TILT_DEG = 22.5
    BEAM_WIDTH_DEG = 4.4
    MIN_ALTITUDE_MM = 50.0
    CABLE_D = 6.8
    # Not in the datasheet: where the cable leaves the housing (the photo
    # shows it from the side, near the back plate), the back plate's M3
    # hole pattern, and which way the beams point relative to the cable.
    CABLE_EXIT_Z = 6.0               # cable axis below the back plate (estimate from the photo)
    BACK_PLATE_HOLES = "M3, pattern not published"
    UNVERIFIED = ("CABLE_EXIT_Z", "BACK_PLATE_HOLES")


class Iso7089M16Washer:
    """Stainless trim washer: ISO 7089 M16."""
    SOURCE = "ISO 7089 plain washer, M16: d1 17, d2 30, h 3 mm; mass from its volume in 316 stainless (8.0 g/cm3)"
    D1 = 17.0
    D2 = 30.0
    H = 3.0
    DENSITY = 8.0

    @classmethod
    def mass_g(cls):
        import math
        return math.pi / 4 * (cls.D2 ** 2 - cls.D1 ** 2) * cls.H / 1000.0 * cls.DENSITY

    UNVERIFIED = ()


class FathomTether:
    SOURCE = "Blue Robotics BlueROV2 tether options (https://bluerobotics.com/store/rov/bluerov2/): standard 7.6 mm, slim 4.0 mm, 25-300 m"
    STANDARD_D = 7.6
    SLIM_D = 4.0
    UNVERIFIED = ()


class Water:
    SOURCE = "Seawater about 1.025 g/cm3 at the surface (fresh water 1.000)"
    SEA = 1.025
    FRESH = 1.000
    STAINLESS = 8.0
    UNVERIFIED = ()


ALL = (BlueROV2, WaterLinkedDVLA50, Iso7089M16Washer, FathomTether, Water)
