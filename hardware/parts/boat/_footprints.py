"""Datasheet footprints of the boat's purchased parts (helper, not a part).

Same conventions as hardware/parts/common/footprints.py: millimetres, every
class cites its source and lists in ``UNVERIFIED`` the attributes the cited
document does not state; a printed part that uses one sets ``fit_test``.
They live here, not in common/footprints.py, only because that shared file
was frozen while these platforms were added; ``ALL`` lists them so the
footprint test can cover them when they move.
"""


class BlueBoat:
    """Blue Robotics BlueBoat (base package): the purchased vessel."""
    SOURCE = ("Blue Robotics BlueBoat (https://bluerobotics.com/store/boat/blueboat/blueboat/): "
              "120 x 93 x 46 cm deployed, 14.5 kg without batteries, 15 kg payload (batteries included), "
              "320 mm max draft, LDPE hulls, two M200 motors, 8.2 kgf static thrust, 3 m/s, 4S-6S, "
              "Raspberry Pi 4 (2 GB) + Navigator, mRobotics M9N GPS; BlueBoat General Integration Guide "
              "(https://bluerobotics.com/learn/blueboat-general-integration-guide/) and BlueBoat Crossbars and "
              "Brackets (https://bluerobotics.com/store/boat/blueboat-components-spares/blueboat-frame-and-crossbars/): "
              "crossbars 6061-T6 with 8 mm holes 25 mm apart on all four sides, M6 hardware through the "
              "included plastic frame bushings; hatch lids with eight molded M6 inserts")
    LENGTH = 1200.0
    BEAM = 930.0
    MASS_KG = 14.5
    PAYLOAD_KG = 15.0
    MAX_DRAFT = 320.0
    CROSSBAR_HOLE_D = 8.0
    CROSSBAR_PITCH = 25.0
    CROSSBAR_SCREW = "M6"
    # Not in the documents: the crossbar's section (the bolts pass through
    # both faces, so their length depends on its height) and where along
    # the bar the hole row starts. The crossbar kit ships M6 x 55 screws.
    CROSSBAR_SECTION = (40.0, 40.0)   # width (along x), height
    CROSSBAR_TOP_Z = 0.0              # the boat frame below puts its top face at z = 0
    UNVERIFIED = ("CROSSBAR_SECTION",)


class Iso7089M6Washer:
    SOURCE = "ISO 7089 plain washer, M6: d1 6.4, d2 12, h 1.6 mm; ISO 273 medium clearance hole for M6: 6.6 mm"
    D1 = 6.4
    D2 = 12.0
    H = 1.6
    CLEARANCE = 6.6
    UNVERIFIED = ()


class Iso4762M6:
    SOURCE = "ISO 4762 socket head cap screw M6: head diameter dk max 10 mm, head height k 6 mm"
    HEAD = (10.0, 6.0)
    UNVERIFIED = ()


class CameraTripodThread:
    SOURCE = ("ISO 1222 / ASME B1.1: camera tripod screw 1/4\"-20 UNC, 6.35 mm major diameter "
              "(hardware/parts/common/fasteners.py)")
    MAJOR = 6.35
    UNVERIFIED = ()


ALL = (BlueBoat, Iso7089M6Washer, Iso4762M6, CameraTripodThread)
