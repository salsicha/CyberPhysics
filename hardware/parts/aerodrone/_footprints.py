"""Datasheet footprints of the aerodrone's purchased parts that are not in
hardware/parts/common/footprints.py (helper, not a part).

Same conventions: millimetres, every class cites its source and lists in
``UNVERIFIED`` the attributes the cited document does not state; a printed
part that uses one sets ``fit_test``. They live here because the shared
file is frozen while this platform changes.
"""


class HaLowLink2:
    """Morse Micro HaLowLink 2: Wi-Fi HaLow (802.11ah, MM8108) router,
    access point and extender, OpenWrt 23.05. Standing on its stand it is 68
    wide, 88 tall and 24 deep; the SMA antenna connector, the two Ethernet
    ports (WAN, LAN) and the USB-C port are on one 88 x 24 side, the SMA
    near the top."""
    SOURCE = ("Morse Micro HaLowLink 2 product brief, Rev 9 "
              "(https://www.morsemicro.com/resources/product_brief/HaLowLink-2_Product-Brief.pdf): "
              "'Dimensions: 88 x 68 x 24mm', 'Weight HaLowLink only: 102g', 'HaLowLink + stand + antenna: 168g', "
              "'Power: 5V...2A', '1x USB-C port (power and Ethernet data)', '2x Gigabit Ethernet ports', "
              "'Temperature range: 0 - 40C', SMA connector; HaLowLink user guide 2.11 "
              "(https://static.gl-inet.com/www/images/products/user-guide/haLowlink2_user-guide.pdf): Extender mode, "
              "6.3 'Virtual Wire', dipole antenna 2.34 dBi, 20 cm RF exposure distance")
    SIZE = (88.0, 68.0, 24.0)          # tall, wide, deep (standing)
    MASS_G = 102.0
    SUPPLY = (5.0, 2.0)                # V, A
    TEMPERATURE_C = (0.0, 40.0)
    # Scaled from the brief's drawing, not dimensioned: the SMA's distance
    # from the top end of the connector side, and the antenna (a straight
    # dipole whip on SMA, "region specific"); its mass is part of the 66 g of
    # stand and antenna.
    SMA_FROM_TOP = 23.0
    ANTENNA = (200.0, 13.0)            # length, diameter
    ANTENNA_MASS_G = 25.0
    # SMA bulkhead jack: 1/4-36 UNS thread (6.35 mm), a generic RF part.
    BULKHEAD_HOLE = 6.6
    UNVERIFIED = ("SMA_FROM_TOP", "ANTENNA", "ANTENNA_MASS_G")


class PololuD36V28F5:
    """Pololu 5 V, 3.2 A step-down regulator D36V28F5 (the HaLowLink's own
    supply)."""
    SOURCE = "Pololu D36V28F5 (https://www.pololu.com/product/3782): 5 V +-4 %, 3.2 A typical maximum, 5.3-50 V in"
    SIZE = (17.8, 20.3, 8.8)
    MASS_G = 3.0
    UNVERIFIED = ("MASS_G",)


class PololuD36V50F12:
    """Pololu 12 V, 4.5 A step-down regulator D36V50F12 (the launcher's servo rail)."""
    SOURCE = "Pololu D36V50F12 (https://www.pololu.com/product/4095): 12 V +-4 %, 4.5 A typical maximum, 13.3-50 V in"
    SIZE = (25.4, 25.4, 9.5)
    MASS_G = 8.0
    UNVERIFIED = ("MASS_G",)


ALL = (HaLowLink2, PololuD36V28F5, PololuD36V50F12)
