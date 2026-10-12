"""Datasheet footprints of the aerodrone's purchased parts that are not in
hardware/parts/common/footprints.py (helper, not a part).

Same conventions: millimetres, every class cites its source and lists in
``UNVERIFIED`` the attributes the cited document does not state; a printed
part that uses one sets ``fit_test``. They live here because the shared
file is frozen while this platform changes (the shared Holybro2216KV920
and Propeller1045 there describe the X500-class propulsion this aircraft
no longer uses).
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


class TMotorMN3510KV700:
    """T-Motor MN3510 KV700 (Navigator series), 3-4S: the aerodrone's motor
    from the 12 in propulsion upgrade on (it replaced the Holybro 2216 KV920,
    whose 10 in propellers left the launcher configuration under 2:1)."""
    SOURCE = ("T-Motor MN3510 KV360/KV630/KV700 product page "
              "(https://store.tmotor.com/product/mn3510-motor-navigator-type.html): KV700 'Motor Dimensions "
              "Φ41.8×28.5mm', 'Weight Excluding Cables 97g', 'Weight Including Cables 118g', 600 mm 18# cables, "
              "'No.of Cells(Lipo) 3-4S', 'Max Continuous Current 180S 25A', 'Max Power (180S) 555W', 50 mΩ, "
              "4 mm shaft; the page's drawing: base '4-M3 Ø25', prop face '4-M3 Ø12'; test data with the "
              "T-MOTOR 12*4CF propeller at 14.8 V (assembly.THRUST_TABLE)")
    KV = 700
    DIAMETER = 41.8
    HEIGHT = 28.5
    MASS_BARE_G = 97.0
    MASS_WITH_CABLES_G = 118.0          # with the full 600 mm leads
    # Leads cut to about 200 mm (the ESC sits on the arm): three 18 AWG
    # silicone wires and the bullets, an estimate.
    LEADS_G = 7.0
    MASS_G = MASS_BARE_G + LEADS_G
    MOUNT_PCD = 25.0                    # four M3 on a 25 mm circle, 90 deg apart (a cross)
    MOUNT_SCREW = "M3"
    PROP_PCD = 12.0
    CELLS = (3, 4)
    MAX_CONTINUOUS_A = 25.0
    # Not dimensioned on the page: the thread depth in the base (M3 x 8
    # through the 4 mm seat engages 4 mm) and the circlip on the shaft end
    # at the base centre (the seat's 10 mm hole clears it).
    THREAD_DEPTH = 4.0
    BASE_CENTRE_CLEARANCE_D = 10.0
    UNVERIFIED = ("LEADS_G", "THREAD_DEPTH", "BASE_CENTRE_CLEARANCE_D")


class TMotorP12x4:
    """T-Motor P12*4 polished carbon propeller (2 per pair, CW + CCW)."""
    SOURCE = ("T-Motor P12*4 Prop-2PCS/PAIR product page "
              "(https://store.tmotor.com/product/polish-carbon-fiber-12x4-prop.html): '12×4 (304.8×101.6mm)', "
              "'14.5g' (the page's 'single blade', one propeller), 'Thrust Limitation 4kg', "
              "'Optimum RPM 5000~9000', CF+Epoxy")
    DIAMETER = 304.8
    PITCH_IN = 4.0
    MASS_G = 14.5
    THRUST_LIMIT_G = 4000.0
    OPTIMUM_RPM = (5000, 9000)
    # The motor's test data name it 'T-MOTOR 12*4CF', T-Motor's carbon 12 x 4
    # that the P12*4 is sold as; the hub's height is not dimensioned (the
    # disc is modelled 3 mm thick on a 10 mm hub envelope).
    HUB_HEIGHT = 10.0
    UNVERIFIED = ("HUB_HEIGHT",)


class TMotorAir40A:
    """T-Motor AIR 40A 2-6S, no BEC: the multirotor ESC T-Motor pairs with
    its U3 KV700 (the MN3510 KV700's industrial twin, same test data);
    standard PWM input."""
    SOURCE = ("T-Motor AIR 40A 2-6S NO BEC product page (https://store.tmotor.com/product/air-40a-6s-esc.html): "
              "40 A, 2-6S LiPo, 'signal frequency of up to 621 Hz', 'Matching Guide: Tmotor U3 KV700 Motor'; "
              "reseller listings: 'peak reference is 60A', 'approximate dimensions of 55.6*25.2*11.3mm', "
              "'weight including cables of 26g' (https://store.uniteduav.com/products/air-40a-6s); "
              "68 x 25 x 8.7 mm, 26 g (https://www.electricwingman.com/air-40-esc)")
    CONTINUOUS_A = 40.0
    PEAK_A = 60.0
    CELLS = (2, 6)
    MAX_SIGNAL_HZ = 621
    SIZE = (56.0, 26.0, 11.5)           # envelope: length, width, height
    MASS_G = 26.0
    UNVERIFIED = ("PEAK_A", "SIZE", "MASS_G")


ALL = (HaLowLink2, PololuD36V28F5, PololuD36V50F12, TMotorMN3510KV700, TMotorP12x4, TMotorAir40A)
