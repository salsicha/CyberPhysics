MMC5983MA_XOUT_0        =   0x00
MMC5983MA_XOUT_1        =   0x01
MMC5983MA_YOUT_0        =   0x02
MMC5983MA_YOUT_1        =   0x03
MMC5983MA_ZOUT_0        =   0x04
MMC5983MA_ZOUT_1        =   0x05
MMC5983MA_XYZOUT_2      =   0x06
MMC5983MA_TOUT          =   0x07
MMC5983MA_STATUS        =   0x08
MMC5983MA_CONTROL_0     =   0x09
MMC5983MA_CONTROL_1     =   0x0A
MMC5983MA_CONTROL_2     =   0x0B
MMC5983MA_CONTROL_3     =   0x0C
MMC5983MA_WHO_AM_I    =   0x2F # Should be =   0x30

MMC5983MA_ADDRESS       =   0x30

# Sample rates
MODR_ONESHOT   =   0x00
MODR_1Hz       =   0x01
MODR_10Hz      =   0x02
MODR_20Hz      =   0x03
MODR_50Hz      =   0x04
MODR_100Hz     =   0x05
MODR_200Hz     =   0x06 # BW = =   0x01 only
MODR_1000Hz    =   0x07 # BW = =   0x11 only

#Bandwidths
MBW_100Hz =   0x00  # 8 ms measurement time
MBW_200Hz =   0x01  # 4 ms
MBW_400Hz =   0x02  # 2 ms
MBW_800Hz =   0x03  # 0.5 ms


# Set/Reset as a function of measurements
MSET_1     =   0x00 # Set/Reset each data measurement
MSET_25    =   0x01 # each 25 data measurements
MSET_75    =   0x02
MSET_100   =   0x03
MSET_250   =   0x04
MSET_500   =   0x05
MSET_1000  =   0x06
MSET_2000  =   0x07
