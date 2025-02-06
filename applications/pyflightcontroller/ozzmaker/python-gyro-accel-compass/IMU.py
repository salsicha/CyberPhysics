import smbus
bus = smbus.SMBus(1)

from LSM6DSL import *
from MMC5983MA import *
import time
import sys






def detectIMU():
    try:
        #Check for OzzMaker LTE IMU ALT (LSM6DSL and MMC5983MA)
        #If no LSM6DSL or MMC5983MA is connected, there will be an I2C bus error and the program will exit.
        #This section of code stops this from happening.
        LSM6DSL_WHO_AM_I_response = (bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_WHO_AM_I))
        MMC5983MA_WHO_AM_I_response = (bus.read_byte_data(MMC5983MA_ADDRESS,MMC5983MA_WHO_AM_I ))

    except IOError as f:
        print('OzzMaker LTE IMU ALT not found')        #need to do something here, so we just print a space
        sys.exit(1)
    else:
        if (LSM6DSL_WHO_AM_I_response == 0x6A) and (MMC5983MA_WHO_AM_I_response == 0x30):
            print("Found OzzMaker LTE IMU ALT (LSM6DSL and MMC5983MA)")

    time.sleep(1)














def writeByte(device_address,register,value):
    bus.write_byte_data(device_address, register, value)



def readACCx():
    acc_l = 0
    acc_h = 0

    acc_l = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_L_XL)
    acc_h = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_H_XL)

    acc_combined = (acc_l | acc_h <<8)
    return acc_combined  if acc_combined < 32768 else acc_combined - 65536


def readACCy():
    acc_l = 0
    acc_h = 0

    acc_l = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTY_L_XL)
    acc_h = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTY_H_XL)

    acc_combined = (acc_l | acc_h <<8)
    return acc_combined  if acc_combined < 32768 else acc_combined - 65536


def readACCz():
    acc_l = 0
    acc_h = 0

    acc_l = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTZ_L_XL)
    acc_h = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTZ_H_XL)

    acc_combined = (acc_l | acc_h <<8)
    return acc_combined  if acc_combined < 32768 else acc_combined - 65536


def readGYRx():
    gyr_l = 0
    gyr_h = 0

    gyr_l = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_L_G)
    gyr_h = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTX_H_G)

    gyr_combined = (gyr_l | gyr_h <<8)
    return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536


def readGYRy():
    gyr_l = 0
    gyr_h = 0

    gyr_l = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTY_L_G)
    gyr_h = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTY_H_G)

    gyr_combined = (gyr_l | gyr_h <<8)
    return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536

def readGYRz():
    gyr_l = 0
    gyr_h = 0

    gyr_l = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTZ_L_G)
    gyr_h = bus.read_byte_data(LSM6DSL_ADDRESS, LSM6DSL_OUTZ_H_G)

    gyr_combined = (gyr_l | gyr_h <<8)
    return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536


def readMAGx():
    mag_l = 0
    mag_h = 0

    mag_l = bus.read_byte_data(MMC5983MA_ADDRESS, MMC5983MA_XOUT_0)
    mag_h = bus.read_byte_data(MMC5983MA_ADDRESS, MMC5983MA_XOUT_1)
    mag_xyz = bus.read_byte_data(MMC5983MA_ADDRESS,MMC5983MA_XYZOUT_2)

    return mag_l << 10 | mag_h << 2 | (mag_xyz & 0b11000000) >> 6


def readMAGy():
    mag_l = 0
    mag_h = 0

    mag_l = bus.read_byte_data(MMC5983MA_ADDRESS, MMC5983MA_YOUT_0)
    mag_h = bus.read_byte_data(MMC5983MA_ADDRESS, MMC5983MA_YOUT_1)
    mag_xyz = bus.read_byte_data(MMC5983MA_ADDRESS,MMC5983MA_XYZOUT_2)

    return mag_l << 10 | mag_h <<2 | (mag_xyz & 0b00110000) >> 6


def readMAGz():
    mag_l = 0
    mag_h = 0

    mag_l = bus.read_byte_data(MMC5983MA_ADDRESS, MMC5983MA_ZOUT_0)
    mag_h = bus.read_byte_data(MMC5983MA_ADDRESS, MMC5983MA_ZOUT_1)
    mag_xyz = bus.read_byte_data(MMC5983MA_ADDRESS,MMC5983MA_XYZOUT_2)

    return mag_l << 10 | mag_h <<2 | (mag_xyz & 0b00001100) >> 6




def initIMU():


        #initialise the accelerometer
        writeByte(LSM6DSL_ADDRESS,LSM6DSL_CTRL1_XL,0b10011111)           #ODR 3.33 kHz, +/- 8g , BW = 400hz
        writeByte(LSM6DSL_ADDRESS,LSM6DSL_CTRL8_XL,0b11001000)           #Low pass filter enabled, BW9, composite filter
        writeByte(LSM6DSL_ADDRESS,LSM6DSL_CTRL3_C,0b01000100)            #Enable Block Data update, increment during multi byte read

        #initialise the gyroscope
        writeByte(LSM6DSL_ADDRESS,LSM6DSL_CTRL2_G,0b10011100)            #ODR 3.3 kHz, 2000 dps


        #Enable compass, Continuous measurement mode, 100Hz
        writeByte(MMC5983MA_ADDRESS,MMC5983MA_CONTROL_0,0b00001000)     #"deGauss" magnetometer
        time.sleep(0.2)
        writeByte(MMC5983MA_ADDRESS,MMC5983MA_CONTROL_1,0b10000000)     #soft reset
        time.sleep(0.2)
        writeByte(MMC5983MA_ADDRESS,MMC5983MA_CONTROL_0,0b00100100)     #Enable auto reset
        writeByte(MMC5983MA_ADDRESS,MMC5983MA_CONTROL_1,0b00000000)     #Filter bandwdith 100Hz (16 bit mode)
        writeByte(MMC5983MA_ADDRESS,MMC5983MA_CONTROL_2,0b10001101)     #Continous mode at 100Hz
