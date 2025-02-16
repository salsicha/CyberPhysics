
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import Header
from geometry_msgs.msg import (Quaternion, Vector3)

import time

import smbus2
bus = smbus2.SMBus(1)

from LSM6DSL import *
from MMC5983MA import *

from gps import *

import gc
gc.disable() # Disable garbage collection


########### Ozzmaker PROGRAM ###########

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


def run():

    initIMU()

    # TODO: 
    # Sync clock to GPS

    gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE) 

    rclpy.init()
    node = Node('nav_publisher_node')
    imu_pub = node.create_publisher(Imu, 'imu', 10)
    nav_pub = node.create_publisher(NavSatFix, 'imu', 10)

    rate = node.create_rate(200) # 200 Hz, device supports up to 6000 Hz!!!

    while rclpy.ok():

        # IMU

        imu_msg = Imu()

        accel = Vector3()
        accel.x = readACCx()
        accel.y = readACCy()
        accel.z = readACCz()
        imu_msg.linear_acceleration = accel
        imu_msg.linear_acceleration_covariance[0] = 0.00001
        imu_msg.linear_acceleration_covariance[4] = 0.00001
        imu_msg.linear_acceleration_covariance[8] = 0.00001

        gyro = Vector3()
        gyro.x = readGYRx()
        gyro.y = readGYRy()
        gyro.z = readGYRz()
        imu_msg.angular_velocity = gyro
        imu_msg.angular_velocity_covariance[0] = 0.00001
        imu_msg.angular_velocity_covariance[4] = 0.00001
        imu_msg.angular_velocity_covariance[8] = 0.00001
            
        imu_msg.orientation[w] = 0.0
        imu_msg.orientation[x] = 0.0
        imu_msg.orientation[y] = 0.0
        imu_msg.orientation[z] = 0.0           
        imu_msg.orientation_covariance[0] = 0.00001
        imu_msg.orientation_covariance[4] = 0.00001
        imu_msg.orientation_covariance[8] = 0.00001
            
        imu_msg.header.stamp = node.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu"

        imu_pub.publish(imu_msg)

        # NAV

        report = gpsd.next() #
        if report['class'] == 'TPV':
            nav_msg = NavSatFix()
            nav_msg.header = Header()
            nav_msg.header.stamp = node.get_clock().now().to_msg()
            nav_msg.header.frame_id = "gps"

            nav_msg.status.status = NavSatStatus.STATUS_FIX
            nav_msg.status.service = NavSatStatus.SERVICE_GPS
             
            nav_msg.latitude = getattr(report,'lat',0.0)
            nav_msg.longitude = getattr(report,'lon',0.0)
            nav_msg.altitude = getattr(report,'alt','nan')

            # getattr(report,'time','')
            # getattr(report,'epv','nan')
            # getattr(report,'ept','nan')
            # getattr(report,'speed','nan')
            # getattr(report,'climb','nan')
        
            nav_msg.position_covariance[0] = 0
            nav_msg.position_covariance[4] = 0
            nav_msg.position_covariance[8] = 0
            nav_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            
            nav_pub.publish(nav_msg)

        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

run()

