
########################################

# how to build:
# python3 setup.py build_ext --inplace
# then run:
# python3 main.py



# TODO: manually convert to python!!!
# 1. profile performance
# 2. add input (ROS?)


# Motor GPIO's (not pin number, GPIO number) ###
gpio_motor1 = 2 # front left, clockwise
gpio_motor2 = 28 # front right, counter clockwise
gpio_motor3 = 15 # rear left, counter clockwise
gpio_motor4 = 16 # rear right, clockwise

# i2c pins used for MPU-6050
gpio_i2c_sda = 12
gpio_i2c_scl = 13

# RC-receiver UART
# either a 0 or 1 (Raspberry Pi Pico supports two UART interfaces)
rc_uart = 1

# throttle settings
throttle_idle = 0.14 # the minumum throttle needed to apply to the four motors for them all to spin up, but not provide lift (idling on the ground). the only way to find this value is through testing (with props off).
throttle_governor = 0.22 # the maximum throttle that can be applied. So, if the pilot is inputting 100% on the controller, it will max out at this. And every value below the pilot's 100% will be scaled linearly within the range of the idle throttle seen above and this governor throttle. If you do not wish to apply a governor (not recommended), set this to None.
# Max attitude rate of change rates (degrees per second)
max_rate_roll = 30.0 # roll
max_rate_pitch = 30.0 # pitch
max_rate_yaw = 50.0 # yaw
# Desired Flight Controller Cycle time
# This is the number of times per second the flight controller will perform an adjustment loop (PID loop)
target_cycle_hz = 250.0


# PID Controller values
pid_roll_kp:float = 0.00043714285
pid_roll_ki:float = 0.00255
pid_roll_kd:float = 0.00002571429
pid_pitch_kp:float = pid_roll_kp
pid_pitch_ki:float = pid_roll_ki
pid_pitch_kd:float = pid_roll_kd
pid_yaw_kp:float = 0.001714287
pid_yaw_ki:float = 0.003428571
pid_yaw_kd:float = 0.0


########################################
########################################
########################################

import time
import toolkit

import smbus2
bus = smbus2.SMBus(1)

from LSM6DSL import *
from MMC5983MA import *
import time
import sys

from rpi_hardware_pwm import HardwarePWM

import gc
# Disable garbage collection
gc.disable()

import flight_funcs
# things like:
# flight_funcs.calculate_duty_cycle()
# should run the cython version of this function



# THE FLIGHT CONTROL LOOP
def run() -> None:
    
    print("Hello from Scout!")

    # wait a few seconds for the IMU to settle
    print("Waiting 3 seconds for the IMU to settle...")
    time.sleep(3)

    # Print settings that are important
    print("Roll PID: " + str(pid_roll_kp) + ", " + str(pid_roll_ki) + ", " + str(pid_roll_kd))
    print("Pitch PID: " + str(pid_pitch_kp) + ", " + str(pid_pitch_ki) + ", " + str(pid_pitch_kd))
    print("Yaw PID: " + str(pid_yaw_kp) + ", " + str(pid_yaw_ki) + ", " + str(pid_yaw_kd))

    # Ozzmaker
    initIMU()
    gxs = []
    gys = []
    gzs = []
    started_at_ticks_ms = time.monotonic() * 1000
    while ((time.monotonic() * 1000 - started_at_ticks_ms) / 1000) < 3.0:
        # TODO: divide by 65.5???
        gxs.append(readGYRx())
        gys.append(readGYRy())
        gzs.append(readGYRz() * -1)
        time.sleep(0.025)
    gyro_bias_x = sum(gxs) / len(gxs)
    gyro_bias_y = sum(gys) / len(gys)
    gyro_bias_z = sum(gzs) / len(gzs)
    print("Gyro bias: " + str((gyro_bias_x, gyro_bias_y, gyro_bias_z)))


    # RPi5 PWM
    pwm1 = HardwarePWM(pwm_channel=0, hz=250, chip=2)
    pwm1.start(50) # full duty cycle
    pwm2 = HardwarePWM(pwm_channel=1, hz=250, chip=2)
    pwm2.start(50) # full duty cycle
    pwm3 = HardwarePWM(pwm_channel=2, hz=250, chip=2)
    pwm3.start(50) # full duty cycle
    pwm4 = HardwarePWM(pwm_channel=3, hz=250, chip=2)
    pwm4.start(50) # full duty cycle


    # Constants calculations / state variables - no need to calculate these during the loop (save processor time)
    cycle_time_seconds = 1.0 / target_cycle_hz
    cycle_time_us = int(round(cycle_time_seconds * 1000000, 0)) # multiply by 1,000,000 to go from seconds to microseconds (us)
    max_throttle = throttle_governor if throttle_governor is not None else 1.0 # what is the MAX throttle we can apply (considering governor)?
    throttle_range = max_throttle - throttle_idle # used for adjusted throttle calculation
    i_limit = 150.0 # PID I-term limiter. The applied I-term cannot exceed or go below (negative) this value. (safety mechanism to prevent excessive spooling of the motors)
    last_mode = False # the most recent mode the flight controller was in. False = Standby (props not spinning), True = Flight mode

    # State variables - PID related
    # required to be delcared outside of the loop because their state will be used in multiple loops (passed from loop to loop)
    roll_last_integral:float = 0.0
    roll_last_error:float = 0.0
    pitch_last_integral:float = 0.0
    pitch_last_error:float = 0.0
    yaw_last_integral:float = 0.0
    yaw_last_error:float = 0.0

    input_throttle:float = 10
    input_roll:float = 0
    input_pitch:float = 0
    input_yaw:float = 0

    adj_throttle:float = 10

    gyro_x:float = 0
    gyro_y:float = 0
    gyro_z:float = 0

    error_rate_roll:float = 0
    error_rate_pitch:float = 0
    error_rate_yaw:float = 0

    roll_p:float = 0
    roll_i:float = 0
    roll_i:float = 0
    roll_d:float = 0
    pid_roll:float = 0

    yaw_p:float = 0
    yaw_i:float = 0
    yaw_i:float = 0
    yaw_d:float = 0
    pid_yaw:float = 0

    t1:float = 0
    t2:float = 0
    t3:float = 0
    t4:float = 0

    loop_end_us:float = 0
    elapsed_us:float = 0

    # Loop performance profiling
    profile_time:float = time.monotonic()
    average_diff:float = 0
    #

    # INFINITE LOOP
    print("-- BEGINNING FLIGHT CONTROL LOOP NOW --")
    try:
        while True:

            # Loop performance profiling
            average_diff = time.monotonic() - profile_time
            print("run av loop time: ", average_diff)
            profile_time = time.monotonic()
            # This is currently looping at just over 200Hz !!! In pure python with no optimization
                    
            # mark start time
            loop_begin_us = time.monotonic() * 1000

            # Capture raw IMU data
            # we divide by 65.5 here because that is the modifier to use at a gyro range scale of 1, which we are using.
            # TODO: divide by 65.5
            gyro_x = (readGYRx() - gyro_bias_x) * -1
            gyro_y = readGYRy() - gyro_bias_y
            gyro_z = (readGYRz() * -1) - gyro_bias_z


            ## TODO:
            ## Stop motors
            # # disable motors
            # pwm1.start(0)
            # pwm2.start(0)
            # pwm3.start(0)
            # pwm4.start(0)
            # # reset PID's
            # roll_last_integral = 0.0
            # roll_last_error = 0.0
            # pitch_last_integral = 0.0
            # pitch_last_error = 0.0
            # yaw_last_integral = 0.0
            # yaw_last_error = 0.0
            # # set last mode
            # last_mode = False


            ## TODO:
            ## Read control commands from RC
            # normalize all RC input values


            # TODO: fix logging in toolkit
            # if last_mode == False: # last mode we were in was standby mode. So, this is the first frame we are going into flight mode
            #     if input_throttle > 0.05: # if throttle is > 5%
            #         FATAL_ERROR("Throttle was set to " + str(input_throttle) + " as soon as flight mode was entered. Throttle must be at 0% when flight mode begins (safety check).")


            # TODO: this replaces the below, which is moved to flight_funcs.pyx
            # TODO: pass all needed values by reference!!!
            flight_funcs.calculate_values(t1, t2, t3, t4)



            # # calculate the adjusted desired throttle (above idle throttle, below governor throttle, scaled linearly)
            # adj_throttle = throttle_idle + (throttle_range * input_throttle)
            # # calculate errors - diff between the actual rates and the desired rates
            # # "error" is calculated as setpoint (the goal) - actual
            # error_rate_roll = (input_roll * max_rate_roll) - gyro_x
            # error_rate_pitch = (input_pitch * max_rate_pitch) - gyro_y
            # error_rate_yaw = (input_yaw * max_rate_yaw) - gyro_z
            # # roll PID calc
            # roll_p = error_rate_roll * pid_roll_kp
            # roll_i = roll_last_integral + (error_rate_roll * pid_roll_ki * cycle_time_seconds)
            # roll_i = max(min(roll_i, i_limit), -i_limit) # constrain within I-term limits
            # roll_d = pid_roll_kd * (error_rate_roll - roll_last_error) / cycle_time_seconds
            # pid_roll = roll_p + roll_i + roll_d
            # # pitch PID calc
            # pitch_p = error_rate_pitch * pid_pitch_kp
            # pitch_i = pitch_last_integral + (error_rate_pitch * pid_pitch_ki * cycle_time_seconds)
            # pitch_i = max(min(pitch_i, i_limit), -i_limit) # constrain within I-term limits
            # pitch_d = pid_pitch_kd * (error_rate_pitch - pitch_last_error) / cycle_time_seconds
            # pid_pitch = pitch_p + pitch_i + pitch_d
            # # yaw PID calc
            # yaw_p = error_rate_yaw * pid_yaw_kp
            # yaw_i = yaw_last_integral + (error_rate_yaw * pid_yaw_ki * cycle_time_seconds)
            # yaw_i = max(min(yaw_i, i_limit), -i_limit) # constrain within I-term limits
            # yaw_d = pid_yaw_kd * (error_rate_yaw - yaw_last_error) / cycle_time_seconds
            # pid_yaw = yaw_p + yaw_i + yaw_d
            # # calculate throttle values
            # t1 = adj_throttle + pid_pitch + pid_roll - pid_yaw
            # t2 = adj_throttle + pid_pitch - pid_roll + pid_yaw
            # t3 = adj_throttle - pid_pitch + pid_roll + pid_yaw
            # t4 = adj_throttle - pid_pitch - pid_roll - pid_yaw



            # TODO: numbers arent scaled properly, duty cycle must be between 0-100, this emits 2000000!!!
            # print(t1, adj_throttle, pid_pitch, pid_roll, pid_yaw)
            # print("duty cycle: ", calculate_duty_cycle(t1))

            # Adjust throttle according to input
            # pwm1.change_duty_cycle(calculate_duty_cycle(t1))
            # pwm2.change_duty_cycle(calculate_duty_cycle(t2))
            # pwm3.change_duty_cycle(calculate_duty_cycle(t3))
            # pwm4.change_duty_cycle(calculate_duty_cycle(t4))

            # Test purposes only
            pwm1.change_duty_cycle(50)
            pwm2.change_duty_cycle(50)
            pwm3.change_duty_cycle(50)
            pwm4.change_duty_cycle(50)


            # Save state values for next loop
            roll_last_error = error_rate_roll
            pitch_last_error = error_rate_pitch
            yaw_last_error = error_rate_yaw
            roll_last_integral = roll_i
            pitch_last_integral = pitch_i
            yaw_last_integral = yaw_i

            # set last mode
            last_mode = True # True = flight mode (props spinning, pid active, motors receiving power commands, etc)

            # mark end time
            loop_end_us = time.monotonic() * 1000000

            # wait to make the hz correct
            elapsed_us = loop_end_us - loop_begin_us
            if elapsed_us < cycle_time_us:
                time.sleep((cycle_time_us - elapsed_us) / 1000000)

    except Exception as e: # something went wrong. Flash the LED so the pilot sees it

        # before we do anything, turn the motors OFF
        duty_0_percent = calculate_duty_cycle(0.0)
        pwm1.change_duty_cycle(duty_0_percent)
        pwm2.change_duty_cycle(duty_0_percent)
        pwm3.change_duty_cycle(duty_0_percent)
        pwm4.change_duty_cycle(duty_0_percent)

        # deinit
        pwm1.stop()
        pwm2.stop()
        pwm3.stop()
        pwm4.stop()
        
        FATAL_ERROR(str(e))


# UTILITY FUNCTIONS BELOW (Anything that is used by the flight controller loop should go here, not in a separate module or class (to save on processing time)

def calculate_duty_cycle(throttle:float, dead_zone:float = 0.03) -> int:
    """Determines the appropriate PWM duty cycle, in nanoseconds, to use for an ESC controlling a BLDC motor"""

    ### SETTINGS (that aren't parameters) ###
    duty_ceiling:int = 2000000 # the maximum duty cycle (max throttle, 100%) is 2 ms, or 10% duty (0.10)
    duty_floor:int = 1000000 # the minimum duty cycle (min throttle, 0%) is 1 ms, or 5% duty (0.05). HOWEVER, I've observed some "twitching" at exactly 5% duty cycle. It is off, but occasionally clips above, triggering the motor temporarily. To prevent this, i'm bringing the minimum down to slightly below 5%
    ################

    # calcualte the filtered percentage (consider dead zone)
    range:float = 1.0 - dead_zone - dead_zone
    percentage:float = min(max((throttle - dead_zone) / range, 0.0), 1.0)
    
    dutyns:int = duty_floor + ((duty_ceiling - duty_floor) * percentage)

    # clamp within the range
    dutyns = max(duty_floor, min(dutyns, duty_ceiling))

    return int(dutyns)

def normalize(value:float, original_min:float, original_max:float, new_min:float, new_max:float) -> float:
    """Normalizes (scales) a value to within a specific range."""
    return new_min + ((new_max - new_min) * ((value - original_min) / (original_max - original_min)))

def translate_pair(high:int, low:int) -> int:
        """Converts a byte pair to a usable value. Borrowed from https://github.com/m-rtijn/mpu6050/blob/0626053a5e1182f4951b78b8326691a9223a5f7d/mpu6050/mpu6050.py#L76C39-L76C39."""
        value = (high << 8) + low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value   

def FATAL_ERROR(msg:str) -> None:
    em:str = "Fatal error @ " + str(time.monotonic() * 1000) + " ms: " + msg
    print(em)
    toolkit.log(em)


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


########### END Ozzmaker PROGRAM ###########


########### RUN THE SCOUT FLIGHT CONTROLLER PROGRAM ###########
run()
