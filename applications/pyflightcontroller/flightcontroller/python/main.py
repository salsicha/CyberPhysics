import time
import RPi.GPIO as GPIO
import smbus

# Constants
mpu6050_address = 0x68
throttle_idle = 1200
throttle_range = 400
max_rate_roll = 3
max_rate_pitch = 3
max_rate_yaw = 3
pid_roll_kp = 1.0
pid_roll_ki = 0.01
pid_roll_kd = 0.05
pid_pitch_kp = 1.0
pid_pitch_ki = 0.01
pid_pitch_kd = 0.05
pid_yaw_kp = 1.0
pid_yaw_ki = 0.01
pid_yaw_kd = 0.05
cycle_time_seconds = 1 / 100  # 100 Hz
cycle_time_us = int(cycle_time_seconds * 1e6)
i_limit = 20

# Motor pins
M1_pin = 17
M2_pin = 18
M3_pin = 19
M4_pin = 20

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(M1_pin, GPIO.OUT)
GPIO.setup(M2_pin, GPIO.OUT)
GPIO.setup(M3_pin, GPIO.OUT)
GPIO.setup(M4_pin, GPIO.OUT)

# LED pin
led_pin = 25
GPIO.setup(led_pin, GPIO.OUT)

# Motor objects (using PWM)
M1 = GPIO.PWM(M1_pin, 50)  # 50 Hz frequency
M2 = GPIO.PWM(M2_pin, 50)
M3 = GPIO.PWM(M3_pin, 50)
M4 = GPIO.PWM(M4_pin, 50)

# Initialize motors
M1.start(0)
M2.start(0)
M3.start(0)
M4.start(0)

# MPU6050 initialization
bus = smbus.SMBus(1)  # Use I2C bus 1

def calculate_duty_cycle(throttle: float, dead_zone: float = 0.03) -> int:
    """Determines the appropriate PWM duty cycle, in nanoseconds, to use for an ESC controlling a BLDC motor"""
    
    ### SETTINGS (that aren't parameters) ###
    duty_ceiling = 2000000  # max throttle, 100%
    duty_floor = 1000000   # min throttle, 0%
    
    # calculate the filtered percentage (consider dead zone)
    range = 1.0 - dead_zone - dead_zone
    percentage = min(max((throttle - dead_zone) / range, 0.0), 1.0)
    
    dutyns = duty_floor + ((duty_ceiling - duty_floor) * percentage)

    # clamp within the range
    dutyns = max(duty_floor, min(dutyns, duty_ceiling))

    return int(dutyns)

def normalize(value: float, original_min: float, original_max: float, new_min: float, new_max: float) -> float:
    """Normalizes (scales) a value to within a specific range."""
    return new_min + ((new_max - new_min) * ((value - original_min) / (original_max - original_min)))

def translate_pair(high: int, low: int) -> int:
    """Converts a byte pair to a usable value. Borrowed from https://github.com/m-rtijn/mpu6050/blob/0626053a5e1182f4951b78b8326691a9223a5f7d/mpu6050/mpu6050.py#L76C39-L76C39."""
    value = (high << 8) + low
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value

def FATAL_ERROR(msg: str) -> None:
    em = "Fatal error @ " + str(time.time()) + " ms: " + msg
    print(em)
    while True:
        GPIO.output(led_pin, not GPIO.input(led_pin))
        time.sleep(1.0)

def run():
    led.on()  # turn on the onboard LED to signal that the flight controller is now active
    print("-- BEGINNING FLIGHT CONTROL LOOP NOW --")
    try:
        while True:
            
            loop_begin_us = int(time.time() * 1e6)  # microsecond timestamp
            
            gyro_data = bus.read_i2c_block_data(mpu6050_address, 0x43, 6)
            gyro_x = ((translate_pair(gyro_data[0], gyro_data[1]) / 65.5) - gyro_bias_x) * -1
            gyro_y = (translate_pair(gyro_data[2], gyro_data[3]) / 65.5) - gyro_bias_y
            gyro_z = ((translate_pair(gyro_data[4], gyro_data[5]) / 65.5) * -1) - gyro_bias_z

            rc_data = rc.read()

            input_throttle = normalize(rc_data[3], 1000.0, 2000.0, 0.0, 1.0)
            input_pitch = (normalize(rc_data[2], 1000.0, 2000.0, -1.0, 1.0)) * -1
            input_roll = normalize(rc_data[1], 1000.0, 2000.0, -1.0, 1.0)
            input_yaw = normalize(rc_data[4], 1000.0, 2000.0, -1.0, 1.0)

            if rc_data[5] == 1000:
                duty_0_percent = calculate_duty_cycle(0.0)
                M1.ChangeDutyCycle(duty_0_percent)
                M2.ChangeDutyCycle(duty_0_percent)
                M3.ChangeDutyCycle(duty_0_percent)
                M4.ChangeDutyCycle(duty_0_percent)

                roll_last_integral = 0.0
                pitch_last_integral = 0.0
                yaw_last_integral = 0.0

                last_mode = False

            elif rc_data[5] == 2000:
                if last_mode == False:
                    if input_throttle > 0.05:
                        FATAL_ERROR("Throttle was set to " + str(input_throttle) + " as soon as flight mode was entered. Throttle must be at 0% when flight mode begins (safety check).")
                
                adj_throttle = throttle_idle + (throttle_range * input_throttle)

                error_rate_roll = (input_roll * max_rate_roll) - gyro_x
                error_rate_pitch = (input_pitch * max_rate_pitch) - gyro_y
                error_rate_yaw = (input_yaw * max_rate_yaw) - gyro_z

                roll_p = error_rate_roll * pid_roll_kp
                roll_i = roll_last_integral + (error_rate_roll * pid_roll_ki * cycle_time_seconds)
                roll_i = max(min(roll_i, i_limit), -i_limit)
                roll_d = pid_roll_kd * (error_rate_roll - roll_last_error) / cycle_time_seconds
                pid_roll = roll_p + roll_i + roll_d

                pitch_p = error_rate_pitch * pid_pitch_kp
                pitch_i = pitch_last_integral + (error_rate_pitch * pid_pitch_ki * cycle_time_seconds)
                pitch_i = max(min(pitch_i, i_limit), -i_limit)
                pitch_d = pid_pitch_kd * (error_rate_pitch - pitch_last_error) / cycle_time_seconds
                pid_pitch = pitch_p + pitch_i + pitch_d

                yaw_p = error_rate_yaw * pid_yaw_kp
                yaw_i = yaw_last_integral + (error_rate_yaw * pid_yaw_ki * cycle_time_seconds)
                yaw_i = max(min(yaw_i, i_limit), -i_limit)
                yaw_d = pid_yaw_kd * (error_rate_yaw - yaw_last_error) / cycle_time_seconds
                pid_yaw = yaw_p + yaw_i + yaw_d

                t1 = adj_throttle + pid_pitch + pid_roll - pid_yaw
                t2 = adj_throttle + pid_pitch - pid_roll + pid_yaw
                t3 = adj_throttle - pid_pitch + pid_roll + pid_yaw
                t4 = adj_throttle - pid_pitch - pid_roll - pid_yaw

                M1.ChangeDutyCycle(calculate_duty_cycle(t1))
                M2.ChangeDutyCycle(calculate_duty_cycle(t2))
                M3.ChangeDutyCycle(calculate_duty_cycle(t3))
                M4.ChangeDutyCycle(calculate_duty_cycle(t4))

                roll_last_error = error_rate_roll
                pitch_last_error = error_rate_pitch
                yaw_last_error = error_rate_yaw
                roll_last_integral = roll_i
                pitch_last_integral = pitch_i
                yaw_last_integral = yaw_i

                last_mode = True

            else:
                print("Channel 5 input '" + str(rc_data[5]) + "' not valid. Is the transmitter turned on and connected?")

            loop_end_us = int(time.time() * 1e6)
            
            elapsed_us = loop_end_us - loop_begin_us
            if elapsed_us < cycle_time_us:
                time.sleep((cycle_time_us - elapsed_us) / 1e6)

    except Exception as e:
        duty_0_percent = calculate_duty_cycle(0.0)
        M1.ChangeDutyCycle(duty_0_percent)
        M2.ChangeDutyCycle(duty_0_percent)
        M3.ChangeDutyCycle(duty_0_percent)
        M4.ChangeDutyCycle(duty_0_percent)

        GPIO.cleanup()
        
        FATAL_ERROR(str(e))

if __name__ == "__main__":
    run()