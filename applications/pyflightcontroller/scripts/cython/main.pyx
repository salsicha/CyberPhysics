# main.pyx

import time
cimport cpython.PyThread as PyThread
cimport cpython.PyObject as PyObject
cdef extern from "Python.h":
    ctypedef object PyThreadState

ctypedef struct Motor:
    int pin
    PyThreadState* state

cpdef calculate_duty_cycle(double throttle, double dead_zone=0.03):
    cdef unsigned long duty_ceiling = 2000000  # max throttle, 100%
    cdef unsigned long duty_floor = 1000000   # min throttle, 0%

    range_ = 1.0 - dead_zone - dead_zone
    percentage = min(max((throttle - dead_zone) / range_, 0.0), 1.0)

    dutyns = duty_floor + ((duty_ceiling - duty_floor) * percentage)

    return int(dutyns)

cpdef normalize(double value, double original_min, double original_max, double new_min, double new_max):
    return new_min + ((new_max - new_min) * ((value - original_min) / (original_max - original_min)))

cpdef translate_pair(int high, int low):
    value = (high << 8) + low
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value

cpdef FATAL_ERROR(str msg):
    em = f"Fatal error @ {time.time()} ms: {msg}"
    print(em)
    while True:
        GPIO.output(led_pin, not GPIO.input(led_pin))
        time.sleep(1.0)

cpdef run():
    led.on()
    print("-- BEGINNING FLIGHT CONTROL LOOP NOW --")
    cdef int last_mode = False
    cdef float roll_last_integral = 0.0
    cdef float pitch_last_integral = 0.0
    cdef float yaw_last_integral = 0.0
    cdef float last_error_roll = 0.0
    cdef float last_error_pitch = 0.0
    cdef float last_error_yaw = 0.0

    try:
        while True:
            loop_begin_us = int(time.time() * 1e6)

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
                if not last_mode:
                    if input_throttle > 0.05:
                        FATAL_ERROR(f"Throttle was set to {input_throttle} as soon as flight mode was entered. Throttle must be at 0% when flight mode begins (safety check).")

                adj_throttle = throttle_idle + (throttle_range * input_throttle)

                error_rate_roll = (input_roll * max_rate_roll) - gyro_x
                error_rate_pitch = (input_pitch * max_rate_pitch) - gyro_y
                error_rate_yaw = (input_yaw * max_rate_yaw) - gyro_z

                roll_p = error_rate_roll * pid_roll_kp
                roll_i = roll_last_integral + (error_rate_roll * pid_roll_ki * cycle_time_seconds)
                roll_i = max(min(roll_i, i_limit), -i_limit)
                roll_d = pid_roll_kd * (error_rate_roll - last_error_roll) / cycle_time_seconds
                pid_roll = roll_p + roll_i + roll_d

                pitch_p = error_rate_pitch * pid_pitch_kp
                pitch_i = pitch_last_integral + (error_rate_pitch * pid_pitch_ki * cycle_time_seconds)
                pitch_i = max(min(pitch_i, i_limit), -i_limit)
                pitch_d = pid_pitch_kd * (error_rate_pitch - last_error_pitch) / cycle_time_seconds
                pid_pitch = pitch_p + pitch_i + pitch_d

                yaw_p = error_rate_yaw * pid_yaw_kp
                yaw_i = yaw_last_integral + (error_rate_yaw * pid_yaw_ki * cycle_time_seconds)
                yaw_i = max(min(yaw_i, i_limit), -i_limit)
                yaw_d = pid_yaw_kd * (error_rate_yaw - last_error_yaw) / cycle_time_seconds
                pid_yaw = yaw_p + yaw_i + yaw_d

                t1 = adj_throttle + pid_pitch + pid_roll - pid_yaw
                t2 = adj_throttle + pid_pitch - pid_roll + pid_yaw
                t3 = adj_throttle - pid_pitch + pid_roll + pid_yaw
                t4 = adj_throttle - pid_pitch - pid_roll - pid_yaw

                M1.ChangeDutyCycle(calculate_duty_cycle(t1))
                M2.ChangeDutyCycle(calculate_duty_cycle(t2))
                M3.ChangeDutyCycle(calculate_duty_cycle(t3))
                M4.ChangeDutyCycle(calculate_duty_cycle(t4))

                last_error_roll = error_rate_roll
                last_error_pitch = error_rate_pitch
                last_error_yaw = error_rate_yaw
                roll_last_integral = roll_i
                pitch_last_integral = pitch_i
                yaw_last_integral = yaw_i

                last_mode = True

            else:
                print(f"Channel 5 input '{rc_data[5]}' not valid. Is the transmitter turned on and connected?")

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

cdef class Motor:
    cdef int pin
    cdef PyThreadState* state

    def __init__(self, int pin):
        self.pin = pin
        self.state = PyThreadState_Get()

    cpdef void start(self, double duty_cycle):
        with self.state as gil:
            GPIO.setup(self.pin, GPIO.OUT)
            GPIO.PWM(self.pin, 50).start(duty_cycle)

    cpdef void ChangeDutyCycle(self, double duty_cycle):
        with self.state as gil:
            GPIO.output(self.pin, int(duty_cycle))

cdef Motor M1 = Motor(17)
cdef Motor M2 = Motor(18)
cdef Motor M3 = Motor(19)
cdef Motor M4 = Motor(20)

cpdef gyro_bias_x = 0.0
cpdef gyro_bias_y = 0.0
cpdef gyro_bias_z = 0.0

cpdef int throttle_idle = 1200
cpdef int throttle_range = 400
cpdef double max_rate_roll = 3.0
cpdef double max_rate_pitch = 3.0
cpdef double max_rate_yaw = 3.0
cpdef double pid_roll_kp = 1.0
cpdef double pid_roll_ki = 0.01
cpdef double pid_roll_kd = 0.05
cpdef double pid_pitch_kp = 1.0
cpdef double pid_pitch_ki = 0.01
cpdef double pid_pitch_kd = 0.05
cpdef double pid_yaw_kp = 1.0
cpdef double pid_yaw_ki = 0.01
cpdef double pid_yaw_kd = 0.05
cpdef double cycle_time_seconds = 1 / 100.0
cpdef int cycle_time_us = int(cycle_time_seconds * 1e6)
cpdef int i_limit = 20

cpdef object rc = None  # You need to define or import this from somewhere else
cpdef smbus.SMBus bus = smbus.SMBus(1)

cdef int led_pin = 25
GPIO.setmode(GPIO.BCM)
GPIO.setup(led_pin, GPIO.OUT)

cpdef void on_run():
    run()

if __name__ == "__main__":
    on_run()