
cdef int calculate_duty_cycle(double *throttle, double *dead_zone):
    """Determines the appropriate PWM duty cycle, in nanoseconds, to use for an ESC controlling a BLDC motor"""

    cdef int duty_ceiling = 2000000
    cdef int duty_floor = 1000000

    # calcualte the filtered percentage (consider dead zone)
    cdef double range = 1.0 - dead_zone[0] - dead_zone[0]
    cdef double percentage = min(max((throttle[0] - dead_zone[0]) / range, 0.0), 1.0)
    
    cdef double dutyns = duty_floor + ((duty_ceiling - duty_floor) * percentage)

    # clamp within the range
    dutyns = max(duty_floor, min(dutyns, duty_ceiling))

    return int(dutyns)


cdef double normalize(double *value, double *original_min, double *original_max, double *new_min, double *new_max):
    """Normalizes (scales) a value to within a specific range."""
    cdef double result = new_min[0] + ((new_max[0] - new_min[0]) * ((value[0] - original_min[0]) / (original_max[0] - original_min[0])))
    
    return result


cdef int translate_pair(int *high, int *low):
    """Converts a byte pair to a usable value. Borrowed from https://github.com/m-rtijn/mpu6050/blob/0626053a5e1182f4951b78b8326691a9223a5f7d/mpu6050/mpu6050.py#L76C39-L76C39."""
    value = (high[0] << 8) + low[0]
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value   


cdef void adj_throttle(double *adj_throttle, double *throttle_idle, double *throttle_range, double *input_throttle):
    # calculate the adjusted desired throttle (above idle throttle, below governor throttle, scaled linearly)
    # Cython style dereferencing!!!
    adj_throttle[0] = throttle_idle[0] + (throttle_range[0] * input_throttle[0])


cdef void calc_errors(double *max_rate_roll, double *max_rate_pitch, double *max_rate_yaw, double *error_rate_roll, double *error_rate_pitch, double *error_rate_yaw, double *input_roll, double *input_pitch, double *input_yaw, double *gyro_x, double *gyro_y, double *gyro_z):
    # calculate errors - diff between the actual rates and the desired rates
    # "error" is calculated as setpoint (the goal) - actual
    error_rate_roll[0] = (input_roll[0] * max_rate_roll[0]) - gyro_x[0]
    error_rate_pitch[0] = (input_pitch[0] * max_rate_pitch[0]) - gyro_y[0]
    error_rate_yaw[0] = (input_yaw[0] * max_rate_yaw[0]) - gyro_z[0]


cdef void roll_pid_calc(double *pid_roll, double *roll_last_integral, double *roll_p, double *roll_i, double *roll_d, double *error_rate_roll, double *roll_last_error, double *pid_roll_kp, double *pid_roll_ki, double *pid_roll_kd, double *cycle_time_seconds, double *i_limit):
    # roll PID calc
    roll_p[0] = error_rate_roll[0] * pid_roll_kp[0]
    roll_i[0] = roll_last_integral[0] + (error_rate_roll[0] * pid_roll_ki[0] * cycle_time_seconds[0])
    roll_i[0] = max(min(roll_i[0], i_limit[0]), -i_limit[0])
    roll_d[0] = pid_roll_kd[0] * (error_rate_roll[0] - roll_last_error[0]) / cycle_time_seconds[0]
    pid_roll[0] = roll_p[0] + roll_i[0] + roll_d[0]


cdef void pitch_pid_calc(double *pitch_p, double *pitch_i, double *pitch_d, double *pid_pitch, double *error_rate_pitch, double *pid_pitch_kp, double *pitch_last_integral, double *pid_pitch_ki, double *cycle_time_seconds, double *i_limit, double *pid_pitch_kd, double *pitch_last_error):
    # pitch PID calc
    pitch_p[0] = error_rate_pitch[0] * pid_pitch_kp[0]
    pitch_i[0] = pitch_last_integral[0] + (error_rate_pitch[0] * pid_pitch_ki[0] * cycle_time_seconds[0])
    pitch_i[0] = max(min(pitch_i[0], i_limit[0]), -i_limit[0]) 
    pitch_d[0] = pid_pitch_kd[0] * (error_rate_pitch[0] - pitch_last_error[0]) / cycle_time_seconds[0]
    pid_pitch[0] = pitch_p[0] + pitch_i[0] + pitch_d[0]


cdef void yaw_pid_calc(double *pid_yaw, double *yaw_p, double *yaw_i, double *yaw_d, double *error_rate_yaw, double *pid_yaw_kp, double *yaw_last_integral, double *pid_yaw_ki, double *cycle_time_seconds, double *i_limit, double *pid_yaw_kd, double *yaw_last_error):
    # yaw PID calc
    yaw_p[0] = error_rate_yaw[0] * pid_yaw_kp[0]
    yaw_i[0] = yaw_last_integral[0] + (error_rate_yaw[0] * pid_yaw_ki[0] * cycle_time_seconds[0])
    yaw_i[0] = max(min(yaw_i[0], i_limit[0]), -i_limit[0]) 
    yaw_d[0] = pid_yaw_kd[0] * (error_rate_yaw[0] - yaw_last_error[0]) / cycle_time_seconds[0]
    pid_yaw[0] = yaw_p[0] + yaw_i[0] + yaw_d[0]


cdef void throttle_calc(double *t1, double *t2, double *t3, double *t4, double *adj_throttle, double *pid_roll, double *pid_pitch, double *pid_yaw):
    # calculate throttle values
    t1[0] = adj_throttle[0] + pid_pitch[0] + pid_roll[0] - pid_yaw[0]
    t2[0] = adj_throttle[0] + pid_pitch[0] - pid_roll[0] + pid_yaw[0]
    t3[0] = adj_throttle[0] - pid_pitch[0] + pid_roll[0] + pid_yaw[0]
    t4[0] = adj_throttle[0] - pid_pitch[0] - pid_roll[0] - pid_yaw[0]



