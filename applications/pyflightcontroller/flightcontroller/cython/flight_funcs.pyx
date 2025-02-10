

def calculate_values(t1, t2, t3, t4):
        # calculate the adjusted desired throttle (above idle throttle, below governor throttle, scaled linearly)
        adj_throttle = throttle_idle + (throttle_range * input_throttle)

        # calculate errors - diff between the actual rates and the desired rates
        # "error" is calculated as setpoint (the goal) - actual
        error_rate_roll = (input_roll * max_rate_roll) - gyro_x
        error_rate_pitch = (input_pitch * max_rate_pitch) - gyro_y
        error_rate_yaw = (input_yaw * max_rate_yaw) - gyro_z

        # roll PID calc
        roll_p = error_rate_roll * pid_roll_kp
        roll_i = roll_last_integral + (error_rate_roll * pid_roll_ki * cycle_time_seconds)
        roll_i = max(min(roll_i, i_limit), -i_limit) # constrain within I-term limits
        roll_d = pid_roll_kd * (error_rate_roll - roll_last_error) / cycle_time_seconds
        pid_roll = roll_p + roll_i + roll_d

        # pitch PID calc
        pitch_p = error_rate_pitch * pid_pitch_kp
        pitch_i = pitch_last_integral + (error_rate_pitch * pid_pitch_ki * cycle_time_seconds)
        pitch_i = max(min(pitch_i, i_limit), -i_limit) # constrain within I-term limits
        pitch_d = pid_pitch_kd * (error_rate_pitch - pitch_last_error) / cycle_time_seconds
        pid_pitch = pitch_p + pitch_i + pitch_d

        # yaw PID calc
        yaw_p = error_rate_yaw * pid_yaw_kp
        yaw_i = yaw_last_integral + (error_rate_yaw * pid_yaw_ki * cycle_time_seconds)
        yaw_i = max(min(yaw_i, i_limit), -i_limit) # constrain within I-term limits
        yaw_d = pid_yaw_kd * (error_rate_yaw - yaw_last_error) / cycle_time_seconds
        pid_yaw = yaw_p + yaw_i + yaw_d

        # calculate throttle values
        t1 = adj_throttle + pid_pitch + pid_roll - pid_yaw
        t2 = adj_throttle + pid_pitch - pid_roll + pid_yaw
        t3 = adj_throttle - pid_pitch + pid_roll + pid_yaw
        t4 = adj_throttle - pid_pitch - pid_roll - pid_yaw


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



