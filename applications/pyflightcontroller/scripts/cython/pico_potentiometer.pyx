cimport cpython.pycapsule
from libc.stdlib cimport malloc, free
from libgcc cimport __floatundisf4

cdef extern from "RPI.GPIO.h":
    void setmode(int mode)
    void setup(int pin, int mode)
    void output(int pin, int value)

cdef extern from "spidev.h":
    void open(int bus, int device)
    void max_speed_hz(int hz)
    list xfer2(list data)

import RPi.GPIO as GPIO
import time
import spidev

### SETTINGS ###
adc_channel = 0  # The channel of the potentiometer on the MCP3008 ADC
pwm_pin = 18     # GPIO pin being used for the PWM output
sample_rate_hz = 250  # Number of times per second
moving_average = 10  # Number of frames in the moving average

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000

# Define GPIO mode and setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(pwm_pin, GPIO.OUT)

# PWM setup using Pulse Width Modulation (PWM)
pwm = GPIO.PWM(pwm_pin, 250)  # Set the frequency to 250 Hz
pwm.start(0)

# Setup LED pin
led_pin = 25
GPIO.setup(led_pin, GPIO.OUT)
GPIO.output(led_pin, GPIO.HIGH)

cdef list ma = []

cpdef read_adc(int channel):
    """Read an analog value from a specified channel on the MCP3008 ADC"""
    cdef list adc = spi.xfer2([1, (8 + channel) << 4, 0])
    cdef int data = ((adc[1] & 3) << 8) + adc[2]
    return data

cpdef calculate_duty_cycle(float throttle, float dead_zone=0.03):
    """Determines the appropriate PWM duty cycle, in nanoseconds, to use for an ESC (at 50 Hz)"""

    ### SETTINGS (that aren't parameters) ###
    cdef int duty_ceiling = 2000000  # The maximum duty cycle (max throttle, 100%) is 2 ms, or 10% duty (0.10)
    cdef int duty_floor = 1000000   # The minimum duty cycle (min throttle, 0%) is 1 ms, or 5% duty (0.05). However, I've observed some "twitching" at exactly 5% duty cycle. It is off, but occasionally clips above, triggering the motor temporarily. To prevent this, i'm bringing the minimum down to slightly below 5%
    ################

    # Calculate the filtered percentage (consider dead zone)
    cdef float range = 1.0 - dead_zone - dead_zone
    cdef float percentage = min(max((throttle - dead_zone) / range, 0.0), 1.0)

    cdef int dutyns = duty_floor + ((duty_ceiling - duty_floor) * percentage)

    # Clamp within the range
    cdef int clamped_dutyns = max(duty_floor, min(dutyns, duty_ceiling))

    return clamped_dutyns

cpdef main():
    while True:
        cdef int val = read_adc(adc_channel)

        # Add to moving average
        ma.append(val)
        while len(ma) > moving_average:
            ma.pop(0)

        # Average the values in the moving average list
        cdef float avg = sum(ma) / len(ma)

        # Convert ADC value to a percentage (assuming 0-3.3V range and 10-bit resolution)
        cdef float percent = avg / 1024.0

        # Calculate the duty cycle
        cdef int nanoseconds = calculate_duty_cycle(percent)

        # Set the PWM duty cycle
        pwm.ChangeDutyCycle(nanoseconds / 10000)  # Convert nanoseconds to duty cycle percentage

        # Print the throttle value and duty cycle in nanoseconds
        print(f"{round(percent * 100, 0)}% ({nanoseconds} nanoseconds)")

        # Wait for the next sample
        time.sleep(1 / sample_rate_hz)

if __name__ == "__main__":
    main()