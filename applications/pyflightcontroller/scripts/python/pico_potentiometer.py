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

ma = []

def read_adc(channel):
    """Read an analog value from a specified channel on the MCP3008 ADC"""
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

def calculate_duty_cycle(throttle: float, dead_zone: float = 0.03) -> int:
    """Determines the appropriate PWM duty cycle, in nanoseconds, to use for an ESC (at 50 Hz)"""

    ### SETTINGS (that aren't parameters) ###
    duty_ceiling = 2000000  # The maximum duty cycle (max throttle, 100%) is 2 ms, or 10% duty (0.10)
    duty_floor = 1000000   # The minimum duty cycle (min throttle, 0%) is 1 ms, or 5% duty (0.05). However, I've observed some "twitching" at exactly 5% duty cycle. It is off, but occasionally clips above, triggering the motor temporarily. To prevent this, i'm bringing the minimum down to slightly below 5%
    ################

    # Calculate the filtered percentage (consider dead zone)
    range = 1.0 - dead_zone - dead_zone
    percentage = min(max((throttle - dead_zone) / range, 0.0), 1.0)

    dutyns = duty_floor + ((duty_ceiling - duty_floor) * percentage)

    # Clamp within the range
    dutyns = max(duty_floor, min(dutyns, duty_ceiling))

    return int(dutyns)

while True:
    val = read_adc(adc_channel)

    # Add to moving average
    ma.append(val)
    while len(ma) > moving_average:
        ma.pop(0)

    # Average the values in the moving average list
    avg = sum(ma) // len(ma)

    # Convert ADC value to a percentage (assuming 0-3.3V range and 10-bit resolution)
    percent = avg / 1024.0

    # Calculate the duty cycle
    nanoseconds = calculate_duty_cycle(percent)

    # Set the PWM duty cycle
    pwm.ChangeDutyCycle(nanoseconds / 10000)  # Convert nanoseconds to duty cycle percentage

    # Print the throttle value and duty cycle in nanoseconds
    print(f"{round(percent * 100, 0)}% ({nanoseconds} nanoseconds)")

    # Wait for the next sample
    time.sleep(1 / sample_rate_hz)