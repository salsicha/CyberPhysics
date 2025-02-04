cimport RPi.GPIO as GPIO
cimport pigpio
from libc.stdio cimport printf
from time cimport sleep
cdef extern from "unistd.h":
    int sleep(int seconds)

# Motor GPIO's (not pin number, GPIO number) ###
cdef const int gpio_motor1 = 28 # front left, clockwise
cdef const int gpio_motor2 = 2    # front right, counter clockwise
cdef const int gpio_motor3 = 16   # rear left, counter clockwise
cdef const int gpio_motor4 = 15   # rear right, clockwise

# min and max throttle (nanoseconds)
cdef const int throttle_max = 2000000
cdef const int throttle_min = 1000000

cpdef void calibrate():
    printf("Hello! Welcome to the ESC throttle range calibration script\n")
    printf("The Raspberry Pi should be running on SEPARATE power from the ESC's\n")
    printf("This is because the Raspberry Pi needs to be able to set the throttle to 100%% BEFORE the ESC's gain power\n")
    printf("To accomplish this, the Raspberry Pi will need to be on a different power source (USB) while the ESC's are powered by the LiPo.\n")
    printf("MAKE SURE you have removed the +5V BEC circuit from the VSYS of the pico before continuing.\n")
    printf("Please ensure that right now the ESC's are NOT powered on\n")
    printf("Press enter to confirm this and when you are ready\n")
    sleep(1)

    # Set up GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(gpio_motor1, GPIO.OUT)
    GPIO.setup(gpio_motor2, GPIO.OUT)
    GPIO.setup(gpio_motor3, GPIO.OUT)
    GPIO.setup(gpio_motor4, GPIO.OUT)

    printf("GPIO's set up\n")

    # Set up PWM
    cdef pigpio.pi pi = pigpio.pi()
    pi.set_mode(gpio_motor1, pigpio.OUTPUT)
    pi.set_mode(gpio_motor2, pigpio.OUTPUT)
    pi.set_mode(gpio_motor3, pigpio.OUTPUT)
    pi.set_mode(gpio_motor4, pigpio.OUTPUT)

    cdef int pwm_freq = 50  # ESC's typically use a frequency of 50 Hz
    pi.set_PWM_frequency(gpio_motor1, pwm_freq)
    pi.set_PWM_frequency(gpio_motor2, pwm_freq)
    pi.set_PWM_frequency(gpio_motor3, pwm_freq)
    pi.set_PWM_frequency(gpio_motor4, pwm_freq)

    printf("PWM's set up @ 50 hz\n")

    # Set max throttle
    pi.set_servo_pulsewidth(gpio_motor1, throttle_max)
    pi.set_servo_pulsewidth(gpio_motor2, throttle_max)
    pi.set_servo_pulsewidth(gpio_motor3, throttle_max)
    pi.set_servo_pulsewidth(gpio_motor4, throttle_max)

    printf("All 4 motors set to max throttle (%d ns)\n", throttle_max)

    # Now ready to power on ESC's
    printf("We are now ready to power on the ESC's\n")
    printf("When you power on the ESC's, you will hear a sequence of 4 single beeps\n")
    printf("Press enter on your keyboard BEFORE the fourth beep. (during the 4 beep period)\n")
    printf("I will then go from the max throttle to the min throttle.\n")
    printf("After this, you will hear a confirmation beep(s) to confirm that the max and min throttle position have been saved.\n")
    printf("Ok, ready to continue?\n")
    printf("1. Plug the power in\n")
    printf("2. Wait for the 4-beep sequence to start\n")
    printf("3. Before the 4th beep, press enter on your keyboard.\n")

    # Min throttle
    sleep(1)  # Simulate waiting for input

    # Send min throttle
    pi.set_servo_pulsewidth(gpio_motor1, throttle_min)
    pi.set_servo_pulsewidth(gpio_motor2, throttle_min)
    pi.set_servo_pulsewidth(gpio_motor3, throttle_min)
    pi.set_servo_pulsewidth(gpio_motor4, throttle_min)

    # Confirmation beeps
    printf("You should now hear confirmation beeps\n")
    printf("After you hear those, your ESC's are calibrated. You may power down :)\n")

    # Program complete
    printf("\n")
    printf("Program complete! Goodbye!\n")

if __name__ == "__main__":
    calibrate()