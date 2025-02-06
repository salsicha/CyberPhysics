import RPi.GPIO as GPIO
import time
import pigpio

# Motor GPIO's (not pin number, GPIO number) ###
gpio_motor1 = 28 # front left, clockwise
gpio_motor2 = 2    # front right, counter clockwise
gpio_motor3 = 16   # rear left, counter clockwise
gpio_motor4 = 15   # rear right, clockwise

# min and max throttle (nanoseconds)
throttle_max = 2000000
throttle_min = 1000000

def calibrate() -> None:
    print("Hello! Welcome to the ESC throttle range calibration script")
    print("The Raspberry Pi should be running on SEPARATE power from the ESC's")
    print("This is because the Raspberry Pi needs to be able to set the throttle to 100% BEFORE the ESC's gain power")
    print("To accomplish this, the Raspberry Pi will need to be on a different power source (USB) while the ESC's are powered by the LiPo.")
    print("MAKE SURE you have removed the +5V BEC circuit from the VSYS of the pico before continuing.")
    print("Please ensure that right now the ESC's are NOT powered on")
    print("Press enter to confirm this and when you are ready")
    input("")

    # Set up GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(gpio_motor1, GPIO.OUT)
    GPIO.setup(gpio_motor2, GPIO.OUT)
    GPIO.setup(gpio_motor3, GPIO.OUT)
    GPIO.setup(gpio_motor4, GPIO.OUT)

    print("GPIO's set up")

    # Set up PWM
    pi = pigpio.pi()
    pi.set_mode(gpio_motor1, pigpio.OUTPUT)
    pi.set_mode(gpio_motor2, pigpio.OUTPUT)
    pi.set_mode(gpio_motor3, pigpio.OUTPUT)
    pi.set_mode(gpio_motor4, pigpio.OUTPUT)

    pwm_freq = 50  # ESC's typically use a frequency of 50 Hz
    pi.set_PWM_frequency(gpio_motor1, pwm_freq)
    pi.set_PWM_frequency(gpio_motor2, pwm_freq)
    pi.set_PWM_frequency(gpio_motor3, pwm_freq)
    pi.set_PWM_frequency(gpio_motor4, pwm_freq)

    print("PWM's set up @ 50 hz")

    # Set max throttle
    pi.set_servo_pulsewidth(gpio_motor1, throttle_max)
    pi.set_servo_pulsewidth(gpio_motor2, throttle_max)
    pi.set_servo_pulsewidth(gpio_motor3, throttle_max)
    pi.set_servo_pulsewidth(gpio_motor4, throttle_max)

    print("All 4 motors set to max throttle (" + str(throttle_max) + " ns)")

    # Now ready to power on ESC's
    print("We are now ready to power on the ESC's")
    print("When you power on the ESC's, you will hear a sequence of 4 single beeps")
    print("Press enter on your keyboard BEFORE the fourth beep. (during the 4 beep period).")
    print("I will then go from the max throttle to the min throttle.")
    print("After this, you will hear a confirmation beep(s) to confirm that the max and min throttle position have been saved.")
    print("Ok, ready to continue?")
    print("1. Plug the power in")
    print("2. Wait for the 4-beep sequence to start")
    print("3. Before the 4th beep, press enter on your keyboard.")

    # Min throttle
    input("Hit ENTER before the fourth beep! Waiting for enter...")

    # Send min throttle
    pi.set_servo_pulsewidth(gpio_motor1, throttle_min)
    pi.set_servo_pulsewidth(gpio_motor2, throttle_min)
    pi.set_servo_pulsewidth(gpio_motor3, throttle_min)
    pi.set_servo_pulsewidth(gpio_motor4, throttle_min)

    # Confirmation beeps
    print("You should now hear confirmation beeps")
    print("After you hear those, your ESC's are calibrated. You may power down :)")

    # Program complete
    print("")
    print("Program complete! Goodbye!")

if __name__ == "__main__":
    calibrate()