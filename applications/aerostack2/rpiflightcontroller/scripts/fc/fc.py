
import rclpy

from rclpy.node import Node


from std_msgs.msg import Empty

from as2_msgs.msg import PlatformState

from geometry_msgs.msg import TwistStamped


import time

import board

import busio

import adafruit_icm20x

import pigpio



class FlightController(Node):

    def __init__(self):

        super().__init__('as2_flight_controller')


        # Motor pins

        self.motor_pins = [17, 18, 27, 22]

        self.pi = pigpio.pi()

        for pin in self.motor_pins:

            self.pi.set_mode(pin, pigpio.OUTPUT)

            self.pi.set_servo_pulsewidth(pin, 0)


        # IMU setup

        i2c = busio.I2C(board.SCL, board.SDA)

        self.imu = adafruit_icm20x.ICM20948(i2c)


        # Command/state variables

        self.armed = False

        self.base_throttle = 1400

        self.velocity_command = [0.0, 0.0, 0.0]  # vx, vy, vz


        # Subscribers

        self.create_subscription(Empty, '/takeoff', self.takeoff_cb, 10)

        self.create_subscription(Empty, '/land', self.land_cb, 10)

        self.create_subscription(TwistStamped, '/motion_reference/velocity', self.velocity_cb, 10)


        # Publishers

        self.state_pub = self.create_publisher(PlatformState, '/drone_state', 10)


        # Control loop

        self.control_timer = self.create_timer(0.01, self.control_loop)


    def takeoff_cb(self, msg):

        self.get_logger().info("Takeoff received.")

        self.armed = True


    def land_cb(self, msg):

        self.get_logger().info("Landing.")

        self.armed = False

        self.stop_motors()


    def velocity_cb(self, msg: TwistStamped):

        vx = msg.twist.linear.x

        vy = msg.twist.linear.y

        vz = msg.twist.linear.z

        self.velocity_command = [vx, vy, vz]


    def stop_motors(self):

        for pin in self.motor_pins:

            self.pi.set_servo_pulsewidth(pin, 0)


    def control_loop(self):

        if self.armed:

            # Basic throttle adjustment (placeholder logic)

            vz = self.velocity_command[2]

            throttle = self.base_throttle + int(vz * 100)


            motor_pwms = [throttle] * 4

            for pin, pwm in zip(self.motor_pins, motor_pwms):

                self.pi.set_servo_pulsewidth(pin, max(1000, min(2000, pwm)))


        # Always publish telemetry

        self.publish_state()


    def publish_state(self):

        imu_data = self.imu.acceleration + self.imu.gyro

        state = PlatformState()


        # Simple feedback: just linear velocity as sensed acceleration

        state.twist.linear.x = imu_data[0]

        state.twist.linear.y = imu_data[1]

        state.twist.linear.z = imu_data[2]


        # Angular velocity

        state.twist.angular.x = imu_data[3]

        state.twist.angular.y = imu_data[4]

        state.twist.angular.z = imu_data[5]


        # Armed state

        state.status.is_armed = self.armed

        state.status.is_flying = self.armed


        self.state_pub.publish(state)



def main(args=None):

    rclpy.init(args=args)

    node = FlightController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


