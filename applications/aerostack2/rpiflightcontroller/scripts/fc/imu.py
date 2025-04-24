
import rclpy

from rclpy.node import Node


from sensor_msgs.msg import Imu, Temperature, MagneticField, FluidPressure

from std_msgs.msg import Float32

from geometry_msgs.msg import Quaternion


import board

import busio

import adafruit_icm20x

import adafruit_bme280


class SARA_R5_SensorNode(Node):

    def __init__(self):

        super().__init__('sara_r5_sensor_node')


        # I2C bus and sensors

        i2c = busio.I2C(board.SCL, board.SDA)

        self.icm = adafruit_icm20x.ICM20948(i2c)

        self.bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)


        # ROS publishers

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)

        self.temp_pub = self.create_publisher(Temperature, 'temperature', 10)

        self.pressure_pub = self.create_publisher(FluidPressure, 'barometer/pressure', 10)

        self.alt_pub = self.create_publisher(Float32, 'barometer/altitude', 10)


        self.timer = self.create_timer(0.01, self.publish_sensor_data)  # 100 Hz


    def publish_sensor_data(self):

        imu_msg = Imu()

        mag_msg = MagneticField()

        temp_msg = Temperature()

        pressure_msg = FluidPressure()

        alt_msg = Float32()


        # Orientation not available

        imu_msg.orientation = Quaternion()

        imu_msg.orientation_covariance[0] = -1


        # Gyroscope (rad/s)

        gx, gy, gz = self.icm.gyro

        imu_msg.angular_velocity.x = gx

        imu_msg.angular_velocity.y = gy

        imu_msg.angular_velocity.z = gz


        # Accelerometer (m/s²)

        ax, ay, az = self.icm.acceleration

        imu_msg.linear_acceleration.x = ax

        imu_msg.linear_acceleration.y = ay

        imu_msg.linear_acceleration.z = az


        # Magnetometer (µT)

        mx, my, mz = self.icm.magnetic

        mag_msg.magnetic_field.x = mx

        mag_msg.magnetic_field.y = my

        mag_msg.magnetic_field.z = mz


        # Temperature (°C)

        temp_msg.temperature = self.bme280.temperature

        temp_msg.variance = 0.0


        # Barometric pressure (Pa)

        pressure_msg.fluid_pressure = self.bme280.pressure * 100  # hPa to Pa

        pressure_msg.variance = 0.0


        # Altitude (m)

        alt_msg.data = self.bme280.altitude


        # Publish all messages

        self.imu_pub.publish(imu_msg)

        self.mag_pub.publish(mag_msg)

        self.temp_pub.publish(temp_msg)

        self.pressure_pub.publish(pressure_msg)

        self.alt_pub.publish(alt_msg)


def main(args=None):

    rclpy.init(args=args)

    node = SARA_R5_SensorNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()

