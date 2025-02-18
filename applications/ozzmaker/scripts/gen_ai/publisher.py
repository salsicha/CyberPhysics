import smbus2
import time
from rcl_interfaces.msg import Time
from sensor_msgs.msg import Imu, NavSatFix
import gps
import numpy as np

# Constants
I2C_ADDRESS_MMC5983MA = 0x1E
I2C_ADDRESS_LSM6DSL = 0x6A
GPS_PORT = "/dev/ttyUSB0"
UPDATE_RATE = 200  # Hz

# Initialize I2C bus
bus = smbus2.SMBus(1)  # Use 1 for /dev/i2c-1

def read_mmc5983ma():
    """Read accelerometer and gyroscope data from MMC5983MA."""
    try:
        accel_x = bus.read_word_data(I2C_ADDRESS_MMC5983MA, 0x6E)
        accel_y = bus.read_word_data(I2C_ADDRESS_MMC5983MA, 0x70)
        accel_z = bus.read_word_data(I2C_ADDRESS_MMC5983MA, 0x72)

        gyro_x = bus.read_word_data(I2C_ADDRESS_LSM6DSL, 0x22)
        gyro_y = bus.read_word_data(I2C_ADDRESS_LSM6DSL, 0x24)
        gyro_z = bus.read_word_data(I2C_ADDRESS_LSM6DSL, 0x26)

        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
    except smbus2.SMBusError as e:
        print(f"Failed to read from MMC5983MA: {e}")
        return None

def read_gps():
    """Read GPS data."""
    session = gps.gps("localhost", "2947")
    session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

    try:
        report = session.next()
        while True:
            if report['class'] == 'TPV':
                if hasattr(report, 'lat') and hasattr(report, 'lon'):
                    return report.lat, report.lon, report.time
            report = session.next()
    except KeyboardInterrupt:
        session.stream(gps.WATCH_DISABLE)
        return None

def main():
    # Initialize ROS2 node
    import rclpy
    from rclpy.node import Node
    rclpy.init()

    class OzzMakerPublisher(Node):
        def __init__(self):
            super().__init__('ozzmaker_publisher')
            self.imu_pub = self.create_publisher(Imu, 'imu_data', 10)
            self.gps_pub = self.create_publisher(NavSatFix, 'gps_data', 10)
            self.timer = self.create_timer(1.0 / UPDATE_RATE, self.timer_callback)

        def timer_callback(self):
            imu_data = read_mmc5983ma()
            if imu_data:
                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = imu_data

                imu_msg = Imu()
                imu_msg.header.stamp = Time(seconds=int(time.time()))
                imu_msg.linear_acceleration.x = np.frombuffer(struct.pack('>HHH', accel_x, accel_y, accel_z), dtype=np.float32)
                imu_msg.angular_velocity.x = np.frombuffer(struct.pack('>hhh', gyro_x, gyro_y, gyro_z), dtype=np.float32)

                self.imu_pub.publish(imu_msg)

            gps_data = read_gps()
            if gps_data:
                lat, lon, time_str = gps_data

                gps_msg = NavSatFix()
                gps_msg.header.stamp = Time(seconds=int(time.time()))
                gps_msg.latitude = lat
                gps_msg.longitude = lon
                gps_msg.status.service = 1  # Assuming GPS service is active
                gps_msg.status.status = 0  # Assuming GPS fix is good

                self.gps_pub.publish(gps_msg)

    ozz_publisher_node = OzzMakerPublisher()
    rclpy.spin(ozz_publisher_node)
    ozz_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()