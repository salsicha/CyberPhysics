#!/usr/bin/env python3
"""
BlueOS ROS 2 Bridge
-------------------
Bridges MAVLink (BlueOS) to ROS 2.
"""

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import BatteryState, Imu
from std_srvs.srv import SetBool

# Try importing pymavlink, handle failure gracefully if not installed
try:
    from pymavlink import mavutil
except ImportError:
    print("pymavlink not found. Please install it with: pip install pymavlink")
    sys.exit(1)

class BlueOSROS2Bridge(Node):
    def __init__(self):
        super().__init__('blueos_ros2_bridge')

        # Parameters
        # Default BlueOS MAVLink endpoint for extensions is often 14550 UDP
        self.declare_parameter('mavlink_connection', 'udpin:0.0.0.0:14550')
        self.declare_parameter('target_system', 1)
        self.declare_parameter('target_component', 1)
        self.declare_parameter('frame_id', 'base_link')

        connection_string = self.get_parameter('mavlink_connection').value
        self.target_system = self.get_parameter('target_system').value
        self.target_component = self.get_parameter('target_component').value
        self.frame_id = self.get_parameter('frame_id').value

        self.get_logger().info(f'Connecting to MAVLink: {connection_string}')
        
        # Initialize MAVLink connection
        try:
            self.master = mavutil.mavlink_connection(connection_string)
            # Wait for a heartbeat to confirm connection
            self.get_logger().info('Waiting for heartbeat...')
            self.master.wait_heartbeat(timeout=10)
            self.get_logger().info(f'Heartbeat received from System {self.master.target_system}, Component {self.master.target_component}')
        except Exception as e:
            self.get_logger().fatal(f'Failed to connect to MAVLink: {e}')
            sys.exit(1)

        # QoS for sensor data (Best Effort for high frequency telemetry)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', sensor_qos)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', sensor_qos)
        self.attitude_pub = self.create_publisher(PoseStamped, 'local_position/pose', sensor_qos)

        # Subscribers
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Services
        self.create_service(SetBool, 'arm', self.arm_callback)
        self.create_service(SetBool, 'set_guided', self.set_guided_callback)

        # Timers
        self.create_timer(0.01, self.run_loop) # 100Hz MAVLink processing
        self.create_timer(1.0, self.send_heartbeat) # 1Hz Heartbeat

        self.get_logger().info('BlueOS ROS 2 Bridge Ready')

    def send_heartbeat(self):
        # Send heartbeat to let BlueOS/Autopilot know we are here
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )

    def run_loop(self):
        # Process all available MAVLink messages
        while True:
            msg = self.master.recv_match(blocking=False)
            if not msg:
                break
            
            msg_type = msg.get_type()
            
            if msg_type == 'ATTITUDE':
                self.handle_attitude(msg)
            elif msg_type == 'SYS_STATUS':
                self.handle_sys_status(msg)
            elif msg_type == 'BATTERY_STATUS':
                self.handle_battery_status(msg)

    def handle_attitude(self, msg):
        # Current time
        now = self.get_clock().now().to_msg()

        # Publish Pose (Orientation)
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = self.frame_id
        
        # Convert Euler (Roll, Pitch, Yaw) to Quaternion
        # MAVLink: Roll/Pitch/Yaw in radians
        q = self.quaternion_from_euler(msg.roll, msg.pitch, msg.yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        self.attitude_pub.publish(pose)

        # Publish IMU
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.frame_id
        imu.orientation = pose.pose.orientation
        
        # Angular velocities (rad/s)
        imu.angular_velocity.x = msg.rollspeed
        imu.angular_velocity.y = msg.pitchspeed
        imu.angular_velocity.z = msg.yawspeed
        
        # Note: Linear acceleration is not in ATTITUDE message. 
        # It is in RAW_IMU or SCALED_IMU, but those might not be streamed by default.
        # We leave linear_acceleration as 0 or covariance -1.
        imu.linear_acceleration_covariance[0] = -1
        
        self.imu_pub.publish(imu)

    def handle_sys_status(self, msg):
        # Fallback battery info if BATTERY_STATUS is not available
        # voltage_battery: mV, current_battery: cA, battery_remaining: %
        bat = BatteryState()
        bat.header.stamp = self.get_clock().now().to_msg()
        bat.voltage = msg.voltage_battery / 1000.0
        bat.current = msg.current_battery / 100.0
        bat.percentage = float(msg.battery_remaining) / 100.0
        self.battery_pub.publish(bat)

    def handle_battery_status(self, msg):
        # Preferred over SYS_STATUS if available
        bat = BatteryState()
        bat.header.stamp = self.get_clock().now().to_msg()
        
        v = 0.0
        if len(msg.voltages) > 0:
             v = msg.voltages[0]
             # If it's UINT16_MAX, it's invalid.
             if v == 65535: v = 0
        
        bat.voltage = v / 1000.0
        bat.current = msg.current_battery / 100.0 if msg.current_battery != -1 else 0.0
        bat.percentage = float(msg.battery_remaining) / 100.0 if msg.battery_remaining != -1 else 0.0
        self.battery_pub.publish(bat)

    def cmd_vel_callback(self, msg):
        """
        Convert ROS Twist (FLU) to MAVLink SET_POSITION_TARGET_LOCAL_NED (Body NED)
        ROS X (Forward) -> MAV X (Forward)
        ROS Y (Left)    -> MAV Y (Right) = -ROS Y
        ROS Z (Up)      -> MAV Z (Down)  = -ROS Z
        ROS Yaw (CCW)   -> MAV Yaw (CW)  = -ROS Yaw
        """
        
        # MAV_FRAME_BODY_NED = 8
        coordinate_frame = mavutil.mavlink.MAV_FRAME_BODY_NED
        
        # Type mask to ignore everything except velocities and yaw rate
        # We want to control: vx, vy, vz, yaw_rate
        # Ignore: x, y, z (1,2,4), ax, ay, az (64,128,256), yaw (1024)
        # Mask = 1+2+4 + 64+128+256 + 1024 = 1479
        type_mask = 1479

        self.master.mav.set_position_target_local_ned_send(
            0, # time_boot_ms
            self.target_system,
            self.target_component,
            coordinate_frame,
            type_mask,
            0, 0, 0, # x, y, z (ignored)
            msg.linear.x,
            -msg.linear.y,
            -msg.linear.z,
            0, 0, 0, # accel (ignored)
            0, # yaw (ignored)
            -msg.angular.z # yaw_rate
        )

    def arm_callback(self, request, response):
        arm_cmd = 1 if request.data else 0
        self.get_logger().info(f"Sending {'Arm' if arm_cmd else 'Disarm'} command")
        
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            arm_cmd, 0, 0, 0, 0, 0, 0
        )
        
        # Ideally we should wait for ACK, but for simple bridge we just send
        response.success = True
        response.message = f"Sent {'Arm' if arm_cmd else 'Disarm'} command"
        return response

    def set_guided_callback(self, request, response):
        if request.data:
            self.get_logger().info("Switching to GUIDED mode")
            try:
                # Check if we have mode mapping, otherwise default to ArduCopter 'GUIDED' (4)
                if 'GUIDED' in self.master.mode_mapping():
                    mode_id = self.master.mode_mapping()['GUIDED']
                    self.master.set_mode(mode_id)
                    response.success = True
                    response.message = "Sent GUIDED mode command"
                else:
                    response.success = False
                    response.message = "GUIDED mode not found in mode mapping"
            except Exception as e:
                response.success = False
                response.message = f"Failed to set mode: {e}"
        else:
             response.success = False
             response.message = "Request was false"
        return response

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion [x, y, z, w]
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [x, y, z, w]

def main(args=None):
    rclpy.init(args=args)
    node = BlueOSROS2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()