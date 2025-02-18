import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import Imu
from rcl_interfaces.msg import SetParametersResult

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Subscriptions
        self.feature_tracks_sub = self.create_subscription(
            PoseStamped, 'feature_tracks', self.feature_tracks_callback, 10)
        
        self.imu_sub = self.create_subscription(
            Imu, 'imu', self.imu_callback, 10)
        
        # Publisher
        self.velocity_pub = self.create_publisher(
            TwistStamped, 'fused_velocity', 10)

        self.last_imu_timestamp = None

    def feature_tracks_callback(self, msg):
        """
        Callback for handling the Feature Tracks message.
        """
        # Assume this is a 2D PoseStamped (x, y, z)
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        # For demonstration purposes, let's assume we have velocity in x,y,z from feature tracks
        self.velocity_x += (x - self.last_feature_track_x) * 10  # Assuming dt = 0.1s for simplicity
        self.velocity_y += (y - self.last_feature_track_y) * 10
        self.velocity_z += (z - self.last_feature_track_z) * 10
        
        self.last_feature_track_x = x
        self.last_feature_track_y = y
        self.last_feature_track_z = z

    def imu_callback(self, msg):
        """
        Callback for handling the IMU message.
        """
        current_timestamp = msg.header.stamp
        
        if self.last_imu_timestamp is not None:
            dt = (current_timestamp.sec - self.last_imu_timestamp.sec) + \
                 (current_timestamp.nanosec - self.last_imu_timestamp.nanosec) * 1e-9
            
            # Extract angular velocities from the IMU message
            angular_x = msg.angular_velocity.x
            angular_y = msg.angular_velocity.y
            angular_z = msg.angular_velocity.z

            # Integrate angular velocities to get orientation (simplified)
            self.orientation_x += angular_x * dt
            self.orientation_y += angular_y * dt
            self.orientation_z += angular_z * dt
            
        self.last_imu_timestamp = current_timestamp
        
    def publish_velocity(self):
        """
        Publish the fused velocity as a TwistStamped message.
        """
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        
        twist_stamped_msg.twist.linear.x = self.velocity_x
        twist_stamped_msg.twist.linear.y = self.velocity_y
        twist_stamped_msg.twist.linear.z = self.velocity_z
        
        self.velocity_pub.publish(twist_stamped_msg)

    def timer_callback(self):
        """
        Timer callback to publish fused velocity.
        """
        self.publish_velocity()

def main(args=None):
    rclpy.init(args=args)
    
    sensor_fusion_node = SensorFusionNode()
    
    # Set up a timer callback to control the publishing frequency
    timer_period = 0.1  # seconds
    timer = sensor_fusion_node.create_timer(timer_period, sensor_fusion_node.timer_callback)
    
    rclpy.spin(sensor_fusion_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor_fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()