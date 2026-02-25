#!/usr/bin/env python3
import argparse
import numpy as np
import genesis as gs
import time
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

class RosDroneController(Node):
    def __init__(self):
        super().__init__('genesis_drone_bridge')
        
        # Subscribe to velocity commands
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # Publisher for TF (drone state)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Control parameters
        self.base_thrust = 14468.429183500699
        self.rpms = np.array([self.base_thrust] * 4)
        
        # Gains to map Twist velocities to RPM offsets
        # Tuned relative to interactive_drone.py values
        self.pitch_gain = 200.0  # RPM per unit of linear.x
        self.roll_gain = 200.0   # RPM per unit of linear.y
        self.yaw_gain = 100.0    # RPM per unit of angular.z
        self.thrust_gain = 500.0 # RPM per unit of linear.z

    def cmd_vel_callback(self, msg):
        # Start with base hover thrust
        rpms = np.array([self.base_thrust] * 4)
        
        # Vertical Thrust (linear.z)
        thrust_delta = msg.linear.z * self.thrust_gain
        rpms += thrust_delta
        
        # Pitch / Forward-Backward (linear.x)
        # Matches interactive_drone.py: Forward -> Front rotors +, Back rotors -
        pitch_delta = msg.linear.x * self.pitch_gain
        rpms[0] += pitch_delta # Front Left
        rpms[1] += pitch_delta # Front Right
        rpms[2] -= pitch_delta # Back Left
        rpms[3] -= pitch_delta # Back Right
        
        # Roll / Left-Right (linear.y)
        # Matches interactive_drone.py: Left -> Right rotors +, Left rotors -
        roll_delta = msg.linear.y * self.roll_gain
        rpms[0] -= roll_delta # Front Left
        rpms[2] -= roll_delta # Back Left
        rpms[1] += roll_delta # Front Right
        rpms[3] += roll_delta # Back Right
        
        # Yaw (angular.z)
        # Standard Quad X configuration: FL(CW), FR(CCW), BL(CCW), BR(CW)
        # Yaw Left (positive) -> Increase CCW (FR, BL), Decrease CW (FL, BR)
        yaw_delta = msg.angular.z * self.yaw_gain
        rpms[0] -= yaw_delta
        rpms[1] += yaw_delta
        rpms[2] += yaw_delta
        rpms[3] -= yaw_delta
        
        self.rpms = np.clip(rpms, 0, 25000)

    def publish_state(self, drone_entity):
        try:
            # Get state from Genesis
            pos = drone_entity.get_pos()
            quat = drone_entity.get_quat() # [w, x, y, z]
            
            # Handle tensor conversion if necessary
            if hasattr(pos, 'cpu'): pos = pos.cpu().numpy()
            if hasattr(quat, 'cpu'): quat = quat.cpu().numpy()

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = 'base_link'
            
            t.transform.translation.x = float(pos[0])
            t.transform.translation.y = float(pos[1])
            t.transform.translation.z = float(pos[2])
            
            # Genesis quat is typically [w, x, y, z]
            t.transform.rotation.w = float(quat[0])
            t.transform.rotation.x = float(quat[1])
            t.transform.rotation.y = float(quat[2])
            t.transform.rotation.z = float(quat[3])
            
            self.tf_broadcaster.sendTransform(t)
        except Exception:
            pass # Handle initialization transients

def run_sim(scene, drone, ros_node):
    while rclpy.ok():
        # Process ROS callbacks
        rclpy.spin_once(ros_node, timeout_sec=0)
        
        # Update drone RPMs based on ROS commands
        drone.set_propellels_rpm(ros_node.rpms)
        
        # Step Physics
        scene.step()
        
        # Publish State
        ros_node.publish_state(drone)
        
        # Control simulation rate
        time.sleep(0.01)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--vis", action="store_true", default=True, help="Enable visualization")
    parser.add_argument("-m", "--mac", action="store_true", default=False, help="Running on MacOS")
    args = parser.parse_args()

    # Init ROS 2
    rclpy.init()
    ros_node = RosDroneController()

    # Init Genesis
    gs.init(backend=gs.cpu)

    viewer_options = gs.options.ViewerOptions(
        camera_pos=(0.0, -4.0, 2.0),
        camera_lookat=(0.0, 0.0, 0.5),
        camera_fov=45,
        max_FPS=60,
    )

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=0.01,
            gravity=(0, 0, -9.81),
        ),
        viewer_options=viewer_options,
        show_viewer=args.vis,
    )

    # Add entities
    scene.add_entity(gs.morphs.Plane())
    drone = scene.add_entity(
        morph=gs.morphs.Drone(
            file="urdf/drones/cf2x.urdf",
            pos=(0.0, 0, 0.5),
        ),
    )
    
    if args.vis:
        scene.viewer.follow_entity(drone)

    scene.build()

    print("ROS 2 Genesis Drone Bridge Started")
    print("----------------------------------")
    print("Subscribe to: /tf (base_link)")
    print("Publish to:   /cmd_vel (Twist)")
    print("----------------------------------")

    try:
        if args.mac:
            # Run simulation in another thread for MacOS compatibility
            sim_thread = threading.Thread(target=run_sim, args=(scene, drone, ros_node))
            sim_thread.start()
            if args.vis:
                scene.viewer.start()
            sim_thread.join()
        else:
            run_sim(scene, drone, ros_node)
            
    except KeyboardInterrupt:
        pass
    finally:
        if scene.viewer:
            scene.viewer.stop()
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
