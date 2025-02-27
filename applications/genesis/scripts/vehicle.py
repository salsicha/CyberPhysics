
### Instructions:
### https://pegasussimulator.github.io/PegasusSimulator/source/tutorials/create_custom_backend.html


# TODO:
# Follow example in single_vehicle.py to create fake Imu data
#  - look for where state is extracted
#  - do accel calculation
# Publish Imu from here as ROS topic
# Extra, if possible extract optical flow from down facing camera and publish that



import genesis as gs
import math
from quadcopter_controller import DronePIDController
from genesis.engine.entities.drone_entity import DroneEntity
from genesis.vis.camera import Camera
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import Header
from geometry_msgs.msg import (Quaternion, Vector3)


base_rpm = 14468.429183500699
min_rpm = 0.9 * base_rpm
max_rpm = 1.5 * base_rpm

kp = [2.0, 2.0, 2.0, 20.0, 20.0, 25.0, 10.0, 10.0, 2.0]
ki = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
kd = [0.0, 0.0, 0.0, 20.0, 20.0, 20.0, 1.0, 1.0, 0.2]

RPM_1 = 50
RPM_2 = 50
RPM_3 = 50
RPM_4 = 50

def rpm_callback(msg):
    # print("rpm: ", msg.data)

    global RPM_1
    global RPM_2
    global RPM_3
    global RPM_4

    RPM_1 = msg.data.rpm1
    RPM_2 = msg.data.rpm2
    RPM_3 = msg.data.rpm3
    RPM_4 = msg.data.rpm4

    # TODO: set RPMs of simulated motors
    # [M1, M2, M3, M4] = controller.update(target)
    # M1 = clamp(M1)
    # M2 = clamp(M2)
    # M3 = clamp(M3)
    # M4 = clamp(M4)
    # drone.set_propellels_rpm([M1, M2, M3, M4])


# TODO: 
# Subscribe IMU
# Publish RPMs
rclpy.init()
node = Node('subscriber_node')
rpm_sub = node.create_subscription(Float32MultiArray, 'rpms', rpm_callback, 10)
imu_pub = node.create_publisher(Imu, 'imu_data', 10)


# def hover(drone: DroneEntity):
#     drone.set_propellels_rpm([base_rpm, base_rpm, base_rpm, base_rpm])


def clamp(rpm):
    return max(min_rpm, min(int(rpm), max_rpm))


def fly_to_point(target, controller: DronePIDController, scene: gs.Scene, cam: Camera):

    # Publish IMU
    # drone_pos = drone.get_pos()
    global imu_pub
    imu_msg = Imu()
    accel = Vector3()
    # accel.x = drone_pos
    # accel.y = drone_pos
    # accel.z = drone_pos
    imu_msg.linear_acceleration = accel
    imu_msg.linear_acceleration_covariance[0] = 0.00001
    imu_msg.linear_acceleration_covariance[4] = 0.00001
    imu_msg.linear_acceleration_covariance[8] = 0.00001
    gyro = Vector3()
    # gyro.x = drone_pos
    # gyro.y = drone_pos
    # gyro.z = drone_pos
    imu_msg.angular_velocity = gyro
    imu_msg.angular_velocity_covariance[0] = 0.00001
    imu_msg.angular_velocity_covariance[4] = 0.00001
    imu_msg.angular_velocity_covariance[8] = 0.00001
    # imu_msg.orientation[w] = 0.0
    # imu_msg.orientation[x] = 0.0
    # imu_msg.orientation[y] = 0.0
    # imu_msg.orientation[z] = 0.0
    imu_msg.orientation_covariance[0] = 0.00001
    imu_msg.orientation_covariance[4] = 0.00001
    imu_msg.orientation_covariance[8] = 0.00001
    imu_msg.header.stamp = node.get_clock().now().to_msg()
    imu_msg.header.frame_id = "imu"
    imu_pub.publish(imu_msg)


    drone = controller.drone
    step = 0
    x = target[0] - drone.get_pos()[0]
    y = target[1] - drone.get_pos()[1]
    z = target[2] - drone.get_pos()[2]

    distance = math.sqrt(x**2 + y**2 + z**2)

    while distance > 0.1 and step < 1000:
        [M1, M2, M3, M4] = controller.update(target)
        M1 = clamp(M1)
        M2 = clamp(M2)
        M3 = clamp(M3)
        M4 = clamp(M4)
        drone.set_propellels_rpm([M1, M2, M3, M4])
        # print("drone: ", drone.get_dofs_force())
        scene.step()
        cam.render()
        # print("point =", drone.get_pos())
        drone_pos = drone.get_pos()
        drone_pos = drone_pos.cpu().numpy()
        x = drone_pos[0]
        y = drone_pos[1]
        z = drone_pos[2]
        cam.set_pose(lookat=(x, y, z))
        x = target[0] - x
        y = target[0] - y
        z = target[0] - z
        distance = math.sqrt(x**2 + y**2 + z**2)
        step += 1


def main():

    gs.init(backend=gs.gpu)

    ##### scene #####
    scene = gs.Scene(show_viewer=True, sim_options=gs.options.SimOptions(dt=0.01))

    ##### entities #####
    # plane = scene.add_entity(morph=gs.morphs.Plane())

    drone = scene.add_entity(morph=gs.morphs.Drone(file="urdf/drones/cf2x.urdf", pos=(0, 0, 0.2)))

    # parameters are tuned such that the
    # drone can fly, not optimized
    pid_params = [
        [kp[0], ki[0], kd[0]],
        [kp[1], ki[1], kd[1]],
        [kp[2], ki[2], kd[2]],
        [kp[3], ki[3], kd[3]],
        [kp[4], ki[4], kd[4]],
        [kp[5], ki[5], kd[5]],
        [kp[6], ki[6], kd[6]],
        [kp[7], ki[7], kd[7]],
        [kp[8], ki[8], kd[8]],
    ]

    controller = DronePIDController(drone=drone, dt=0.01, base_rpm=base_rpm, pid_params=pid_params)

    cam = scene.add_camera(pos=(1, 1, 1), lookat=drone.morph.pos, GUI=False, res=(640, 480), fov=30)

    ##### build #####

    scene.build()

    # cam.start_recording()

    points = [(1, 1, 2), (-1, 2, 1), (0, 0, 0.5)]

    for point in points:
        fly_to_point(point, controller, scene, cam)

    # cam.stop_recording(save_to_filename="../../videos/fly_route.mp4")


if __name__ == "__main__":
    main()
