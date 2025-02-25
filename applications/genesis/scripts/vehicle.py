
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

base_rpm = 14468.429183500699
min_rpm = 0.9 * base_rpm
max_rpm = 1.5 * base_rpm


###
kp = [2.0, 2.0, 2.0, 20.0, 20.0, 25.0, 10.0, 10.0, 2.0]
ki = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
kd = [0.0, 0.0, 0.0, 20.0, 20.0, 20.0, 1.0, 1.0, 0.2]
mass = 1.50
gravity = 9.81
rotation = Rotation.identity()
###


def hover(drone: DroneEntity):
    drone.set_propellels_rpm([base_rpm, base_rpm, base_rpm, base_rpm])


def clamp(rpm):
    return max(min_rpm, min(int(rpm), max_rpm))


def fly_to_point(target, controller: DronePIDController, scene: gs.Scene, cam: Camera):
    drone = controller.drone
    step = 0
    x = target[0] - drone.get_pos()[0]
    y = target[1] - drone.get_pos()[1]
    z = target[2] - drone.get_pos()[2]

    distance = math.sqrt(x**2 + y**2 + z**2)

    while distance > 0.1 and step < 1000:


        # TODO:
        # Kp, Kd, Ki params are in pid_params below

        ### Compute an approximation of the current vehicle acceleration in the inertial frame (since we cannot measure it directly)
        # Kp=[10.0, 10.0, 10.0]
        # Kd=[8.5, 8.5, 8.5]
        # Ki=[1.50, 1.50, 1.50]
        # self.Kp = np.diag(Kp)
        # self.Kd = np.diag(Kd)
        # self.Ki = np.diag(Ki)
        # self.total_time += dt # running total
        # s = 0.6
        # self.reverse = False
        ### Desired acceleration (self.dd_pd())
        # a_ref = self.dd_pd(self.total_time, s, self.reverse)
        # F_des = -(self.Kp @ ep) - (self.Kd @ ev) - (self.Ki @ ei) + np.array([0.0, 0.0, self.m * self.g]) + (self.m * a_ref)
        # Z_B = self.R.as_matrix()[:,2]
        ### u_1 = F_des @ Z_B
        # self.R: Rotation = Rotation.identity() # The vehicle attitude
        ### Z_B = self.R.as_matrix()[:,2]
        ### self.m = 1.50
        ### self.g = 9.81
        ### 
        ### self.a = (u_1 * Z_B) / mass - g)


        # TODO: publish acceleration


        # p_ref = 
        # v_ref = 
        # int = np.array([0.0, 0.0, 0.0])
        # dt = 0.05 # time between updates
        # ep = state.position - p_ref
        # ev = state.linear_velocity - v_ref
        # ei = int + (ep * dt)
        # F_des = -(kp @ ep) - (kd @ ev) - (ki @ ei) + np.array([0.0, 0.0, mass * gravity]) + (mass * a_ref)
        # u_1 = F_des @ Z_B
        # Z_B = rotation.as_matrix()[:,2]
        # g = np.array([0.0, 0.0, gravity])
        # acc = (u_1 * Z_B) / mass - g



        [M1, M2, M3, M4] = controller.update(target)
        M1 = clamp(M1)
        M2 = clamp(M2)
        M3 = clamp(M3)
        M4 = clamp(M4)
        drone.set_propellels_rpm([M1, M2, M3, M4])
        print("drone: ", dir(drone))
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
    scene = gs.Scene(show_viewer=False, sim_options=gs.options.SimOptions(dt=0.01))

    ##### entities #####
    plane = scene.add_entity(morph=gs.morphs.Plane())

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

    cam.start_recording()

    points = [(1, 1, 2), (-1, 2, 1), (0, 0, 0.5)]

    for point in points:
        fly_to_point(point, controller, scene, cam)

    cam.stop_recording(save_to_filename="../../videos/fly_route.mp4")


if __name__ == "__main__":
    main()
