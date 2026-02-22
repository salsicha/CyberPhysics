#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_tutorials/raw/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Behaviours for the tutorials.
"""

##############################################################################
# Imports
##############################################################################

from threading import current_thread
import py_trees
import py_trees_ros
import rcl_interfaces.msg as rcl_msgs
import rcl_interfaces.srv as rcl_srvs
import rclpy
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
from dronekit import (
    connect,
    VehicleMode,
    LocationGlobalRelative,
    LocationGlobal,
    Command,
)
import math
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from numpy import linalg as LA
import dronekit_funcs
import numpy as np
import time
from scipy.spatial.transform import Rotation as R

from rclpy.clock import ROSClock
from rclpy.time import Time

import traceback

##############################################################################
# Behaviours
##############################################################################


class Arm(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, vehicle):
        super().__init__(name=name)
        self.logger.debug("%s.init()" % self.__class__.__name__)
        self.name = name
        self.vehicle = vehicle

    def setup(self, **kwargs):
        self.logger.debug("%s.setup()" % self.__class__.__name__)

    def initialise(self):
        self.logger.debug("%s.initialise()" % self.__class__.__name__)

    def update(self) -> py_trees.common.Status:
        self.logger.debug("%s.update()" % self.__class__.__name__)

        self.arm_vehicle()

        if not self.vehicle.armed:
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status):
        self.logger.debug("%s.terminate()" % self.__class__.__name__)

    def arm_vehicle(self):
        self.vehicle.mode = VehicleMode("GUIDED")

        # Wait until mode has changed
        while not self.vehicle.mode.name == "GUIDED":
            print(" Waiting for mode change ...")
            time.sleep(1)

        # Check that vehicle is armable
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        self.vehicle.armed = True
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)





class Liftoff(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, blackboard, vehicle, aTargetAltitude=10):
        super().__init__(name=name)
        self.logger.debug("%s.init()" % self.__class__.__name__)
        self.name = name
        self.vehicle = vehicle
        self.aTargetAltitude = aTargetAltitude
        self.takeoff_called = False
        self.start_location = None
        self.blackboard = blackboard

    def setup(self, **kwargs):
        self.logger.debug("%s.setup()" % self.__class__.__name__)

    def initialise(self):
        self.logger.debug("%s.initialise()" % self.__class__.__name__)

    def update(self) -> py_trees.common.Status:
        self.logger.debug("%s.update()" % self.__class__.__name__)

        self.call_takeoff()

        if (
            self.vehicle.location.global_relative_frame.alt
            >= self.aTargetAltitude * 0.95
        ):
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        self.logger.debug("%s.terminate()" % self.__class__.__name__)

    def call_takeoff(self):
        if not self.takeoff_called:
            self.vehicle.simple_takeoff(self.aTargetAltitude)
            self.takeoff_called = True
            self.blackboard.location = self.vehicle.location.global_frame
            self.blackboard.start_location = self.vehicle.location.global_frame

            # print(f"{py_trees.display.unicode_blackboard()}")
            print(f"location : {self.blackboard.get('location')}")


class CollisionCheck(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, blackboard, vehicle, update_rate):
        super().__init__(name=name)
        self.logger.debug("%s.init()" % self.__class__.__name__)
        self.name = name
        self.vehicle = vehicle
        self.blackboard = blackboard

        self.node = rclpy.create_node("pc2_pub")
        self.publisher = self.node.create_publisher(PointCloud2, "pc_slice", 10)

        break_time = 2.0  # seconds
        self.time_limit = break_time / update_rate
        self.timeout = self.time_limit

    def setup(self, **kwargs):
        self.logger.debug("%s.setup()" % self.__class__.__name__)

    def initialise(self):
        self.logger.debug("%s.initialise()" % self.__class__.__name__)

    def update(self) -> py_trees.common.Status:
        self.logger.debug("%s.update()" % self.__class__.__name__)

        print("mode: ", self.vehicle.mode.name)
        if self.vehicle.mode.name == "BRAKE":
            if self.timeout == 0:
                self.blackboard.next_location = None
                self.vehicle.mode = VehicleMode("GUIDED")
                return py_trees.common.Status.SUCCESS
            else:
                self.timeout -= 1
                return py_trees.common.Status.RUNNING

        if self.avoid_collision():
            self.vehicle.mode = VehicleMode("BRAKE")
            self.timeout = self.time_limit
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status):
        self.logger.debug("%s.terminate()" % self.__class__.__name__)

    def avoid_collision(self):
        try:
            points = self.blackboard.get("velodyne_points")
            cloud_points = list(
                point_cloud2.read_points(
                    points, skip_nans=True, field_names=("x", "y", "z")
                )
            )
            cld_pts = np.asarray(cloud_points)
            print("cld pts: ", cld_pts.shape)

            p2 = np.array([1.0, 0.0, 0.0])
            standoff = 1.0

            # Slice
            vecs_d = np.cross(p2, cld_pts)
            d = LA.norm(vecs_d, axis=1)
            l2 = np.sum((p2[:2]) ** 2)
            t = np.sum((cld_pts[:, :2]) * (p2[:2]), axis=1) / l2
            arr_points = cld_pts[(d < standoff) & (t > 0.0)]
            print("arr_points: ", arr_points.shape)

            self.publisher.publish(self.point_cloud(arr_points, "velodyne"))

            if arr_points.shape[0] == 0:
                min_norm = 10000
            else:
                norm = LA.norm(arr_points, axis=1)
                min_norm = np.min(norm)
            print("min ", min_norm)

            print("airspeed: ", self.vehicle.airspeed)

            break_distance = self.vehicle.airspeed * 3.0

            if min_norm < break_distance:
                print("breaking!!!")
                return True
        except:
            traceback.print_exc()
        return False

    def point_cloud(self, points, parent_frame):
        """Creates a point cloud message.
        Args:
            points: Nx3 array of xyz positions.
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message
        Code source:
            https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
        """
        ros_dtype = sensor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes()
        fields = [
            sensor_msgs.PointField(
                name=n, offset=i * itemsize, datatype=ros_dtype, count=1
            )
            for i, n in enumerate("xyz")
        ]
        header = std_msgs.Header(frame_id=parent_frame)

        return sensor_msgs.PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3),  # Every point consists of three float32s.
            row_step=(itemsize * 3 * points.shape[0]),
            data=data,
        )


class MoveToGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, blackboard, vehicle):
        super().__init__(name=name)
        self.logger.debug("%s.init()" % self.__class__.__name__)
        self.name = name
        self.vehicle = vehicle
        self.blackboard = blackboard

        self.goal_size = 1.5

    def setup(self, **kwargs):
        self.logger.debug("%s.setup()" % self.__class__.__name__)

    def initialise(self):
        self.logger.debug("%s.initialise()" % self.__class__.__name__)

    def update(self) -> py_trees.common.Status:
        self.logger.debug("%s.update()" % self.__class__.__name__)

        print("next_location: ", self.blackboard.next_location)
        if self.blackboard.next_location is None:
            return py_trees.common.Status.SUCCESS

        if self.get_distance() > self.goal_size:
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status):
        self.logger.debug("%s.terminate()" % self.__class__.__name__)

    def get_distance(self):
        print("altitude: ", self.vehicle.location.global_relative_frame.alt)
        print("yaw: ", self.vehicle.attitude.yaw)

        current_location = np.array(
            [
                self.vehicle.location.local_frame.north,
                self.vehicle.location.local_frame.east,
                self.vehicle.location.global_relative_frame.alt,
            ]
        )
        print("current_location: ", current_location)

        remainingDistance = LA.norm(self.blackboard.next_location - current_location)
        print("remainingDistance: ", remainingDistance)

        return remainingDistance


class NewGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, blackboard, vehicle, altitude):
        super().__init__(name=name)
        self.logger.debug("%s.init()" % self.__class__.__name__)
        self.name = name
        self.last_goal = None
        self.next_goal = None
        self.blackboard = blackboard
        self.vehicle = vehicle
        self.raster_position = 0
        self.altitude = altitude
        self.move_count = 0
        self.move_limit = 9

    def lidar_callback(self, msg):
        print("lidar callback")

    def setup(self, **kwargs):
        self.logger.debug("%s.setup()" % self.__class__.__name__)

    def initialise(self):
        self.logger.debug("%s.initialise()" % self.__class__.__name__)

    def update(self) -> py_trees.common.Status:
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.blackboard.start_raster == False:
            return py_trees.common.Status.RUNNING

        if self.move_count >= self.move_limit:
            return py_trees.common.Status.SUCCESS

        self.call_simple_goto()
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        self.logger.debug("%s.terminate()" % self.__class__.__name__)

    def call_simple_goto(self):
        new_goal = self.raster()
        print("NEW GOAL: ", new_goal)

        targetLocation = dronekit_funcs.get_location_metres(
            self.vehicle.location.global_relative_frame, new_goal[0], new_goal[1]
        )
        print("target Location: ", targetLocation)

        # self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.simple_goto(targetLocation)
        self.move_count = self.move_count + 1

    def raster(self):
        self.raster_position += 1

        # Bounds:
        # [North, East]
        # [[0, 80], [-50, 0]]
        # north_step = 20
        # east_diff = east_min - east_cur

        # Raster box:
        north_step = 20
        east_left_bound = -50
        east_right_bound = 0

        north = self.vehicle.location.local_frame.north
        east = self.vehicle.location.local_frame.east

        raster_delta = [
            [0, 0], # this is ignored
            [0, east_left_bound],
            [north_step, east],
            [0, east_right_bound],
            [north_step, east],
            [0, east_left_bound],
            [north_step, east],
            [0, east_right_bound],
            [north_step, east],
            [0, east_left_bound],
        ]
        north_diff = raster_delta[self.raster_position][0]
        east_diff = raster_delta[self.raster_position][1] - east

        print("self.raster_position: ", self.raster_position)
        print("north + point_diff[0]: ", north, north_diff)
        print("east + point_diff[0]: ", east, east_diff)

        self.blackboard.next_location = np.array(
            [north + north_diff, east + east_diff, self.altitude]
        )

        return np.array([north_diff, east_diff])


class FlyHome(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, vehicle, blackboard):
        super().__init__(name=name)
        self.logger.debug("%s.init()" % self.__class__.__name__)
        self.name = name
        self.vehicle = vehicle
        self.blackboard = blackboard
        self.rtl_called = False

    def setup(self, **kwargs):
        self.logger.debug("%s.setup()" % self.__class__.__name__)

    def initialise(self):
        self.logger.debug("%s.initialise()" % self.__class__.__name__)

    def update(self) -> py_trees.common.Status:
        self.fly_home()

        self.blackboard.start_raster = False

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        self.logger.debug("%s.terminate()" % self.__class__.__name__)

    def fly_home(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        print("self.vehicle.mode: ", self.vehicle.mode)

        current_location = np.array(
            [
                self.vehicle.location.local_frame.north,
                self.vehicle.location.local_frame.east,
            ]
        )
        print("current_location: ", current_location)

        remainingDistance = LA.norm(current_location)
        print("remainingDistance: ", remainingDistance)

        if not self.rtl_called:
            self.rtl_called = True
            self.vehicle.mode = VehicleMode("RTL")
