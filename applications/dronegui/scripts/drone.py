#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# ros-galactic-ros2cli ros-foxy-py-trees-ros-tutorials ros-foxy-py-trees ros-foxy-py-trees-ros


# source /opt/ros/foxy/setup.sh


# https://dronekit-python.readthedocs.io/en/latest/examples/drone_delivery.html


# Visualize tree:
# ros2 launch py_trees_ros_tutorials tutorial_eight_dynamic_application_loading_launch.py
# py-trees-blackboard-watcher -b
# py-trees-tree-viewer
# ros2 topic echo /battery/state

# Rendering the skeleton:
# py-trees-tree-watcher --dot-graph


import operator
import sys
import launch
import launch_ros
import py_trees
import py_trees_ros.trees
import py_trees.console as console
import py_trees_ros_interfaces.action as py_trees_actions  # noqa
import py_trees_ros_interfaces.srv as py_trees_srvs  # noqa
import rclpy
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
import behaviours
from dronekit import (
    connect,
    VehicleMode,
    LocationGlobalRelative,
    LocationGlobal,
    Command,
)

import cherrypy
from cherrypy.lib.static import serve_file
from jinja2 import Environment, FileSystemLoader
import os
import time
import simplejson

import math

import geopy.distance

import numpy as np


def create_root(
    vehicle, blackboard, altitude, update_rate
) -> py_trees.behaviour.Behaviour:
    """
    Behavior Tree root node
    """

    root = py_trees.composites.Parallel(
        name="Drone root",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )

    # /velodyne_points is split into 8 wedges
    # using /scan_matched_points2 instead

    topics2bb = py_trees.composites.Sequence("Topics2BB")
    scan2bb = py_trees_ros.subscribers.ToBlackboard(
        name="Scan2BB",
        topic_name="/velodyne_points",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        topic_type=sensor_msgs.PointCloud2,
        blackboard_variables={"velodyne_points": None},
    )

    tasks = py_trees.composites.Sequence("Tasks", True)

    takeoff = py_trees.composites.Sequence("Takeoff", True)
    arming = behaviours.Arm(name="arming motors", vehicle=vehicle)
    liftoff = behaviours.Liftoff(
        name="liftoff", blackboard=blackboard, vehicle=vehicle, aTargetAltitude=altitude
    )
    takeoff.add_children([arming, liftoff])

    move = py_trees.composites.Sequence("Move", False)
    collision_check = behaviours.CollisionCheck(
        name="collision check",
        blackboard=blackboard,
        vehicle=vehicle,
        update_rate=update_rate,
    )
    moving = behaviours.MoveToGoal(
        name="move to goal", blackboard=blackboard, vehicle=vehicle
    )
    new_goal = behaviours.NewGoal(
        name="new goal?", blackboard=blackboard, vehicle=vehicle, altitude=altitude
    )
    move.add_children([collision_check, moving, new_goal])

    return_home = py_trees.composites.Sequence("ReturnHome", True)
    fly_home = behaviours.FlyHome(
        name="fly home", vehicle=vehicle, blackboard=blackboard
    )
    return_home.add_children([fly_home])

    topics2bb.add_children([scan2bb])
    root.add_child(topics2bb)

    tasks.add_children([takeoff, move, return_home])
    root.add_child(tasks)

    return root


class Templates:
    """
    HTML templates for web server
    """
    def __init__(self, home_coords, blackboard):
        self.home_coords = home_coords
        self.blackboard = blackboard
        self.options = self.get_options()
        local_path = os.path.dirname(os.path.abspath(__file__))
        self.environment = Environment(loader=FileSystemLoader(local_path + "/html"))

    # TODO: (maybe) add tap to set raster bounding box

    def get_options(self):
        return {
            "width": 550,
            "height": 500,
            "zoom": 18,
            "format": "png",
            "access_token": "pk.eyJ1Ijoic2Fsc2ljaGEiLCJhIjoiY2t0YWtieDR4MW15YjJxcTJrbTBpOGxkZyJ9.juJ3iiq1TBiFYAeGbJm2Vw",
            "mapid": "mapbox.satellite",
            "home_coords": self.home_coords,
            "menu": [
                {"name": "Home", "location": "/"},
                {"name": "Track", "location": "/track"},
                {"name": "Command", "location": "/command"},
            ],
            "current_url": "/",
            "json": "",
        }

    def index(self):
        self.options = self.get_options()
        self.options["current_url"] = "/"
        return self.get_template("index")

    def track(self, current_coords):

        # Dugway
        # lat: 40.06644051683807, lon: -113.061433065372, elevation 1324.061810551025, scale 14
        # scale 14, latitude +-40:
        # 3.660 meters/pixel

        # RFS
        # lat: 37.916120, lon: -122.336566, elevation 5, scale 18
        # scale 18, latitude +-40:
        # 0.229 meters/pixel

        # 1. meters north/east from map center to drone home

        equator = 156543.03

        # Dugway: map center
        # origin_lat = 40.06644051683807
        # origin_lon = -113.061433065372
        # zoom = 14

        # RFS: map center
        origin_lat = 37.916120
        origin_lon = -122.336566
        zoom = 18

        pixel_scale = equator * math.cos(math.radians(origin_lat)) / float(2 ** zoom) / 2.0

        print("pixel_scale: ", pixel_scale)
        print("origin_lat: ", origin_lat)
        print("origin_lon: ", origin_lon)

        try:
            print(
                "self.blackboard.start_location.lat: ",
                self.blackboard.start_location.lat,
            )
            print(
                "self.blackboard.start_location.lon: ",
                self.blackboard.start_location.lon,
            )
        except Exception as e:
            print(str(e))

        lat_to_meters = 0
        try:
            coords_1 = (origin_lat, origin_lon)
            coords_2 = (self.blackboard.start_location.lat, origin_lon)
            lat_to_meters = geopy.distance.distance(coords_1, coords_2).km * 1000.0
        except Exception as e:
            print(str(e))

        lon_to_meters = 0
        try:
            coords_1 = (origin_lat, origin_lon)
            coords_2 = (origin_lat, self.blackboard.start_location.lon)
            lon_to_meters = geopy.distance.distance(coords_1, coords_2).km * 1000.0
        except Exception as e:
            print(str(e))

        print("lat_to_meters: ", lat_to_meters)
        print("lon_to_meters: ", lon_to_meters)

        # 2. pixels north/east from map center to next waypoint

        try:
            next_location = self.blackboard.next_location
        except Exception as e:
            print(str(e))

        if next_location is None:
            next_location = np.array([0, 0, 0])

        pixels_north = (lat_to_meters + next_location[0]) / pixel_scale
        pixels_east = (lon_to_meters + next_location[1]) / pixel_scale

        # Pixel origin is the right, center border of the frame
        # Positive horizontal pixels are to the left (from the far right)
        # Positive vertical pixels are up (from the center)
        pixel_x = 275 - pixels_east
        pixel_y = pixels_north

        print("pixel_x: ", pixel_x)
        print("pixel_y: ", pixel_y)

        self.options = self.get_options()

        self.options["marker_bottom"] = pixel_y
        self.options["marker_right"] = pixel_x

        self.options["current_url"] = "/track"
        self.options["current_coords"] = current_coords
        self.options["json"] = simplejson.dumps(self.options)
        return self.get_template("track")

    def command(self, current_coords):
        self.options = self.get_options()
        self.options["current_url"] = "/command"
        self.options["current_coords"] = current_coords
        return self.get_template("command")

    def get_template(self, file_name):
        template = self.environment.get_template(file_name + ".html")
        return template.render(options=self.options)



class WebServer(object):
    """
    CherryPy web server
    """

    def __init__(self, vehicle, blackboard):
        self.drone = vehicle
        self.commands = self.drone.commands
        self.current_coords = []
        self.current_location = None
        self.blackboard = blackboard

        self.drone.add_attribute_listener("location", self.location_callback)

        home_coords = [
            self.drone.location.global_frame.lat,
            self.drone.location.global_frame.lon,
        ]
        self.templates = Templates(home_coords, blackboard)

    @cherrypy.expose
    def index(self):
        return self.templates.index()

    @cherrypy.expose
    def assets(self, name):
        # print("name: ", name)
        # print(os.path.dirname(os.path.abspath(__file__))+"/html/assets/")
        return serve_file(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)) + "/html/assets/", name
            )
        )

    @cherrypy.expose
    def command(self):
        return self.templates.command(self.get_location())

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def vehicle(self):
        return dict(position=self.get_location())

    @cherrypy.expose
    def track(self, lat=None, lon=None):
        # Process POST request from Command
        # Sending MAVLink packet with goto instructions
        if lat is not None and lon is not None:
            self.drone.goto([lat, lon], True)

        return self.templates.track(self.get_location())

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_location(self):

        return [self.current_location.lat, self.current_location.lon]

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def get_next_wapoint(self):

        return self.blackboard.next_location

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def goto(self, location, relative=None):
        if relative:
            self.drone.simple_goto(
                LocationGlobalRelative(
                    float(location[0]), float(location[1]), float(self.altitude)
                )
            )
        else:
            self.drone.simple_goto(
                LocationGlobal(
                    float(location[0]), float(location[1]), float(self.altitude)
                )
            )
        self.drone.flush()

        return "calling localhost:8080/goto"

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def change_mode(self, mode):
        self.drone.mode = VehicleMode(mode)
        while self.drone.mode.name != mode:
            time.sleep(1)

        return "calling localhost:8080/change_mode"

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def start(self):
        self.blackboard.start_raster = True

        return "Starting"

    @cherrypy.expose
    @cherrypy.tools.json_out()
    def rtl(self):
        self.blackboard.start_raster = False
        self.drone.mode = VehicleMode("RTL")
        while self.drone.mode.name != "RTL":
            time.sleep(1)

        return "RTL"

    def location_callback(self, vehicle, name, location):
        if location.global_relative_frame.alt is not None:
            self.altitude = location.global_relative_frame.alt

        self.current_location = location.global_relative_frame

        # TODO: show next waypoint


def main():
    """
    Init PyTreesROS, DroneKit, and CherryPy
    """

    # DroneKit
    # vehicle = connect("127.0.0.1:14550", wait_ready=True)
    # vehicle = connect('/dev/ttyUSB0', wait_ready=True, baud=57600)
    vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=115200)

    # DroneKit throws an error but doesn't stop:
    # ... "dronekit.APIException: mode (0, 4) not available on mavlink definition"
    # This would quiet the error, but may also prevent dronekit from being able to get data:
    # vehicle.parameters['SERIAL5_PROTOCOL'] = -1

    altitude = 5

    update_period = 200.0 # milliseconds
    update_rate = update_period / 1000.0

    blackboard = py_trees.blackboard.Client(name="location_client")
    blackboard.register_key(key="velodyne_points", access=py_trees.common.Access.READ)
    blackboard.register_key(key="/location", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="/start_location", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="/next_location", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="/origin_location", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="/start_raster", access=py_trees.common.Access.WRITE)

    blackboard.next_location = None
    blackboard.start_raster = False

    rclpy.init(args=None)
    root = create_root(vehicle, blackboard, altitude, update_rate)
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
    try:
        tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(
            console.red
            + "failed to setup the tree, aborting [{}]".format(str(e))
            + console.reset
        )
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=update_period)

    py_trees.logging.level = py_trees.logging.Level.DEBUG

    # CherryPy
    config = {
        "global": {
            "server.socket_host": "127.0.0.1",
            "server.socket_port": 8080,
            "server.thread_pool": 10,
        },
        "/assets": {
            "tools.staticdir.on": True,
            "tools.staticdir.dir": os.path.dirname(os.path.abspath(__file__)),
        },
    }
    cherrypy.tree.mount(WebServer(vehicle, blackboard), "/", config=config)
    cherrypy.engine.start()

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass

    tree.shutdown()
    rclpy.shutdown()


main()
