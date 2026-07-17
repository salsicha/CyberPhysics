"""Shared test setup: import paths and ROS stubs.

The navigation packages are plain Python underneath their ROS wrappers, so
the tests run without a ROS installation: sys.path points at the package
sources, and install_ros_stubs() replaces the ROS modules with mocks for
the tests that import node modules.
"""
import importlib.util
import sys
from pathlib import Path
from unittest import mock

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / 'applications/aerostack2/synthetic_world'))
sys.path.insert(0, str(REPO / 'applications/demnav/src/demnav_pkg'))
sys.path.insert(0, str(REPO / 'applications/wildnav/src/wildnav_pkg'))

ROS_MODULES = (
    'rclpy', 'rclpy.node', 'rclpy.qos',
    'nav_msgs', 'nav_msgs.msg',
    'sensor_msgs', 'sensor_msgs.msg',
    'geometry_msgs', 'geometry_msgs.msg',
    'std_msgs', 'std_msgs.msg',
    'cv_bridge',
)


def install_ros_stubs():
    for name in ROS_MODULES:
        sys.modules.setdefault(name, mock.MagicMock())
    # Node must be a real class so `class X(Node)` definitions work.
    sys.modules['rclpy.node'].Node = object


def load_script(path, name):
    """Import a standalone script (not part of a package) by path."""
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
