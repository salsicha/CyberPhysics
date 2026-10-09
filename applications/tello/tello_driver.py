#!/venv/bin/python3
"""Run tello-ros2's driver node (tello.node.TelloNode) for the launch file.

Upstream's ``main()`` lets a failed connection escape as a traceback and
never exits on shutdown, because the node's video/status/odometry threads
are non-daemon infinite loops. This runs the same node, reports a missing
drone in one log line and exits (tello.launch.py respawns it to retry), and
ends the process on Ctrl-C/SIGTERM instead of waiting for those threads.
"""

import os
import sys
import traceback

import rclpy
from rclpy.executors import ExternalShutdownException
from tello.node import TelloNode


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('tello')
    code = 0
    try:
        try:
            TelloNode(node)
        except Exception as err:  # djitellopy raises bare Exception, OSError
            node.get_logger().error(
                f'Tello: no connection to the drone ({err}). '
                "Join the Tello's WiFi on this host; the launch file retries.")
            code = 1
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # e.g. a drone command failing in a topic callback
        traceback.print_exc()
        code = 1
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


if __name__ == '__main__':
    main()
