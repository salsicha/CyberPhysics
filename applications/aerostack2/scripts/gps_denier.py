#!/usr/bin/env python3
"""Deny an Aerodrone simulation's GPS 1 for a window of simulation time.

The ``gps_denier`` service of the ``denial`` profile of the Gazebo, Genesis
and Isaac compositions (tools/gps_denied_sim_test.sh and
tools/gps_denied_sim_test.md):

1. waits until the mission reaches cruise: the vehicle armed and at least
   ``--cruise-height-m`` above the EKF origin (MAVROS local position; the
   wilderness mission climbs to CRUISE_AGL_M, 170-255 m above home);
2. waits ``--start-s`` (GPS_DENIAL_START_S) of simulation time, then sets
   ``SIM_GPS_DISABLE=1`` on ArduPilot SITL, so GPS 1 reports no fix;
3. restores it after ``--duration-s`` (GPS_DENIAL_DURATION_S) of simulation
   time, waits ``--recovery-s`` (GPS_DENIAL_RECOVERY_S) and exits 0.

Every interval is simulation time from ``/clock`` (ArduPilot's lock-step
time), never wall time: Isaac runs at about 0.2x and Genesis at about 0.5x
real time. Each step is appended to ``--events`` as a JSON line with the wall
time ``rx`` and the simulation time ``sim``, for
tools/gps_denied_sim_scorer.py. Exits 1 if ArduPilot refuses the parameter.

The schedule (DenialSchedule) is plain Python; ROS is imported only by
run(), so tests/test_gps_denied_sim_wiring.py exercises it without ROS.
"""

import argparse
import json
import signal
import sys
import time

# Actions, in order.
CRUISE, DENY, RESTORE, DONE = 'cruise', 'deny', 'restore', 'done'


class DenialSchedule:
    """When to deny and restore GPS 1, from simulation time alone.

    due() returns the action due now (or None) until mark() records that it
    was carried out, so an action whose parameter write failed stays due and
    is retried. The window runs from the simulation time GPS 1 was actually
    disabled."""

    def __init__(self, start_s, duration_s, recovery_s, cruise_height_m):
        if start_s < 0.0 or recovery_s < 0.0:
            raise ValueError('start and recovery must be >= 0 s')
        if duration_s <= 0.0:
            raise ValueError('the denial must last > 0 s of simulation time')
        self.start_s = float(start_s)
        self.duration_s = float(duration_s)
        self.recovery_s = float(recovery_s)
        self.cruise_height_m = float(cruise_height_m)
        self.cruise_sim = None
        self.off_sim = None
        self.on_sim = None
        self.finished = False

    def due(self, sim, height_m, armed):
        if sim is None or self.finished:
            return None
        if self.cruise_sim is None:
            at_cruise = (armed and height_m is not None
                         and height_m >= self.cruise_height_m)
            return CRUISE if at_cruise else None
        if self.off_sim is None:
            return DENY if sim >= self.cruise_sim + self.start_s else None
        if self.on_sim is None:
            return RESTORE if sim >= self.off_sim + self.duration_s else None
        return DONE if sim >= self.on_sim + self.recovery_s else None

    def mark(self, action, sim):
        if action == CRUISE:
            self.cruise_sim = sim
        elif action == DENY:
            self.off_sim = sim
        elif action == RESTORE:
            self.on_sim = sim
        elif action == DONE:
            self.finished = True
        else:
            raise ValueError(action)

    @property
    def phase(self):
        if self.cruise_sim is None:
            return 'waiting for cruise'
        if self.off_sim is None:
            return 'cruise, GPS on'
        if self.on_sim is None:
            return 'GPS 1 denied'
        return 'GPS restored'


class EventLog:
    """Appends one JSON line per step (and echoes it to the log)."""

    def __init__(self, path):
        self.path = path

    def write(self, event, sim=None, **fields):
        row = {'event': event, 'rx': round(time.time(), 3)}
        if sim is not None:
            row['sim'] = round(sim, 3)
        row.update(fields)
        with open(self.path, 'a') as stream:
            stream.write(json.dumps(row) + '\n')
        print(json.dumps(row), flush=True)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--events', default='/out/denial_events.jsonl')
    parser.add_argument('--start-s', type=float, default=60.0,
                        help='simulation time from cruise to the denial')
    parser.add_argument('--duration-s', type=float, default=150.0,
                        help='simulation time GPS 1 stays denied')
    parser.add_argument('--recovery-s', type=float, default=60.0,
                        help='simulation time recorded after GPS 1 is restored')
    parser.add_argument('--cruise-height-m', type=float, default=140.0,
                        help='height above the EKF origin that marks cruise')
    parser.add_argument('--mavros-namespace', default='/mavros')
    parser.add_argument('--clock-topic', default='/clock')
    parser.add_argument('--param', default='SIM_GPS_DISABLE')
    parser.add_argument('--retries', type=int, default=10)
    return parser.parse_args(argv)


def run(args):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import State, StatusText
    from mavros_msgs.srv import ParamSetV2
    from rcl_interfaces.msg import ParameterType, ParameterValue
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock

    schedule = DenialSchedule(args.start_s, args.duration_s, args.recovery_s,
                              args.cruise_height_m)
    log = EventLog(args.events)
    log.write('started', start_s=schedule.start_s, duration_s=schedule.duration_s,
              recovery_s=schedule.recovery_s, cruise_height_m=schedule.cruise_height_m,
              param=args.param)

    class Denier(Node):
        def __init__(self):
            super().__init__('gps_denier')
            self.sim = None
            self.height = None
            self.armed = False
            ns = args.mavros_namespace
            self.create_subscription(Clock, args.clock_topic, self._on_clock,
                                     qos_profile_sensor_data)
            self.create_subscription(PoseStamped, f'{ns}/local_position/pose', self._on_pose,
                                     qos_profile_sensor_data)
            self.create_subscription(State, f'{ns}/state', self._on_state, 10)
            self.create_subscription(
                StatusText, f'{ns}/statustext/recv',
                lambda msg: self.get_logger().info(f'FCU: {msg.text}'),
                qos_profile_sensor_data)
            self.param_client = self.create_client(ParamSetV2, f'{ns}/param/set')

        def _on_clock(self, msg):
            first = self.sim is None
            self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9
            if first:
                log.write('clock', self.sim)

        def _on_pose(self, msg):
            self.height = msg.pose.position.z

        def _on_state(self, msg):
            self.armed = bool(msg.armed)

        def set_param(self, value):
            for attempt in range(1, args.retries + 1):
                if self.param_client.wait_for_service(timeout_sec=10.0):
                    request = ParamSetV2.Request()
                    request.force_set = True
                    request.param_id = args.param
                    request.value = ParameterValue(
                        type=ParameterType.PARAMETER_INTEGER, integer_value=int(value))
                    future = self.param_client.call_async(request)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                    result = future.result()
                    if result is not None and result.success:
                        return True
                self.get_logger().warning(
                    f'{args.param}={value} not set (attempt {attempt}/{args.retries})')
                time.sleep(2.0)
            return False

    rclpy.init()
    node = Denier()
    status = 0
    last_report = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - last_report >= 30.0:
                last_report = now
                sim = 'no /clock yet' if node.sim is None else f'{node.sim:.1f} s'
                height = 'unknown' if node.height is None else f'{node.height:.0f} m'
                node.get_logger().info(
                    f'{schedule.phase}: simulation time {sim}, height {height}')
            action = schedule.due(node.sim, node.height, node.armed)
            if action is None:
                continue
            if action in (DENY, RESTORE):
                value = 1 if action == DENY else 0
                if not node.set_param(value):
                    log.write('error', node.sim, reason=f'could not set {args.param}={value}')
                    status = 1
                    break
            schedule.mark(action, node.sim)
            name = {CRUISE: 'cruise', DENY: 'gps_disabled', RESTORE: 'gps_restored',
                    DONE: 'done'}[action]
            extra = {'height_m': round(node.height, 1)} if action == CRUISE else {}
            log.write(name, node.sim, **extra)
            if action == DONE:
                break
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


def main(argv=None):
    args = parse_args(argv)
    # PID 1 in its container: without a handler SIGTERM would be ignored.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
