#!/usr/bin/env python3
"""Stand-in for the aerodrone's foam dart launcher in the Gazebo and Isaac
tracking stacks (systems/aerodrone/tracking.md, "Foam dart launcher"), so
the tracker's fire control runs its fire cycle end to end.

It takes the place of the launcher's servo bridge
(turret_arduino_bridge.py, ``sts`` transport, launcher only) with the same
topics under ``topic_prefix`` (default ``/aerodrone/launcher``) and the same
behaviour:

- ``joint_commands`` (sensor_msgs/JointState): ``actuator_joint`` (the
  priming rack, metres, 0 forward) and ``trigger_joint`` (radians, 0 safe)
  goals, clamped to the stroke and the fire angle; a command without
  ``actuator_joint`` returns the trigger to 0 and stops the rack where it
  is, as do the emergency stop and 0.5 s without commands (the watchdog);
- ``emergency_stop`` (std_msgs/Bool, latched: the tracker's launcher arm
  state, true while disarmed): commands are dropped while it is engaged,
  and nothing moves until it is first released;
- ``joint_states``: the measured rack and trigger at 25 Hz;
  ``actuator_ready`` (latched, 5 Hz): true while the emergency stop is
  released.

LauncherPlant moves the rack and the trigger toward their goals at the
ST3215's speed at the bridge's 2000 steps/s (the rack on the design's
13.5 mm pinion: 41 mm/s; the trigger 3.07 rad/s), in whole servo steps,
with the sear between them: it catches the plunger when the rack is driven
past 34.25 mm and releases it, a shot, when the trigger is driven past
0.5194 rad (the turret simulator's launcher, TurretSim.update_launcher, and
hardware/parts/turret/_launcher.py). No dart physics: a shot is published
on ``shots_topic`` (default ``/simulation/launcher/shots``, JSON
``{"stamp": <s>, "shot": <n>}``).

It runs on the simulation clock (use_sim_time), as the tracker, ArduPilot
and the targets: the servos move at their speed in the simulated world.
LauncherPlant is plain Python (tests/test_aerodrone_launcher.py).
"""
import json
import math
import sys

STEPS_PER_TURN = 4096
DEFAULTS = {
    'stroke_m': 0.038,             # the rack at prime
    'sear_catch_m': 0.03425,       # the sear drops into the rod's groove from here
    'pinion_radius_m': 0.0135,     # the design's module 1.5, 18-tooth pinion
    'trigger_fire_rad': 0.6,       # lifts the sear 5.4 mm
    'trigger_release_rad': 0.5194,  # 4.45 mm of lift releases it
    'speed_steps_s': 2000,         # the bridge's goal speed, both servos
    'watchdog_s': 0.5,             # no command for this long: trigger to 0, rack stopped
}
ACTUATOR_JOINT = 'actuator_joint'
TRIGGER_JOINT = 'trigger_joint'


def clamp(value, low, high):
    return min(max(value, low), high)


class LauncherPlant:
    """The launcher's rack, trigger and sear behind the bridge (module doc).
    Times in seconds of one clock."""

    def __init__(self, **config):
        unknown = set(config) - set(DEFAULTS)
        if unknown:
            raise ValueError(f'unknown launcher settings {sorted(unknown)}')
        self.config = dict(DEFAULTS, **config)
        self.rack = self.trigger = 0.0
        self.rack_cmd = self.trigger_cmd = 0.0
        self.last_command = -math.inf
        self.estop = True  # nothing moves until the emergency stop is released
        self.primed = False  # the sear holds the plunger
        self.shots = []

    @property
    def servo_speed_rad_s(self):
        return float(self.config['speed_steps_s']) * 2.0 * math.pi / STEPS_PER_TURN

    @property
    def rack_speed_m_s(self):
        return self.servo_speed_rad_s * float(self.config['pinion_radius_m'])

    def command(self, t, rack, trigger):
        """A joint command: rack None stops the rack where it is; trigger
        None returns it to 0. Dropped (False) while the emergency stop is
        engaged, or when not finite."""
        if self.estop:
            return False
        if not all(value is None or math.isfinite(value) for value in (rack, trigger)):
            return False
        self.rack_cmd = self.rack if rack is None else clamp(rack, 0.0,
                                                             float(self.config['stroke_m']))
        self.trigger_cmd = clamp(0.0 if trigger is None else trigger, 0.0,
                                 float(self.config['trigger_fire_rad']))
        self.last_command = t
        return True

    def emergency_stop(self, engaged):
        """Engaged: STOP (the trigger to 0, the rack stopped) and commands
        dropped until released."""
        self.estop = bool(engaged)
        if self.estop:
            self.rack_cmd, self.trigger_cmd = self.rack, 0.0

    def update(self, t, dt):
        """Move dt seconds toward the goals; the time of a shot, or None."""
        config = self.config
        stroke, fire = float(config['stroke_m']), float(config['trigger_fire_rad'])
        if self.estop or t - self.last_command >= float(config['watchdog_s']):
            self.rack_cmd, self.trigger_cmd = self.rack, 0.0
        dt = max(0.0, dt)
        rack, trigger = self.rack, self.trigger
        step = clamp(clamp(self.rack_cmd, 0.0, stroke) - self.rack,
                     -self.rack_speed_m_s * dt, self.rack_speed_m_s * dt)
        quantum = 2.0 * math.pi / STEPS_PER_TURN * float(config['pinion_radius_m'])
        self.rack = clamp(round((self.rack + step) / quantum) * quantum, 0.0, stroke)
        step = clamp(clamp(self.trigger_cmd, 0.0, fire) - self.trigger,
                     -self.servo_speed_rad_s * dt, self.servo_speed_rad_s * dt)
        quantum = 2.0 * math.pi / STEPS_PER_TURN
        self.trigger = clamp(round((self.trigger + step) / quantum) * quantum, 0.0, fire)
        catch, release = float(config['sear_catch_m']), float(config['trigger_release_rad'])
        if rack < catch <= self.rack:
            self.primed = True
        if self.primed and trigger < release <= self.trigger:
            self.primed = False
            self.shots.append(t)
            return t
        return None


def joint_position(msg, name):
    names = list(msg.name)
    if name in names:
        index = names.index(name)
        if index < len(msg.position):
            return float(msg.position[index])
    return None


def run():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String

    rclpy.init()
    node = Node('launcher_standin')
    for name, default in (('topic_prefix', '/aerodrone/launcher'),
                          ('shots_topic', '/simulation/launcher/shots'),
                          ('rate_hz', 50.0), ('feedback_rate_hz', 25.0),
                          ('ready_rate_hz', 5.0)):
        node.declare_parameter(name, default)
    param = lambda name: node.get_parameter(name).value  # noqa: E731
    prefix = '/' + str(param('topic_prefix')).strip('/')
    plant = LauncherPlant()
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    states = node.create_publisher(JointState, f'{prefix}/joint_states', qos_profile_sensor_data)
    ready_pub = node.create_publisher(Bool, f'{prefix}/actuator_ready', latched)
    shots_pub = node.create_publisher(String, str(param('shots_topic')), 10)
    clock = {'last': None}

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def on_command(msg):
        t = now()
        if t <= 0.0:
            return  # no /clock yet
        rack = joint_position(msg, ACTUATOR_JOINT)
        trigger = joint_position(msg, TRIGGER_JOINT)
        plant.command(t, rack, trigger)

    def on_estop(msg):
        if bool(msg.data) != plant.estop:
            node.get_logger().info(f'emergency stop {"engaged" if msg.data else "released"}')
        plant.emergency_stop(msg.data)
        publish_ready()

    def step():
        t = now()
        if t <= 0.0:
            return
        last, clock['last'] = clock['last'], t
        if last is None or t < last:
            return  # the first step, or the simulation was reset
        shot = plant.update(t, t - last)
        if shot is not None:
            shots_pub.publish(String(data=json.dumps({'stamp': round(shot, 4),
                                                      'shot': len(plant.shots)})))
            node.get_logger().info(f'shot {len(plant.shots)} at {shot:.3f} s')

    def publish_state():
        t = now()
        if t <= 0.0:
            return
        state = JointState()
        state.header.stamp = node.get_clock().now().to_msg()
        state.name = [ACTUATOR_JOINT, TRIGGER_JOINT]
        state.position = [float(plant.rack), float(plant.trigger)]
        states.publish(state)

    def publish_ready():
        ready_pub.publish(Bool(data=not plant.estop))

    node.create_subscription(JointState, f'{prefix}/joint_commands', on_command, 10)
    for qos in (latched, 10):
        node.create_subscription(Bool, f'{prefix}/emergency_stop', on_estop, qos)
    node.create_timer(1.0 / float(param('rate_hz')), step)
    node.create_timer(1.0 / float(param('feedback_rate_hz')), publish_state)
    node.create_timer(1.0 / float(param('ready_rate_hz')), publish_ready)
    node.get_logger().info(f'launcher stand-in on {prefix}: ST3215 speed, no dart physics; '
                           f'shots on {param("shots_topic")}')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(run())
