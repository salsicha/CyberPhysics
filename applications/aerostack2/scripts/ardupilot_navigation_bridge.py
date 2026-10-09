#!/usr/bin/env python3
"""ArduPilot side of the vision-navigation stack.

1. Shared origin. Relays the ArduPilot EKF origin from MAVROS
   ``<mavros>/global_position/gp_origin`` to ``/navigation/origin``
   (geographic_msgs/GeoPointStamped, reliable + transient_local, depth 1) so
   every navigation node anchors local ENU at the same point as the
   autopilot's local odometry. MAVROS converts the origin altitude that
   ArduPilot reports (GPS_GLOBAL_ORIGIN, metres AMSL) to WGS-84 ellipsoid
   height (verified in SITL: home 781.0 m AMSL at Mount Tamalpais gives an
   origin altitude of 748.77 m, the EGM96 geoid height there being -32.2 m),
   so ``/navigation/origin`` altitude is ellipsoidal like MAVROS
   ``global_position/global``. ArduPilot sends GPS_GLOBAL_ORIGIN when the
   origin is set; until one arrives the bridge requests it
   (MAV_CMD_REQUEST_MESSAGE).

2. GPS-denied corrections (``gps_input_enabled``; the node default is off,
   every ArduCopter stack enables it through ``GPS_INPUT_CORRECTIONS``,
   default true). Accepted DemNav/WildNav corrections from the fusion node
   (``/navigation/correction_fix``: each accepted absolute fix once,
   time-aligned to the latest odometry sample) are forwarded to ArduPilot as
   a second GPS through the MAVROS gps_input plugin (MAVLink GPS_INPUT,
   GPS2_TYPE=14). Only fresh accepted fixes are sent, never the fused
   estimate. Horizontal accuracy is the fix's standard deviation; altitude,
   vertical accuracy and velocity are flagged as absent (visual fixes carry
   the prior's altitude, and height stays on the barometer).

   ArduPilot fuses GPS velocity whenever GPS is a position source, and a
   GPS_INPUT without velocity would be fused as zero velocity, which trips
   the EKF velocity innovation check. The bridge therefore switches the EKF
   to source set 2 (GPS position only, no velocity: see
   systems/aerodrone/config/arducopter_gps_input.params) while the primary
   GPS has no 3D fix, and back to set 1 once it has held a 3D fix for
   ``gps_recovery_s``.

   A configured GPS 2 without a fix fails ArduPilot's "GPS 2: Bad fix"
   pre-arm check, and visual fixes only exist in flight. With
   ``manage_gps2_type`` (default) the bridge therefore sets GPS2_TYPE=14
   only when the primary GPS is lost, and back to 0 whenever the vehicle is
   disarmed, so the pre-arm GPS checks stay enabled. (If the bridge dies
   while GPS 2 is configured, the next arming attempt is refused with
   "GPS 2: Bad fix" until GPS2_TYPE is reset: a fail-safe outcome.)

   Flight-controller check. Forwarding needs the EKF source set 2 and GPS
   configuration of arducopter_gps_input.params ON THE AUTOPILOT, and the
   bridge never writes those parameters. When enabled it reads them through
   MAVROS (``<mavros>/param/get_parameters``) every ``fc_check_period_s``
   and arms forwarding (GPS_INPUT, EKF source switching and GPS2_TYPE
   management) only once every one matches ``required_fc_params``. A
   mismatch is logged as an error naming each wrong parameter and
   forwarding stays inactive; checking continues, so loading the file on
   the autopilot arms it without a restart. GPS2_TYPE may be 0 or 14 (the
   bridge manages it); any other GPS 2 type is a real receiver the bridge
   must not overwrite. The check runs once per bridge start: parameters
   changed after it passed are not re-checked. SITL passes it because
   run_arducopter_sitl.sh loads the file.

   Once forwarding is armed the bridge reports so on
   ``/navigation/autopilot_corrections_active`` so the fusion node stops
   applying its own offsets (the EKF already absorbs the corrections).
   Until then, and whenever forwarding is disabled, it reports false.

Prototype results in ArduCopter 4.6.3 SITL (GPS 1 disabled mid-flight in
GUIDED, sparse GPS_INPUT built from simulator truth plus noise) and the
measured limits are recorded in tools/gps_denied_sitl_test.md.
"""
from __future__ import annotations

import math

import rclpy
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import GPSINPUT, GPSRAW, State
from mavros_msgs.srv import CommandLong, ParamSetV2
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool

MAV_CMD_REQUEST_MESSAGE = 512
MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN = 49
MAV_CMD_SET_EKF_SOURCE_SET = 42007
GPS_TYPE_NONE = 0
GPS_TYPE_MAV = 14

# MAVLink GPS_INPUT_IGNORE_FLAG
IGNORE_ALT = 1
IGNORE_HDOP = 2
IGNORE_VDOP = 4
IGNORE_VEL_HORIZ = 8
IGNORE_VEL_VERT = 16
IGNORE_SPEED_ACCURACY = 32
IGNORE_HORIZONTAL_ACCURACY = 64
IGNORE_VERTICAL_ACCURACY = 128
HORIZONTAL_ONLY = (IGNORE_ALT | IGNORE_VDOP | IGNORE_VEL_HORIZ | IGNORE_VEL_VERT
                   | IGNORE_SPEED_ACCURACY | IGNORE_VERTICAL_ACCURACY)

GPS_FIX_TYPE_3D_FIX = 3
GPS_EPOCH_UNIX_S = 315964800       # 1980-01-06T00:00:00Z
GPS_LEAP_SECONDS = 18              # GPS - UTC since 2017-01-01
SECONDS_PER_WEEK = 604800
# Nominal user-equivalent range error used to express accuracy as HDOP.
NOMINAL_UERE_M = 5.0


def gps_week_ms(unix_ns):
    """(GPS week, milliseconds of week) for a UTC unix time in ns."""
    gps_ms = (int(unix_ns) // 1_000_000
              - GPS_EPOCH_UNIX_S * 1000 + GPS_LEAP_SECONDS * 1000)
    week, week_ms = divmod(gps_ms, SECONDS_PER_WEEK * 1000)
    return int(week), int(week_ms)


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def horizontal_sigma(fix):
    """Horizontal standard deviation (m) of a NavSatFix, or None."""
    if fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
        return None
    variance = max(fix.position_covariance[0], fix.position_covariance[4])
    if not math.isfinite(variance) or variance <= 0.0:
        return None
    return math.sqrt(variance)


def gps_input_fields(fix, gps_id, satellites, min_accuracy_m=0.5):
    """MAVLink GPS_INPUT field values for an accepted correction fix, or None
    when the fix cannot be expressed honestly."""
    sigma = horizontal_sigma(fix)
    if (sigma is None or not math.isfinite(fix.latitude)
            or not math.isfinite(fix.longitude)
            or not -90.0 <= fix.latitude <= 90.0
            or not -180.0 <= fix.longitude <= 180.0):
        return None
    sigma = max(float(min_accuracy_m), sigma)
    week, week_ms = gps_week_ms(stamp_ns(fix.header.stamp))
    return {
        'fix_type': GPS_FIX_TYPE_3D_FIX,
        'gps_id': int(gps_id),
        'ignore_flags': HORIZONTAL_ONLY,
        'time_week': week,
        'time_week_ms': week_ms,
        'lat': int(round(fix.latitude * 1e7)),
        'lon': int(round(fix.longitude * 1e7)),
        'alt': 0.0,
        'hdop': float(min(99.0, max(0.5, sigma / NOMINAL_UERE_M))),
        'vdop': 0.0,
        'vn': 0.0, 've': 0.0, 'vd': 0.0,
        'speed_accuracy': 0.0,
        'horiz_accuracy': float(sigma),
        'vert_accuracy': 0.0,
        'satellites_visible': int(satellites),
        'yaw': 0,
    }


def required_fc_params(denied_set=2, gps2_type_param='GPS2_TYPE'):
    """Flight-controller parameters GPS_INPUT forwarding depends on, as
    name -> accepted values: systems/aerodrone/config/
    arducopter_gps_input.params (a unit test keeps the two identical).

    GPS2_TYPE is the one the bridge manages at runtime (0 on the ground, 14
    while GPS 1 is lost in flight), so both are accepted; anything else is
    a real second receiver."""
    source = f'EK3_SRC{int(denied_set)}_'
    return {
        str(gps2_type_param): (GPS_TYPE_NONE, GPS_TYPE_MAV),
        'GPS_AUTO_SWITCH': (4,),
        'GPS_PRIMARY': (0,),
        source + 'POSXY': (3,),
        source + 'VELXY': (0,),
        source + 'POSZ': (1,),
        source + 'VELZ': (0,),
        source + 'YAW': (1,),
        'EK3_SRC_OPTIONS': (0,),
    }


def parameter_number(value):
    """The number in an rcl_interfaces ParameterValue, or None when MAVROS
    does not have the parameter (yet)."""
    if value.type == ParameterType.PARAMETER_INTEGER:
        return value.integer_value
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return value.double_value
    return None


def check_fc_params(required, reported):
    """Compare reported FC parameters (name -> number, None when unknown)
    with ``required`` (name -> accepted values).

    Returns (missing, wrong): names not reported, and ``NAME=value (needs
    X)`` for every reported parameter with another value."""
    missing, wrong = [], []
    for name, accepted in required.items():
        value = reported.get(name)
        if value is None:
            missing.append(name)
        elif not any(abs(value - ok) < 1e-6 for ok in accepted):
            wrong.append(f'{name}={value:g} (needs '
                         f"{' or '.join(str(ok) for ok in accepted)})")
    return missing, wrong


class SourceSetSwitch:
    """Hysteresis on the primary GPS fix: denied set after loss_s without a
    3D fix, nominal set after recovery_s with one.

    A missing fix only counts as a loss once the GPS has had a fix (or the
    vehicle is armed): a GPS still acquiring its first fix on the ground is
    not GPS denial.
    """

    def __init__(self, loss_s, recovery_s, nominal_set=1, denied_set=2):
        self.loss_s = float(loss_s)
        self.recovery_s = float(recovery_s)
        self.nominal_set = int(nominal_set)
        self.denied_set = int(denied_set)
        self.current = self.nominal_set
        self.armed = False
        self._since = None
        self._state = True
        self._seen_fix = False

    def update(self, has_3d_fix, now_s):
        """Return the source set to request now, or None to keep it."""
        has_3d_fix = bool(has_3d_fix)
        self._seen_fix = self._seen_fix or has_3d_fix
        if has_3d_fix != self._state or self._since is None:
            self._state = has_3d_fix
            self._since = now_s
        held = now_s - self._since
        if (not has_3d_fix and self.current != self.denied_set
                and held >= self.loss_s and (self._seen_fix or self.armed)):
            return self.denied_set
        if has_3d_fix and self.current != self.nominal_set and held >= self.recovery_s:
            return self.nominal_set
        return None


class ArduPilotNavigationBridge(Node):
    def __init__(self):
        super().__init__('ardupilot_navigation_bridge')
        defaults = {
            'mavros_namespace': '/mavros',
            'origin_topic': '/navigation/origin',
            'origin_request_period_s': 5.0,
            'gps_input_enabled': False,
            'correction_topic': '/navigation/correction_fix',
            'autopilot_corrections_topic': '/navigation/autopilot_corrections_active',
            'gps_id': 1,
            'satellites_visible': 6,
            'max_correction_age_s': 1.0,
            'min_horizontal_accuracy_m': 0.5,
            'source_switching': True,
            'gps_loss_s': 1.0,
            'gps_recovery_s': 10.0,
            'nominal_source_set': 1,
            'denied_source_set': 2,
            'manage_gps2_type': True,
            'gps2_type_param': 'GPS2_TYPE',
            'fc_check_period_s': 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        mavros = '/' + str(self.get_parameter('mavros_namespace').value).strip('/')
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.origin = None
        self.origin_pub = self.create_publisher(
            GeoPointStamped, self.get_parameter('origin_topic').value, latched)
        # MAVROS (2.14) latches gp_origin: reliable + transient_local.
        self.create_subscription(
            GeoPointStamped, f'{mavros}/global_position/gp_origin',
            self._on_origin, latched)
        self.command_client = self.create_client(
            CommandLong, f'{mavros}/cmd/command')
        self.create_timer(
            max(0.5, float(self.get_parameter('origin_request_period_s').value)),
            self._request_origin)

        self.enabled = bool(self.get_parameter('gps_input_enabled').value)
        # Forwarding arms only once the FC configuration has been verified.
        self.forwarding = False
        self.active_pub = self.create_publisher(
            Bool, self.get_parameter('autopilot_corrections_topic').value, latched)
        self.active_pub.publish(Bool(data=False))
        self.sent = 0
        self.switch = None
        self.armed = None
        self.gps2_type = None
        self.manage_gps2 = self.enabled and bool(
            self.get_parameter('manage_gps2_type').value)
        if self.enabled:
            self.required_params = required_fc_params(
                self.get_parameter('denied_source_set').value,
                self.get_parameter('gps2_type_param').value)
            self.param_get_client = self.create_client(
                GetParameters, f'{mavros}/param/get_parameters')
            self.fc_check = None
            self.fc_report = (None, -math.inf)
            self.fc_timer = self.create_timer(
                max(0.5, float(self.get_parameter('fc_check_period_s').value)),
                self._check_fc_params)
            self.param_client = self.create_client(
                ParamSetV2, f'{mavros}/param/set')
            self.gps_input_pub = self.create_publisher(
                GPSINPUT, f'{mavros}/gps_input/gps_input', 10)
            self.create_subscription(
                NavSatFix, self.get_parameter('correction_topic').value,
                self._on_correction, 10)
            if bool(self.get_parameter('source_switching').value):
                self.switch = SourceSetSwitch(
                    self.get_parameter('gps_loss_s').value,
                    self.get_parameter('gps_recovery_s').value,
                    self.get_parameter('nominal_source_set').value,
                    self.get_parameter('denied_source_set').value)
                self.create_subscription(
                    GPSRAW, f'{mavros}/gpsstatus/gps1/raw', self._on_gps1,
                    qos_profile_sensor_data)
            self.create_subscription(
                State, f'{mavros}/state', self._on_state, 10)
            self.get_logger().info(
                'GPS_INPUT corrections requested: forwarding arms once the '
                'flight controller reports the arducopter_gps_input.params '
                'configuration')
        self.get_logger().info(
            f'ArduPilot navigation bridge on {mavros}; origin -> '
            f"{self.get_parameter('origin_topic').value}")

    # Shared origin ---------------------------------------------------
    def _on_origin(self, msg):
        position = msg.position
        values = (position.latitude, position.longitude, position.altitude)
        if (not all(math.isfinite(value) for value in values)
                or not -90.0 <= position.latitude <= 90.0
                or not -180.0 <= position.longitude <= 180.0
                or (position.latitude == 0.0 and position.longitude == 0.0)):
            self.get_logger().warning('Ignoring invalid EKF origin from MAVROS')
            return
        if values == self.origin:
            return
        self.origin = values
        out = GeoPointStamped()
        out.header = msg.header
        out.header.frame_id = 'earth'
        out.position = msg.position
        self.origin_pub.publish(out)
        self.get_logger().info(
            f'EKF origin {values[0]:.7f}, {values[1]:.7f}, '
            f'{values[2]:.2f} m (WGS-84 ellipsoid)')

    def _request_origin(self):
        if self.origin is not None or not self.command_client.service_is_ready():
            return
        request = CommandLong.Request()
        request.command = MAV_CMD_REQUEST_MESSAGE
        request.param1 = float(MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN)
        self.command_client.call_async(request)

    # Flight-controller configuration check ----------------------------
    def _check_fc_params(self):
        now_s = self.get_clock().now().nanoseconds / 1e9
        if self.forwarding or (self.fc_check is not None
                               and now_s - self.fc_check[1] < 30.0):
            return  # (a call MAVROS never answered is retried after 30 s)
        if not self.param_get_client.service_is_ready():
            self._report_fc('warn', 'GPS_INPUT corrections not active yet: '
                                    'waiting for the MAVROS parameter service')
            return
        request = GetParameters.Request()
        request.names = list(self.required_params)
        future = self.param_get_client.call_async(request)
        self.fc_check = (future, now_s)
        future.add_done_callback(self._fc_params_done)

    def _fc_params_done(self, future):
        if self.fc_check is None or self.fc_check[0] is not future:
            return
        self.fc_check = None
        response = future.result()
        values = list(response.values) if response is not None else []
        reported = {name: parameter_number(value)
                    for name, value in zip(self.required_params, values)}
        missing, wrong = check_fc_params(self.required_params, reported)
        if wrong:
            self._report_fc(
                'error',
                'GPS_INPUT corrections NOT active: the flight controller does '
                'not have the systems/aerodrone/config/'
                'arducopter_gps_input.params configuration: '
                + ', '.join(wrong) + '. Load that file on the autopilot '
                '(systems/aerodrone/README.md); this bridge never writes EKF '
                'source parameters.')
            return
        if missing:
            self._report_fc(
                'warn', 'GPS_INPUT corrections not active yet: MAVROS has not '
                'read ' + ', '.join(missing) + ' from the flight controller')
            return
        self.forwarding = True
        self.fc_timer.cancel()
        self.active_pub.publish(Bool(data=True))
        self.get_logger().warn(
            'GPS_INPUT corrections ACTIVE: flight controller parameters '
            'verified; accepted DemNav/WildNav fixes are sent to ArduPilot as '
            'GPS 2')

    def _report_fc(self, level, text, repeat_s=60.0):
        """Log a check result when it changes, else at most every repeat_s."""
        now_s = self.get_clock().now().nanoseconds / 1e9
        last_text, last_s = self.fc_report
        if text == last_text and now_s - last_s < repeat_s:
            return
        self.fc_report = (text, now_s)
        # One call site per severity: rclpy refuses to change it per site.
        if level == 'error':
            self.get_logger().error(text)
        else:
            self.get_logger().warn(text)

    # GPS-denied corrections -------------------------------------------
    def _on_correction(self, fix):
        if not self.forwarding:
            return
        now_ns = self.get_clock().now().nanoseconds
        age_s = (now_ns - stamp_ns(fix.header.stamp)) / 1e9
        if not -0.5 <= age_s <= float(self.get_parameter('max_correction_age_s').value):
            self.get_logger().warning(
                f'Dropped correction {age_s:.2f} s old', throttle_duration_sec=5.0)
            return
        fields = gps_input_fields(
            fix, self.get_parameter('gps_id').value,
            self.get_parameter('satellites_visible').value,
            self.get_parameter('min_horizontal_accuracy_m').value)
        if fields is None:
            self.get_logger().warning('Dropped correction without a usable '
                                      'horizontal position/covariance')
            return
        msg = GPSINPUT()
        msg.header.stamp = fix.header.stamp
        msg.header.frame_id = 'map'
        for name, value in fields.items():
            setattr(msg, name, value)
        self.gps_input_pub.publish(msg)
        self.sent += 1
        if self.sent == 1 or self.sent % 20 == 0:
            self.get_logger().info(
                f'Sent {self.sent} GPS_INPUT corrections '
                f"(last horizontal accuracy {fields['horiz_accuracy']:.1f} m)")

    def _on_state(self, msg):
        if not msg.connected:
            return
        armed = bool(msg.armed)
        if self.switch is not None:
            self.switch.armed = armed
        if self.forwarding and self.manage_gps2 and not armed and (
                self.armed is not False or self.gps2_type != GPS_TYPE_NONE):
            # On the ground GPS 2 must not exist, or pre-arm checks fail.
            self._set_gps2_type(GPS_TYPE_NONE)
        self.armed = armed

    def _set_gps2_type(self, value):
        if self.gps2_type == value or not self.param_client.service_is_ready():
            return
        request = ParamSetV2.Request()
        request.force_set = True
        request.param_id = str(self.get_parameter('gps2_type_param').value)
        request.value = ParameterValue(
            type=ParameterType.PARAMETER_INTEGER, integer_value=int(value))
        future = self.param_client.call_async(request)
        future.add_done_callback(
            lambda done, wanted=value: self._gps2_type_done(done, wanted))
        self.gps2_type = value

    def _gps2_type_done(self, future, value):
        response = future.result()
        if response is None or not response.success:
            self.get_logger().error(
                f"Could not set {self.get_parameter('gps2_type_param').value}"
                f'={value}; will retry')
            self.gps2_type = None
            return
        self.get_logger().warn(
            f"{self.get_parameter('gps2_type_param').value}={value} "
            f"({'GPS 2 = GPS_INPUT corrections' if value else 'GPS 2 disabled'})")

    def _on_gps1(self, msg):
        if not self.forwarding:
            return
        requested = self.switch.update(
            msg.fix_type >= GPS_FIX_TYPE_3D_FIX,
            self.get_clock().now().nanoseconds / 1e9)
        if requested is None or not self.command_client.service_is_ready():
            return
        if self.manage_gps2 and requested == self.switch.denied_set:
            # GPS 2 must exist before ArduPilot can switch to it.
            self._set_gps2_type(GPS_TYPE_MAV)
        request = CommandLong.Request()
        request.command = MAV_CMD_SET_EKF_SOURCE_SET
        request.param1 = float(requested)
        future = self.command_client.call_async(request)
        future.add_done_callback(
            lambda done, value=requested: self._source_set_done(done, value))
        # Assume success until told otherwise so the request is not spammed.
        self.switch.current = requested

    def _source_set_done(self, future, value):
        response = future.result()
        if response is None or not response.success:
            self.get_logger().error(f'ArduPilot rejected EKF source set {value}')
            # Retry on the next GPS status message.
            self.switch.current = (self.switch.nominal_set
                                   if value == self.switch.denied_set
                                   else self.switch.denied_set)
            return
        self.get_logger().warn(
            f'EKF source set {value} '
            f"({'GPS-denied: GPS 2 position only' if value == self.switch.denied_set else 'nominal GPS'})")


def main():
    rclpy.init()
    node = ArduPilotNavigationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
