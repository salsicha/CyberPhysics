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
   (``/navigation/correction_fix``: on each accepted fix, the
   inverse-variance combination of the sources' latest fixes, time-aligned
   to the latest odometry sample, its variance scaled for the EKF, which
   fuses every sample as independent) are forwarded to ArduPilot as a
   second GPS through the MAVROS gps_input plugin (MAVLink GPS_INPUT,
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
   only when the primary GPS is lost in flight, and back to 0 whenever the
   vehicle is disarmed, so the pre-arm GPS checks stay enabled. (If the
   bridge dies while GPS 2 is configured, the next arming attempt is
   refused with "GPS 2: Bad fix" until GPS2_TYPE is reset: a fail-safe
   outcome.) GPS 2 exists only while ArduPilot runs source set 2: on GPS 1
   loss the bridge asks for set 2 and configures GPS 2 once ArduPilot has
   accepted it; on recovery it removes GPS 2 and, once that is done, asks
   for set 1. In set 1 (GPS velocity fused) a GPS_INPUT GPS 2 that became
   primary (GPS_AUTO_SWITCH picks it whenever GPS 1 has no fix) is fused
   with the zero velocity it does not carry: in ArduCopter 4.6.3 SITL three
   seconds of that gave "GPS Glitch or Compass error" and the EKF failsafe
   (LAND), with a vertical velocity "Vibration compensation ON" too. A slow
   link or companion makes that window as long as its lag, so the order is
   enforced rather than left to the order MAVROS delivers two requests in.

   Late and bursty corrections. A correction is stamped with the odometry
   sample its position is valid at (the fusion propagates the camera fix to
   it), and ArduPilot dates GPS_INPUT by that stamp: its MAV driver
   corrects the GPS time for transport jitter (assuming the fastest message
   was the least delayed, at most 2 s) and the EKF subtracts GPS2_DELAY_MS,
   then clamps the time to its delay buffer, which ends 250 ms in the past
   at most. A correction more than that late is fused as if 250 ms old: its
   lateness times the speed becomes position error (ArduCopter 4.6.3 SITL,
   GPS-denied at 5 m/s: 0.5 / 1 / 2 / 4 s late gave 3.8 / 6.1 / 12.4 / 16.8
   m median error against 2.2 m on time; bursts with 8 s gaps tripped the
   EKF failsafe). It never reaches the vertical: source set 2 takes height
   from the barometer and no velocity, and the bridge flags altitude and
   velocity absent (tools/gps_denied_sitl_test.md, "Late and bursty
   corrections"). The bridge therefore gates what it forwards
   (``CorrectionGate``): a correction more than ``max_correction_age_s``
   (1 s: at most about 0.75 s misdated, 4 m at 5 m/s, against the 20-100 m
   accuracy the corrections are sent with) old when it reaches the bridge,
   or stamped more than ``max_future_s`` ahead, is dropped, as is one not
   newer than the last sent (ArduPilot ignores a GPS sample less than 50 ms
   after the previous one anyway); at most one goes out per
   ``min_send_interval_s``, the newest of a burst; and when
   ``lag_suspend_count`` of the last ``lag_window`` corrections arrived
   stale the companion is lagging and forwarding stops until
   ``lag_resume_count`` arrive in time in a row. Without corrections
   ArduPilot dead-reckons and, with GPS 1 lost, its EKF failsafe lands the
   aircraft after about 8 s: the designed degradation, rather than
   positions that are seconds old. GPS2_DELAY_MS
   (arducopter_gps_input.params) is the measured shortest time from a
   correction's stamp to the autopilot.

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

import collections
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
# GPS2_DELAY_MS of arducopter_gps_input.params: the measured shortest time
# from a correction's stamp to the autopilot (ArduPilot dates GPS 2 samples
# by it; see the module docstring).
GPS2_DELAY_MS = 20
# In arducopter_gps_input.params but not checked through MAVROS (see
# required_fc_params).
UNCHECKED_FC_PARAMS = {'GPS2_DELAY_MS': GPS2_DELAY_MS}
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
    a real second receiver. GPS2_DELAY_MS (UNCHECKED_FC_PARAMS) is in the
    file but not checked: ArduPilot hides GPS 2's other parameters from its
    parameter list while GPS2_TYPE is 0, so MAVROS never has it."""
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


class CorrectionGate:
    """Which accepted corrections reach ArduPilot, and when.

    ``offer(stamp_ns, now_ns, item)`` judges a correction as it reaches the
    bridge; ``poll(now_ns)`` releases a held one. Both return the
    ``(item, age_s)`` to send now. A correction is

    - stale, and dropped, when it is more than ``max_age_s`` old (its stamp
      against the bridge's clock) or stamped more than ``max_future_s``
      ahead;
    - dropped when it is not newer than the last one sent (a burst
      delivered out of order, a repeat);
    - sent at once when ``min_interval_s`` has passed since the last one,
      else held, replacing any older held one (a burst collapses to its
      newest), and sent by ``poll`` when the interval has passed if it is
      still fresh.

    Lag: when ``suspend_count`` of the last ``window`` corrections were
    stale the pipeline is lagging and nothing is sent until
    ``resume_count`` fresh ones arrive in a row (the last of them is sent).
    ``events`` collects ('suspended' | 'resumed', stale, window) for the
    caller to log; ``counts`` the outcomes, ``ages`` the ages of the
    arrivals (sent or not) since the caller last cleared it.
    """

    def __init__(self, max_age_s=1.0, min_interval_s=0.2, window=10,
                 suspend_count=5, resume_count=5, max_future_s=0.5):
        self.max_age_s = float(max_age_s)
        self.min_interval_s = float(min_interval_s)
        self.max_future_s = float(max_future_s)
        self.suspend_count = max(1, int(suspend_count))
        self.resume_count = max(1, int(resume_count))
        self.recent = collections.deque(maxlen=max(1, int(window)))
        self.fresh_run = 0
        self.suspended = False
        self.pending = None          # (stamp_ns, item)
        self.last_sent_stamp = None
        self.last_sent_ns = None
        self.counts = collections.Counter()
        self.ages = []
        self.events = []

    def _age_s(self, stamp_ns, now_ns):
        return (int(now_ns) - int(stamp_ns)) / 1e9

    def _fresh(self, age_s):
        return -self.max_future_s <= age_s <= self.max_age_s

    def _send(self, stamp_ns, now_ns, item):
        age_s = self._age_s(stamp_ns, now_ns)
        self.last_sent_stamp = int(stamp_ns)
        self.last_sent_ns = int(now_ns)
        self.pending = None
        self.counts['sent'] += 1
        return [(item, age_s)]

    def offer(self, stamp_ns, now_ns, item):
        age_s = self._age_s(stamp_ns, now_ns)
        self.ages.append(age_s)
        stale = not self._fresh(age_s)
        self.recent.append(stale)
        if stale:
            self.counts['stale'] += 1
            self.fresh_run = 0
            if not self.suspended and sum(self.recent) >= self.suspend_count:
                self.suspended = True
                self.pending = None
                self.events.append(('suspended', sum(self.recent), len(self.recent)))
            return self.poll(now_ns)
        self.fresh_run += 1
        if self.suspended:
            if self.fresh_run < self.resume_count:
                self.counts['suspended'] += 1
                return []
            self.suspended = False
            self.recent.clear()
            self.events.append(('resumed', self.fresh_run, self.resume_count))
        newest = max([s for s in (self.last_sent_stamp,
                                  self.pending[0] if self.pending else None)
                      if s is not None], default=None)
        if newest is not None and int(stamp_ns) <= newest:
            self.counts['out_of_order'] += 1
            return self.poll(now_ns)
        if (self.last_sent_ns is None
                or (int(now_ns) - self.last_sent_ns) / 1e9 >= self.min_interval_s):
            return self._send(stamp_ns, now_ns, item)
        if self.pending is not None:
            self.counts['superseded'] += 1
        self.pending = (int(stamp_ns), item)
        return []

    def poll(self, now_ns):
        if self.pending is None or self.suspended:
            return []
        if (int(now_ns) - self.last_sent_ns) / 1e9 < self.min_interval_s:
            return []
        stamp_ns, item = self.pending
        if not self._fresh(self._age_s(stamp_ns, now_ns)):
            self.pending = None
            self.counts['stale'] += 1
            return []
        return self._send(stamp_ns, now_ns, item)


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
            # Correction gate (CorrectionGate).
            'max_correction_age_s': 1.0,
            'max_future_s': 0.5,
            'min_send_interval_s': 0.2,
            'lag_window': 10,
            'lag_suspend_count': 5,
            'lag_resume_count': 5,
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
        self.gate = None
        self.switch = None
        self.armed = None
        # GPS2_TYPE as ArduPilot last confirmed it (None: unknown), and the
        # value of a request MAVROS has not answered yet (one at a time).
        self.gps2_type = None
        self.gps2_pending = None
        self.gps2_pending_since = None
        # The source set ArduPilot last accepted (it boots on set 1).
        self.source_set = int(self.get_parameter('nominal_source_set').value)
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
            self.gate = CorrectionGate(
                self.get_parameter('max_correction_age_s').value,
                self.get_parameter('min_send_interval_s').value,
                self.get_parameter('lag_window').value,
                self.get_parameter('lag_suspend_count').value,
                self.get_parameter('lag_resume_count').value,
                self.get_parameter('max_future_s').value)
            self.create_subscription(
                NavSatFix, self.get_parameter('correction_topic').value,
                self._on_correction, 10)
            self.create_timer(0.05, self._release_correction)
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
        fields = gps_input_fields(
            fix, self.get_parameter('gps_id').value,
            self.get_parameter('satellites_visible').value,
            self.get_parameter('min_horizontal_accuracy_m').value)
        if fields is None:
            self.get_logger().warning('Dropped correction without a usable '
                                      'horizontal position/covariance')
            return
        now_ns = self.get_clock().now().nanoseconds
        stale_before = self.gate.counts['stale']
        out = self.gate.offer(stamp_ns(fix.header.stamp), now_ns, (fix, fields))
        if self.gate.counts['stale'] > stale_before:
            age_s = (now_ns - stamp_ns(fix.header.stamp)) / 1e9
            self.get_logger().warning(
                f'Dropped correction {age_s:.2f} s old (max_correction_age_s '
                f'{self.gate.max_age_s:g})', throttle_duration_sec=5.0)
        self._send_corrections(out)

    def _release_correction(self):
        if self.forwarding and self.gate is not None:
            self._send_corrections(self.gate.poll(self.get_clock().now().nanoseconds))

    def _send_corrections(self, out):
        for kind, count, size in self.gate.events:
            if kind == 'suspended':
                self.get_logger().error(
                    f'GPS_INPUT corrections SUSPENDED: {count} of the last '
                    f'{size} reached the bridge more than '
                    f'{self.gate.max_age_s:g} s late (the companion is '
                    'lagging); none are sent until '
                    f'{self.gate.resume_count} arrive in time in a row. With '
                    'GPS 1 lost ArduPilot dead-reckons and its EKF failsafe '
                    'lands the aircraft after about 8 s without corrections.')
            else:
                self.get_logger().warn(
                    f'GPS_INPUT corrections resumed: {count} in time in a row')
        self.gate.events.clear()
        for (fix, fields), _age_s in out:
            msg = GPSINPUT()
            msg.header.stamp = fix.header.stamp
            msg.header.frame_id = 'map'
            for name, value in fields.items():
                setattr(msg, name, value)
            self.gps_input_pub.publish(msg)
            self.sent += 1
            if self.sent == 1 or self.sent % 20 == 0:
                ages = sorted(self.gate.ages) or [math.nan]
                self.gate.ages.clear()
                counts = self.gate.counts
                self.get_logger().info(
                    f'Sent {self.sent} GPS_INPUT corrections (last horizontal '
                    f"accuracy {fields['horiz_accuracy']:.1f} m; age at the "
                    f'bridge of the {len(ages)} arrivals since the last report: '
                    f'min {ages[0]:.3f}, median {ages[len(ages) // 2]:.3f}, '
                    f'p95 {ages[int(0.95 * (len(ages) - 1))]:.3f}, max {ages[-1]:.3f} s; '
                    f"dropped stale {counts['stale']}, out of order "
                    f"{counts['out_of_order']}, superseded {counts['superseded']}, "
                    f"while suspended {counts['suspended']})")

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
        """Ask MAVROS to set GPS2_TYPE, one request at a time; the answer
        updates ``gps2_type`` (a request MAVROS never answers is given up
        after 10 s)."""
        now_s = self.get_clock().now().nanoseconds / 1e9
        if self.gps2_pending is not None:
            if now_s - self.gps2_pending_since < 10.0:
                return
            self.gps2_pending = None
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
        self.gps2_pending = value
        self.gps2_pending_since = now_s

    def _gps2_type_done(self, future, value):
        if self.gps2_pending == value:
            self.gps2_pending = None
        response = future.result()
        if response is None or not response.success:
            self.get_logger().error(
                f"Could not set {self.get_parameter('gps2_type_param').value}"
                f'={value}; will retry')
            self.gps2_type = None
            return
        self.gps2_type = value
        self.get_logger().warn(
            f"{self.get_parameter('gps2_type_param').value}={value} "
            f"({'GPS 2 = GPS_INPUT corrections' if value else 'GPS 2 disabled'})")

    def _on_gps1(self, msg):
        if not self.forwarding:
            return
        requested = self.switch.update(
            msg.fix_type >= GPS_FIX_TYPE_3D_FIX,
            self.get_clock().now().nanoseconds / 1e9)
        if requested == self.switch.denied_set:
            # Source set 2 first; GPS 2 follows once ArduPilot runs it.
            self._request_source_set(requested)
        elif requested == self.switch.nominal_set:
            gps2_maybe_on = self.gps2_type != GPS_TYPE_NONE or self.gps2_pending not in (
                None, GPS_TYPE_NONE)
            if self.manage_gps2 and gps2_maybe_on:
                # GPS 2 goes first: set 1 is asked for on a later message,
                # once GPS2_TYPE=0 is confirmed (the switch asks again).
                self._set_gps2_type(GPS_TYPE_NONE)
            else:
                self._request_source_set(requested)
        if (self.manage_gps2 and self.armed
                and self.switch.current == self.switch.denied_set
                and self.source_set == self.switch.denied_set
                and self.gps2_type != GPS_TYPE_MAV):
            self._set_gps2_type(GPS_TYPE_MAV)

    def _request_source_set(self, value):
        if not self.command_client.service_is_ready():
            return
        request = CommandLong.Request()
        request.command = MAV_CMD_SET_EKF_SOURCE_SET
        request.param1 = float(value)
        future = self.command_client.call_async(request)
        previous = self.switch.current
        future.add_done_callback(
            lambda done, value=value, previous=previous:
            self._source_set_done(done, value, previous))
        # Assume success until told otherwise so the request is not spammed.
        self.switch.current = value

    def _source_set_done(self, future, value, previous):
        response = future.result()
        if response is None or not response.success:
            self.get_logger().error(f'ArduPilot rejected EKF source set {value}')
            # Retry on the next GPS status message.
            if self.switch.current == value:
                self.switch.current = previous
            return
        self.source_set = value
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
