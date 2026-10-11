"""Navigation fusion: shared EKF origin, stamp-matched corrections, honest
uncertainty/status, and the corrections handed to the autopilot bridge.

The node runs with ROS stubbed out: a fake Node base records publishers and
subscription callbacks so the tests drive it through its topics.
"""
import math
import sys
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import pytest

from conftest import REPO, install_ros_stubs, load_script
from synthetic_world import latlon_to_local, local_to_latlon

ORIGIN = (37.9234, -122.5967, 750.0)
WALL_S = 1_760_000_000  # raw odometry (MAVROS) stamps are wall-clock time


class FakeNode:
    def __init__(self, name):
        # The harness pre-initialises the node to inject parameter overrides.
        if hasattr(self, 'params'):
            return
        self.params = {}
        self.publishers = {}
        self.callbacks = {}
        self.now_ns = WALL_S * 1_000_000_000
        self.logger = MagicMock()

    def declare_parameter(self, name, value):
        self.params.setdefault(name, value)

    def get_parameter(self, name):
        return NS(value=self.params[name])

    def create_publisher(self, msg_type, topic, qos):
        return self.publishers.setdefault(topic, MagicMock())

    def create_subscription(self, msg_type, topic, callback, qos):
        self.callbacks[topic] = callback

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=self.now_ns))

    def get_logger(self):
        return self.logger


class NavSatStatus:
    STATUS_NO_FIX = -1
    STATUS_FIX = 0
    STATUS_SBAS_FIX = 1
    STATUS_GBAS_FIX = 2
    SERVICE_GPS = 1

    def __init__(self):
        self.status = self.STATUS_NO_FIX
        self.service = 0


class NavSatFix:
    COVARIANCE_TYPE_UNKNOWN = 0
    COVARIANCE_TYPE_DIAGONAL_KNOWN = 2

    def __init__(self):
        self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id='')
        self.status = NavSatStatus()
        self.latitude = self.longitude = self.altitude = 0.0
        self.position_covariance = [0.0] * 9
        self.position_covariance_type = self.COVARIANCE_TYPE_UNKNOWN


@pytest.fixture
def module():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('geographic_msgs', 'geographic_msgs.msg',
                     'rosgraph_msgs', 'rosgraph_msgs.msg'):
            sys.modules[name] = MagicMock()
        sys.modules['rclpy.node'].Node = FakeNode
        sys.modules['sensor_msgs.msg'] = NS(NavSatFix=NavSatFix, NavSatStatus=NavSatStatus)
        sys.modules['std_msgs.msg'] = NS(
            Bool=lambda data=False: NS(data=data),
            Float32=lambda data=0.0: NS(data=data),
            String=lambda data='': NS(data=data))
        yield load_script(
            REPO / 'applications/wildnav/src/wildnav_pkg/wildnav_pkg/navigation_fusion.py',
            'navigation_fusion_review')


def stamp(seconds):
    ns = int(round(seconds * 1e9))
    return NS(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def odometry(t, x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, child='base_link'):
    vector = lambda a=0.0, b=0.0: NS(x=a, y=b, z=0.0)  # noqa: E731
    return NS(
        header=NS(stamp=stamp(t), frame_id='odom'), child_frame_id=child,
        pose=NS(pose=NS(position=NS(x=x, y=y, z=z),
                        orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)),
                covariance=[0.0] * 36),
        twist=NS(twist=NS(linear=vector(vx, vy), angular=vector()),
                 covariance=[0.0] * 36))


def fix_at(t, east, north, sigma=5.0):
    fix = NavSatFix()
    fix.header.stamp = stamp(t)
    fix.status.status = NavSatStatus.STATUS_GBAS_FIX
    fix.latitude, fix.longitude = local_to_latlon(east, north, ORIGIN[0], ORIGIN[1])
    fix.altitude = 900.0
    fix.position_covariance = [sigma ** 2, 0, 0, 0, sigma ** 2, 0, 0, 0, 25.0]
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return fix


class Harness:
    def __init__(self, module, **params):
        self.module = module
        self.node = module.NavigationFusionNode.__new__(module.NavigationFusionNode)
        FakeNode.__init__(self.node, 'navigation_fusion_node')
        self.node.params.update(params)
        module.NavigationFusionNode.__init__(self.node)

    def topic(self, name):
        return self.node.publishers[name]

    def published(self, name):
        return [call.args[0] for call in self.topic(name).publish.call_args_list]

    def send(self, topic, msg):
        self.node.callbacks[topic](msg)

    def origin(self, lat=ORIGIN[0], lon=ORIGIN[1], alt=ORIGIN[2]):
        self.send('/navigation/origin', NS(position=NS(latitude=lat, longitude=lon, altitude=alt)))

    def raw(self, t, **kwargs):
        self.node.now_ns = int(t * 1e9)
        self.send('/drone0/sensor_measurements/odom', odometry(t, **kwargs))

    def source(self, name, fix, confidence=0.8, valid=True):
        self.send(f'/{name}/confidence', NS(data=confidence))
        self.send(f'/{name}/fix', fix)
        self.send(f'/{name}/valid', NS(data=valid))

    def last(self, name):
        return self.published(name)[-1]


@pytest.fixture
def harness(module):
    return Harness(module)


def test_topics_follow_the_output_namespace(module):
    h = Harness(module, output_topic='/airplane/navigation/odometry')
    for leaf in ('fix', 'correction_source', 'correction_confidence', 'correction_fix'):
        assert f'/airplane/navigation/{leaf}' in h.node.publishers
    assert '/airplane/navigation/autopilot_corrections_active' in h.node.callbacks
    assert '/navigation/origin' in h.node.callbacks


def test_without_origin_no_corrections_and_no_fix_are_published(harness):
    harness.raw(WALL_S + 0.0)
    harness.source('demnav', fix_at(WALL_S + 0.0, 20.0, 0.0))
    harness.raw(WALL_S + 0.1)
    assert harness.last('/navigation/correction_source').data == 'dead_reckoning'
    assert harness.last('/navigation/odometry').pose.pose.position.x == 0.0
    assert not harness.published('/navigation/fix')
    assert not harness.published('/navigation/correction_fix')


def test_fix_is_converted_about_the_ekf_origin_and_accepted(module):
    harness = Harness(module, correction_time_constant_s=0.001)
    harness.origin()
    harness.raw(WALL_S + 0.0, x=100.0, y=50.0)
    harness.raw(WALL_S + 0.1, x=100.0, y=50.0)
    harness.source('demnav', fix_at(WALL_S + 0.05, 112.0, 45.0))
    harness.raw(WALL_S + 0.2, x=100.0, y=50.0)
    fused = harness.last('/navigation/odometry').pose.pose.position
    assert fused.x == pytest.approx(112.0, abs=1e-3)
    assert fused.y == pytest.approx(45.0, abs=1e-3)
    assert harness.last('/navigation/correction_source').data == 'demnav'
    fix = harness.last('/navigation/fix')
    assert fix.status.status == NavSatStatus.STATUS_FIX
    assert fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    east, north = latlon_to_local(fix.latitude, fix.longitude, ORIGIN[0], ORIGIN[1])
    assert (east, north) == pytest.approx((112.0, 45.0), abs=1e-3)
    assert fix.altitude == pytest.approx(ORIGIN[2])


def test_fix_stamped_in_another_clock_domain_is_rejected(harness):
    """Simulator-time camera stamps must not be matched with the oldest
    wall-clock raw sample."""
    harness.origin()
    for step in range(20):
        harness.raw(WALL_S + 0.1 * step, x=float(step))
    harness.source('wildnav', fix_at(123.4, 40.0, 0.0))
    harness.raw(WALL_S + 2.1, x=21.0)
    assert harness.last('/navigation/correction_source').data == 'dead_reckoning'
    assert harness.last('/navigation/odometry').pose.pose.position.x == 21.0
    assert not harness.published('/navigation/correction_fix')


def test_fix_without_raw_sample_within_tolerance_is_rejected(harness):
    harness.origin()
    harness.raw(WALL_S + 0.0)
    harness.raw(WALL_S + 2.0)  # 2 s gap: no sample within 0.25 s of 1.0 s
    harness.source('demnav', fix_at(WALL_S + 1.0, 30.0, 0.0))
    harness.raw(WALL_S + 2.1)
    assert harness.last('/navigation/correction_source').data == 'dead_reckoning'


def test_raw_position_is_interpolated_at_the_fix_stamp(harness):
    harness.origin()
    harness.raw(WALL_S + 0.0, x=0.0)
    harness.raw(WALL_S + 0.2, x=2.0)
    harness.source('demnav', fix_at(WALL_S + 0.1, 1.0, 0.0))
    state = harness.node.demnav
    assert state.received_ns > 0
    assert state.east == pytest.approx(0.0, abs=1e-3)


def test_correction_fix_is_published_once_and_propagated_with_velocity(harness):
    harness.origin()
    for step in range(31):  # 3 s eastward at 5 m/s
        harness.raw(WALL_S + 0.1 * step, x=0.5 * step, vx=5.0)
    fix = fix_at(WALL_S + 1.0, 8.0, 3.0)  # raw was at x=5 then: 3 m east error
    harness.source('demnav', fix)
    harness.send('/demnav/valid', NS(data=True))  # duplicate validity: no resend
    corrections = harness.published('/navigation/correction_fix')
    assert len(corrections) == 1
    sent = corrections[0]
    east, north = latlon_to_local(sent.latitude, sent.longitude, ORIGIN[0], ORIGIN[1])
    assert east == pytest.approx(8.0 + 10.0, abs=0.05)
    assert north == pytest.approx(3.0, abs=0.05)
    assert harness.module.stamp_ns(sent.header.stamp) == int((WALL_S + 3.0) * 1e9)
    # Fix variance plus velocity uncertainty over the 2 s latency.
    assert sent.position_covariance[0] == pytest.approx(25.0 + (0.3 * 2.0) ** 2)
    assert math.isnan(sent.altitude)


def test_propagation_ignores_position_jumps_from_the_autopilot(harness):
    harness.origin()
    harness.raw(WALL_S + 0.0, x=0.0)
    harness.raw(WALL_S + 0.1, x=0.0)
    harness.raw(WALL_S + 0.2, x=40.0)  # EKF absorbed an earlier correction
    harness.source('wildnav', fix_at(WALL_S + 0.0, 5.0, 0.0))
    sent = harness.last('/navigation/correction_fix')
    east, _ = latlon_to_local(sent.latitude, sent.longitude, ORIGIN[0], ORIGIN[1])
    assert east == pytest.approx(5.0, abs=0.01)


def test_uncertainty_grows_and_status_degrades_without_corrections(module):
    # fix_variance_weight 1: a fix the estimate agrees with leaves its own
    # variance.
    h = Harness(module, dead_reckoning_drift_mps=1.0, dead_reckoning_limit_s=60.0,
                fix_variance_weight=1.0)
    h.origin()
    h.raw(WALL_S + 0.0)
    h.raw(WALL_S + 0.1)
    h.source('demnav', fix_at(WALL_S + 0.05, 0.0, 0.0, sigma=4.0))
    h.raw(WALL_S + 0.2)
    assert h.last('/navigation/odometry').pose.covariance[0] == pytest.approx(16.0, rel=0.01)
    h.raw(WALL_S + 30.2)
    assert h.last('/navigation/odometry').pose.covariance[0] == pytest.approx(16.0 + 900.0, rel=0.01)
    assert h.last('/navigation/fix').status.status == NavSatStatus.STATUS_FIX
    assert h.last('/navigation/fix').position_covariance[0] == pytest.approx(916.0, rel=0.01)
    # The correction timed out (30 s): the source is no longer active.
    h.raw(WALL_S + 31.0)
    assert h.last('/navigation/correction_source').data == 'dead_reckoning'
    h.raw(WALL_S + 61.0)
    assert h.last('/navigation/fix').status.status == NavSatStatus.STATUS_NO_FIX



def test_dead_reckoning_ages_on_the_simulator_clock(module):
    """Genesis and Isaac run slower than real time; the uncertainty and the
    NO_FIX limit follow their /clock, not wall time."""
    h = Harness(module, dead_reckoning_drift_mps=1.0, dead_reckoning_limit_s=60.0,
                fix_variance_weight=1.0)

    def at(wall_s):  # the simulator runs at a tenth of real time
        h.node.now_ns = int((WALL_S + wall_s) * 1e9)
        h.send('/clock', NS(clock=stamp(0.1 * wall_s)))
        h.raw(WALL_S + wall_s)

    at(0.0)
    h.origin()
    at(300.0)  # 30 s simulated
    assert h.last('/navigation/fix').status.status == NavSatStatus.STATUS_FIX
    assert h.last('/navigation/fix').position_covariance[0] == pytest.approx(100.0 + 30.0 ** 2)
    h.source('demnav', fix_at(WALL_S + 300.0, 0.0, 0.0, sigma=4.0))
    at(500.0)  # 20 s simulated since the correction
    assert h.last('/navigation/fix').position_covariance[0] == pytest.approx(16.0 + 20.0 ** 2)
    at(899.0)
    assert h.last('/navigation/fix').status.status == NavSatStatus.STATUS_FIX
    at(901.0)  # 60.1 s simulated
    assert h.last('/navigation/fix').status.status == NavSatStatus.STATUS_NO_FIX


def test_simulator_reset_falls_back_to_wall_time(module):
    h = Harness(module, dead_reckoning_drift_mps=1.0)
    h.node.now_ns = int(WALL_S * 1e9)
    h.send('/clock', NS(clock=stamp(500.0)))
    h.origin()
    h.node.now_ns = int((WALL_S + 10.0) * 1e9)
    h.send('/clock', NS(clock=stamp(1.0)))  # the simulator restarted
    h.raw(WALL_S + 10.0)
    assert h.last('/navigation/fix').position_covariance[0] == pytest.approx(100.0 + 10.0 ** 2)

def test_gbas_status_is_never_claimed(harness):
    harness.origin()
    harness.raw(WALL_S)
    harness.source('demnav', fix_at(WALL_S, 1.0, 1.0))
    harness.raw(WALL_S + 0.1)
    assert all(fix.status.status != NavSatStatus.STATUS_GBAS_FIX
               for fix in harness.published('/navigation/fix'))


def test_simultaneous_sources_are_reported_together(harness):
    harness.origin()
    harness.raw(WALL_S)
    harness.source('demnav', fix_at(WALL_S, 10.0, 0.0))
    harness.source('wildnav', fix_at(WALL_S, 10.0, 0.0))
    harness.raw(WALL_S + 0.1)
    assert harness.last('/navigation/correction_source').data == 'demnav+wildnav'


def test_autopilot_absorbing_corrections_disables_offsets(harness):
    harness.origin()
    harness.send('/navigation/autopilot_corrections_active', NS(data=True))
    harness.raw(WALL_S)
    harness.source('demnav', fix_at(WALL_S, 10.0, 0.0))
    harness.raw(WALL_S + 0.1)
    assert harness.last('/navigation/odometry').pose.pose.position.x == 0.0
    assert harness.last('/navigation/correction_source').data == 'demnav'
    assert len(harness.published('/navigation/correction_fix')) == 1


def test_origin_change_discards_corrections_made_about_the_old_origin(harness):
    harness.origin()
    harness.raw(WALL_S)
    harness.source('demnav', fix_at(WALL_S, 10.0, 0.0))
    harness.raw(WALL_S + 0.1)
    harness.origin(lat=ORIGIN[0] + 0.001)
    harness.raw(WALL_S + 0.2)
    assert harness.node.demnav.received_ns == 0
    assert harness.last('/navigation/correction_source').data == 'dead_reckoning'


def test_correction_beyond_maximum_is_rejected(harness):
    harness.origin()
    harness.raw(WALL_S)
    harness.source('demnav', fix_at(WALL_S, 400.0, 0.0))
    assert harness.node.demnav.received_ns == 0


@pytest.mark.parametrize('field', ['latitude', 'covariance', 'covariance_type', 'status'])
def test_invalid_fixes_are_rejected(harness, field):
    harness.origin()
    harness.raw(WALL_S)
    fix = fix_at(WALL_S, 10.0, 0.0)
    if field == 'latitude':
        fix.latitude = float('nan')
    elif field == 'covariance':
        fix.position_covariance[0] = float('inf')
    elif field == 'covariance_type':
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    else:
        fix.status.status = NavSatStatus.STATUS_NO_FIX
    harness.source('demnav', fix)
    assert harness.node.demnav.received_ns == 0
    assert not harness.published('/navigation/correction_fix')


@pytest.mark.parametrize('invalid', [float('nan'), float('inf'), -float('inf')])
def test_invalid_confidence_is_never_accepted(harness, invalid):
    harness.origin()
    harness.raw(WALL_S)
    harness.source('demnav', fix_at(WALL_S, 10.0, 0.0), confidence=invalid)
    assert harness.node.demnav.received_ns == 0


@pytest.mark.parametrize('field', ['position', 'velocity', 'covariance'])
def test_invalid_raw_odometry_is_not_published(harness, field):
    harness.origin()
    msg = odometry(WALL_S)
    if field == 'position':
        msg.pose.pose.position.z = float('nan')
    elif field == 'velocity':
        msg.twist.twist.linear.x = float('inf')
    else:
        msg.pose.covariance[0] = -1.0
    harness.send('/drone0/sensor_measurements/odom', msg)
    harness.topic('/navigation/odometry').publish.assert_not_called()
    assert not harness.node.raw_history


def test_recovers_invalid_internal_offset_without_restart(module):
    harness = Harness(module, correction_time_constant_s=0.001)
    harness.origin()
    harness.node.offset_east = float('nan')
    harness.node.offset_north = float('inf')
    harness.raw(WALL_S)
    harness.source('demnav', fix_at(WALL_S, 10.0, 0.0))
    harness.raw(WALL_S + 0.1)
    fused = harness.last('/navigation/odometry').pose.pose.position
    assert fused.x == pytest.approx(10.0, abs=1e-3)
    assert fused.y == pytest.approx(0.0, abs=1e-3)


def test_ignores_corrupted_source_state_and_extreme_variance(harness, module):
    harness.node.demnav = module.Correction(
        valid=True, confidence=1.0, accepted_confidence=1.0, east=float('nan'),
        received_ns=1, received_at=harness.node._now())
    assert harness.node._target_offset()[2] == ''
    harness.node.demnav.east = 1.0
    assert harness.node._target_offset()[2] == 'demnav'
    harness.node.demnav.variance = float('inf')
    assert harness.node._target_offset()[2] == ''
    harness.node.demnav.variance = 0.0
    assert harness.node._target_offset()[2] == ''


def no_match(harness, name, confidence=0.0):
    """A later frame without an accepted match, as DemNav/WildNav report it
    (WildNav also sends its confidence, 0 when nothing matched)."""
    if name == 'wildnav':
        harness.send('/wildnav/confidence', NS(data=confidence))
    harness.send(f'/{name}/valid', NS(data=False))


@pytest.mark.parametrize('name', ['demnav', 'wildnav'])
def test_correction_is_held_through_later_frames_without_a_match(module, name):
    """valid=false (and WildNav's zero confidence) for later frames does not
    withdraw an accepted correction: it stays active, and the offset keeps
    converging, until correction_timeout_s."""
    h = Harness(module, correction_time_constant_s=1.0)
    h.origin()
    h.raw(WALL_S + 0.0)
    h.raw(WALL_S + 0.1)
    h.source(name, fix_at(WALL_S + 0.05, 20.0, -10.0))
    for step in range(2, 100):  # 10 s of frames without a match
        if step % 2 == 0:
            no_match(h, name)
        h.raw(WALL_S + 0.1 * step)
        assert h.last('/navigation/correction_source').data == name
    fused = h.last('/navigation/odometry').pose.pose.position
    assert (fused.x, fused.y) == pytest.approx((20.0, -10.0), abs=0.01)
    assert h.last('/navigation/correction_confidence').data == pytest.approx(0.8)
    h.raw(WALL_S + 30.0)
    assert h.last('/navigation/correction_source').data == name
    h.raw(WALL_S + 30.2)  # 30.15 s after the correction: timed out
    assert h.last('/navigation/correction_source').data == 'dead_reckoning'
    # The offset itself stays: dead reckoning continues from the corrected
    # position.
    assert h.last('/navigation/odometry').pose.pose.position.x == pytest.approx(20.0, abs=1e-3)


def test_correction_timeout_ages_on_the_simulator_clock(module):
    """At a tenth of real time a correction stays active for 30 s of
    simulated flight (300 s of wall time)."""
    h = Harness(module)

    def at(wall_s):
        h.node.now_ns = int((WALL_S + wall_s) * 1e9)
        h.send('/clock', NS(clock=stamp(0.1 * wall_s)))
        h.raw(WALL_S + wall_s)

    at(0.0)
    h.origin()
    at(0.1)
    h.source('demnav', fix_at(WALL_S + 0.1, 5.0, 0.0))
    at(0.2)
    no_match(h, 'demnav')
    at(299.0)
    assert h.last('/navigation/correction_source').data == 'demnav'
    at(302.0)
    assert h.last('/navigation/correction_source').data == 'dead_reckoning'


def test_accepted_correction_keeps_its_own_confidence(harness):
    """A later rejected WildNav match (valid=false with its confidence) does
    not re-weight the correction accepted before it."""
    harness.origin()
    harness.raw(WALL_S)
    harness.source('wildnav', fix_at(WALL_S, 5.0, 0.0), confidence=0.3)
    no_match(harness, 'wildnav', confidence=0.9)
    harness.raw(WALL_S + 0.1)
    assert harness.last('/navigation/correction_source').data == 'wildnav'
    assert harness.last('/navigation/correction_confidence').data == pytest.approx(0.3)


def test_validated_fix_waiting_for_odometry_survives_the_next_frame(harness):
    """A fix newer than the raw odometry waits for it; the next frame's
    valid=false must not withdraw it."""
    harness.origin()
    harness.raw(WALL_S + 0.0, x=0.0)
    harness.source('demnav', fix_at(WALL_S + 0.6, 10.0, 0.0))
    no_match(harness, 'demnav')
    assert harness.node.demnav.received_ns == 0
    harness.raw(WALL_S + 1.0, x=10.0)
    assert harness.node.demnav.received_ns > 0
    assert harness.node.demnav.east == pytest.approx(4.0, abs=1e-6)
    assert harness.last('/navigation/correction_source').data == 'demnav'


@pytest.mark.parametrize('confidence', [0.8, 0.1])
def test_fix_its_source_reports_invalid_or_unconfident_is_not_used(harness, confidence):
    """valid=false for the fix's own frame (e.g. no EKF origin at the source
    yet), or a confidence under the minimum: the fix is never accepted."""
    harness.origin()
    harness.raw(WALL_S)
    harness.source('demnav', fix_at(WALL_S, 10.0, 0.0), confidence=confidence,
                   valid=confidence < 0.35)
    harness.raw(WALL_S + 0.1)
    assert harness.node.demnav.received_ns == 0
    assert harness.last('/navigation/correction_source').data == 'dead_reckoning'
    assert not harness.published('/navigation/correction_fix')


def test_body_frame_twist_is_rotated_to_enu(module):
    msg = odometry(0.0, vx=2.0)
    yaw = math.pi / 2
    msg.pose.pose.orientation = NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    east, north = module.enu_velocity(msg)
    assert (east, north) == pytest.approx((0.0, 2.0), abs=1e-9)
    msg.child_frame_id = 'odom'
    assert module.enu_velocity(msg) == pytest.approx((2.0, 0.0))


def test_simulator_clock_stamps_are_translated_to_wall_time(harness):
    """Gazebo/Isaac camera stamps are simulator time; /clock maps them."""
    harness.origin()
    sim0 = 100.0
    for step in range(31):
        t = 0.1 * step
        harness.node.now_ns = int((WALL_S + t) * 1e9)
        harness.send('/clock', NS(clock=stamp(sim0 + t)))
        harness.raw(WALL_S + t, x=float(step))
    harness.source('demnav', fix_at(sim0 + 1.0, 10.0 + 5.0, 0.0))
    state = harness.node.demnav
    assert state.received_ns > 0
    assert state.east == pytest.approx(5.0, abs=1e-3)  # raw x was 10 at sim 101 s


def test_stamp_outside_both_clock_histories_is_still_rejected(harness):
    harness.origin()
    harness.node.now_ns = int(WALL_S * 1e9)
    harness.send('/clock', NS(clock=stamp(100.0)))
    harness.send('/clock', NS(clock=stamp(101.0)))
    harness.raw(WALL_S)
    harness.source('demnav', fix_at(50.0, 10.0, 0.0))
    assert harness.node.demnav.received_ns == 0


def test_fix_newer_than_raw_odometry_waits_for_the_next_sample(harness):
    """Low-rate odometry (1 Hz MAVROS default streams): the fix is judged
    against interpolated raw position once a newer sample arrives."""
    harness.origin()
    harness.raw(WALL_S + 0.0, x=0.0)
    harness.source('demnav', fix_at(WALL_S + 0.6, 10.0, 0.0))
    assert harness.node.demnav.received_ns == 0
    harness.raw(WALL_S + 1.0, x=10.0)
    state = harness.node.demnav
    assert state.received_ns > 0
    assert state.east == pytest.approx(4.0, abs=1e-6)  # raw x was 6 at 0.6 s
    assert len(harness.published('/navigation/correction_fix')) == 1


def test_sources_are_weighted_by_inverse_variance_not_confidence(harness):
    """Confidence only admits a fix; the fixes' covariances weight them (a
    confidence factor on top counted the match quality twice)."""
    harness.origin()
    harness.raw(WALL_S)
    harness.source('demnav', fix_at(WALL_S, 10.0, 0.0, sigma=4.0),
                   confidence=0.9)
    harness.source('wildnav', fix_at(WALL_S, 0.0, 0.0, sigma=2.0),
                   confidence=0.3)
    east, _, source, _, variance = harness.node._target_offset()
    assert source == 'demnav+wildnav'
    # 1/16 and 1/4: DemNav weighs a fifth.
    assert east == pytest.approx(10.0 / 5.0, abs=1e-6)
    assert variance == pytest.approx(1.0 / (1.0 / 16.0 + 1.0 / 4.0))


def test_autopilot_gets_the_sources_combined_at_their_combined_rate(harness):
    """ArduPilot's EKF3 holds the largest GPS accuracy of the last seconds
    for every GPS_INPUT sample and fuses each as independent: interleaved,
    DemNav's dense fixes gave WildNav's their accuracy and outvoted them.
    Each accepted fix instead sends the inverse-variance combination of
    every source's latest fix, propagated to the latest raw sample."""
    harness.origin()
    for step in range(11):  # 1 s north at 4 m/s
        harness.raw(WALL_S + 0.1 * step, y=0.4 * step, vy=4.0)
    harness.source('wildnav', fix_at(WALL_S + 0.5, 2.0, 2.0, sigma=2.0))
    sent = harness.last('/navigation/correction_fix')
    east, north = latlon_to_local(sent.latitude, sent.longitude, ORIGIN[0], ORIGIN[1])
    assert (east, north) == pytest.approx((2.0, 4.0), abs=0.05)
    for step in range(11, 16):
        harness.raw(WALL_S + 0.1 * step, y=0.4 * step, vy=4.0)
    harness.source('demnav', fix_at(WALL_S + 1.5, 7.0, 6.0, sigma=4.0))
    sent = harness.last('/navigation/correction_fix')
    assert len(harness.published('/navigation/correction_fix')) == 2
    # WildNav's fix moved 4 m north since; weights 1/(4 + (0.3 x 1)^2)
    # and 1/16.
    w_wild, w_dem = 1.0 / (4.0 + 0.09), 1.0 / 16.0
    east, north = latlon_to_local(sent.latitude, sent.longitude, ORIGIN[0], ORIGIN[1])
    assert east == pytest.approx((2.0 * w_wild + 7.0 * w_dem) / (w_wild + w_dem), abs=0.05)
    assert north == pytest.approx((6.0 * w_wild + 6.0 * w_dem) / (w_wild + w_dem), abs=0.05)
    # Its variance, times the corrections sent in the last
    # correction_correlation_s (60 s): two.
    assert sent.position_covariance[0] == pytest.approx(2.0 / (w_wild + w_dem))
    # A fix older than max_fix_age_s (10 s) no longer counts.
    for step in range(16, 140):
        harness.raw(WALL_S + 0.1 * step, y=0.4 * step, vy=4.0)
    harness.source('demnav', fix_at(WALL_S + 13.9, 9.0, 55.6, sigma=4.0))
    sent = harness.last('/navigation/correction_fix')
    east, north = latlon_to_local(sent.latitude, sent.longitude, ORIGIN[0], ORIGIN[1])
    assert (east, north) == pytest.approx((9.0, 55.6), abs=0.05)
    assert sent.position_covariance[0] == pytest.approx(3 * 16.0)


def test_a_fix_inconsistent_with_the_other_source_is_left_out(harness):
    """A false WildNav match (224 m off in the lite stack, confidence just
    above its minimum) halved into the combination and sent with the
    stream's inflated variance passed the EKF's gate and pulled it 50 m.
    Fixes more than 5 combined sigma apart are not combined: the one
    farther from the fused position is left out."""
    harness.origin()
    for step in range(11):
        harness.raw(WALL_S + 0.1 * step)
    harness.source('demnav', fix_at(WALL_S + 0.9, 3.0, 1.0, sigma=4.0))
    harness.source('wildnav', fix_at(WALL_S + 1.0, 200.0, -90.0, sigma=5.0))
    sent = harness.last('/navigation/correction_fix')
    east, north = latlon_to_local(sent.latitude, sent.longitude, ORIGIN[0], ORIGIN[1])
    assert (east, north) == pytest.approx((3.0, 1.0), abs=0.05)
    assert sent.position_covariance[0] == pytest.approx(2 * 16.0, rel=0.01)
    # Within the gate both count.
    for step in range(11, 16):
        harness.raw(WALL_S + 0.1 * step)
    harness.source('wildnav', fix_at(WALL_S + 1.4, 9.0, 1.0, sigma=5.0))
    sent = harness.last('/navigation/correction_fix')
    east, _ = latlon_to_local(sent.latitude, sent.longitude, ORIGIN[0], ORIGIN[1])
    assert east == pytest.approx((3.0 / 16.0 + 9.0 / 25.0) / (1 / 16.0 + 1 / 25.0),
                                 abs=0.1)


def test_dense_corrections_share_one_correlation_time(module):
    """The fixes' errors persist for tens of seconds while the EKF fuses
    each GPS_INPUT sample as independent: n samples within
    correction_correlation_s are each sent with n times the variance, so
    a 5 Hz stream does not count 25 times one at 0.2 Hz."""
    h = Harness(module, correction_correlation_s=5.0)
    h.origin()
    variances = []
    for step in range(60):  # DemNav at 5 Hz for 12 s
        t = WALL_S + 0.2 * step
        h.raw(t)
        h.source('demnav', fix_at(t, 3.0, 0.0, sigma=4.0))
        variances.append(h.last('/navigation/correction_fix').position_covariance[0])
    assert variances[0] == pytest.approx(16.0)
    assert variances[4] == pytest.approx(5 * 16.0)
    assert variances[-1] == pytest.approx(25 * 16.0)
    # WildNav alone at 0.2 Hz: one sample per 5 s, sent as it is.
    h = Harness(module, correction_correlation_s=5.0)
    h.origin()
    for step in range(4):
        t = WALL_S + 5.0 * step
        h.raw(t)
        h.source('wildnav', fix_at(t, 3.0, 0.0, sigma=2.0))
        assert h.last('/navigation/correction_fix').position_covariance[0] == \
            pytest.approx(4.0)


def test_propagation_counts_simulated_time(module):
    """MAVROS stamps odometry in wall time but its velocity is per
    simulated second: at a fifth of real time, a fix 5 s (wall) old has
    moved with the vehicle for 1 s, not 5."""
    h = Harness(module)
    for step in range(61):  # 6 s of wall time at 0.2x, 4 m/s north
        t = 0.1 * step
        h.node.now_ns = int((WALL_S + t) * 1e9)
        h.send('/clock', NS(clock=stamp(100.0 + 0.2 * t)))
        h.raw(WALL_S + t, y=0.8 * t, vy=4.0)
        if step == 0:
            h.origin()
    h.source('wildnav', fix_at(WALL_S + 1.0, 0.0, 10.0, sigma=2.0))
    sent = h.last('/navigation/correction_fix')
    _, north = latlon_to_local(sent.latitude, sent.longitude, ORIGIN[0], ORIGIN[1])
    assert north == pytest.approx(10.0 + 4.0 * 0.2 * 5.0, abs=0.1)
    assert sent.position_covariance[0] == pytest.approx(4.0 + (0.3 * 1.0) ** 2, rel=0.01)


def test_corrections_thin_out_rather_than_exceed_the_ekf_accuracy_clip(module):
    """ArduPilot's EKF3 clips the GPS accuracy it fuses at 100 m: a sample
    sent with more would count as a 100 m one, more than its share. A
    correction whose inflated sigma would exceed max_correction_sigma_m is
    held back and not counted, so the stream thins out to the rate at which
    its sigma stays under the clip."""
    h = Harness(module, correction_correlation_s=5.1, max_correction_sigma_m=10.0)
    h.origin()
    sent = []
    for step in range(60):  # DemNav at 5 Hz, sigma 4 m, for 12 s
        t = WALL_S + 0.2 * step
        h.raw(t)
        before = len(h.published('/navigation/correction_fix'))
        h.source('demnav', fix_at(t, 3.0, 0.0, sigma=4.0))
        if len(h.published('/navigation/correction_fix')) > before:
            sent.append(round(t - WALL_S, 1))
    variances = [fix.position_covariance[0]
                 for fix in h.published('/navigation/correction_fix')]
    # 16 m^2 x n stays within 10^2: six per correlation time, the first
    # six as before, then one more as each leaves the window.
    assert variances[:6] == pytest.approx([16.0 * n for n in range(1, 7)])
    assert max(variances) <= 100.0
    assert sent[:7] == [0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 5.2]
    assert all(sum(1 for s in sent if t - 5.1 < s <= t) <= 6 for t in sent)
    assert len(sent) == 18


def test_a_correction_beyond_the_clip_on_its_own_is_not_sent(harness):
    """A fix whose own sigma exceeds the clip is never sent (EKF3 would fuse
    it as a 100 m one), and does not count against the next."""
    harness.origin()
    harness.raw(WALL_S)
    harness.source('demnav', fix_at(WALL_S, 3.0, 0.0, sigma=120.0))
    assert not harness.published('/navigation/correction_fix')
    harness.raw(WALL_S + 0.5)
    harness.source('demnav', fix_at(WALL_S + 0.5, 3.0, 0.0, sigma=4.0))
    assert harness.last('/navigation/correction_fix').position_covariance[0] == \
        pytest.approx(16.0)


def test_fused_uncertainty_is_the_distance_to_the_fixes_plus_part_of_their_own(module):
    """While the autopilot absorbs the corrections the fused estimate is its
    odometry, which can lag the fixes by tens of metres (the lite SITL test:
    12-30 m off while claiming the fixes' 4 m). Its variance per axis: the
    weighted mean square of (fix - estimate) at the accepted fixes, plus
    fix_variance_weight (0.5) times their variance, decaying over
    fix_error_time_constant_s (5 s)."""
    h = Harness(module)
    h.origin()
    h.send('/navigation/autopilot_corrections_active', NS(data=True))
    for step in range(201):  # 20 s, the EKF 20 m west of every fix
        t = WALL_S + 0.1 * step
        h.raw(t)
        if step % 5 == 0:
            h.source('demnav', fix_at(t, 20.0, 0.0, sigma=4.0))
    assert h.last('/navigation/fix').position_covariance[0] == pytest.approx(
        200.0 + 0.5 * 16.0, rel=0.01)
    for step in range(201, 301):  # 10 s, the fixes now agree with the EKF
        t = WALL_S + 0.1 * step
        h.raw(t)
        if step % 5 == 0:
            h.source('demnav', fix_at(t, 0.0, 0.0, sigma=4.0))
    # The last fix published follows 19 agreeing fixes over 9.5 s, 0.5 s
    # after the last of them (0.5 m/s of dead-reckoning growth).
    variance = h.last('/navigation/fix').position_covariance[0]
    assert variance == pytest.approx(8.0 + 200.0 * math.exp(-1.9) + 0.25 ** 2, rel=1e-3)
    assert h.last('/navigation/odometry').pose.covariance[0] == variance
    # A different origin starts the statistics again.
    h.origin(lat=ORIGIN[0] + 0.001)
    assert h.node.fix_error is None
