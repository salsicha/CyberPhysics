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
    h = Harness(module, dead_reckoning_drift_mps=1.0, dead_reckoning_limit_s=60.0)
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
    h = Harness(module, dead_reckoning_drift_mps=1.0, dead_reckoning_limit_s=60.0)

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
        valid=True, confidence=1.0, east=float('nan'), received_ns=1)
    assert harness.node._target_offset()[2] == ''
    harness.node.demnav.east = 1.0
    harness.node.demnav.confidence = 0.35
    harness.node.demnav.variance = 1.7e308
    assert harness.node._target_offset()[2] == ''


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
