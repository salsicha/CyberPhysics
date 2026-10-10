"""applications/aerostack2/scripts/validate_aerodrone_navsim.py: message
checks, freshness bookkeeping and per-stack wiring, with stubbed messages
(the script imports ROS only when it runs)."""
import math
import re
import shlex
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml

from conftest import REPO, load_script

V = load_script(REPO / 'applications/aerostack2/scripts/validate_aerodrone_navsim.py',
                'validate_aerodrone_navsim')

HEAVY_STACKS = ('aerostack2_sim.yaml', 'aerostack2_genesis.yaml', 'aerostack2_isaac.yaml')
TOPIC_ENV = ('GPS_TOPIC', 'RAW_ODOM_TOPIC', 'ODOM_TOPIC', 'IMU_TOPIC', 'WILDNAV_IMAGE_TOPIC',
             'WILDNAV_CAMERA_INFO_TOPIC', 'DEPTH_TOPIC', 'CAMERA_INFO_TOPIC',
             'RANGEFINDER_TOPIC', 'NAVIGATION_ODOM_TOPIC', 'WILDNAV_RATE_HZ', 'INITIAL_LAT',
             'INITIAL_LON', 'NAVSIM_CAMERA_FRAME', 'NAVSIM_REQUIRED_TF',
             'NAVSIM_REQUIRE_VALID', 'NAVSIM_REQUIRE_CORRECTION')


@pytest.fixture(autouse=True)
def clean_env(monkeypatch):
    for name in TOPIC_ENV:
        monkeypatch.delenv(name, raising=False)


# --- stub messages ----------------------------------------------------------
def header(frame='frame', sec=100, nanosec=0):
    return NS(frame_id=frame, stamp=NS(sec=sec, nanosec=nanosec))


def fix(lat=37.9234, lon=-122.5967, alt=750.0, variance=25.0, cov_type=2,
        status=0, frame='map', cov=None):
    if cov is None:
        cov = [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, 2.0 * variance]
    return NS(header=header(frame), latitude=lat, longitude=lon, altitude=alt,
              position_covariance=list(cov), position_covariance_type=cov_type,
              status=NS(status=status, service=1))


def odom(x=1.0, q=(0.0, 0.0, 0.0, 1.0), variance=25.0, frame='odom',
         child='base_link', stamp=100):
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = variance
    covariance[14] = 4.0
    vector = NS(x=0.0, y=0.0, z=0.0)
    return NS(header=header(frame, stamp), child_frame_id=child,
              pose=NS(pose=NS(position=NS(x=x, y=2.0, z=3.0),
                              orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3])),
                      covariance=covariance),
              twist=NS(twist=NS(linear=vector, angular=vector)))


def imu(accel=(0.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.1), q=(0.0, 0.0, 0.0, 1.0)):
    return NS(header=header('base_link'),
              orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3]),
              orientation_covariance=[1.0] * 9,
              angular_velocity=NS(x=gyro[0], y=gyro[1], z=gyro[2]),
              linear_acceleration=NS(x=accel[0], y=accel[1], z=accel[2]))


def image(width=8, height=6, encoding='rgb8', frame='cam', data=None, step=None,
          stamp=100):
    step = width * 3 if step is None else step
    data = bytes(step * height) if data is None else data
    return NS(header=header(frame, stamp), width=width, height=height,
              encoding=encoding, step=step, data=data, is_bigendian=0)


def depth(values, frame='cam', encoding='32FC1', pad=0, bigendian=False):
    values = np.asarray(values)
    dtype = {'32FC1': 'f4', '16UC1': 'u2'}[encoding]
    dtype = ('>' if bigendian else '<') + dtype
    rows = values.astype(dtype)
    if pad:
        rows = np.concatenate([rows.view(np.uint8).reshape(rows.shape[0], -1),
                               np.zeros((rows.shape[0], pad), np.uint8)], axis=1)
    data = rows.tobytes()
    height, width = values.shape
    msg = image(width, height, encoding, frame, data,
                step=width * np.dtype(dtype).itemsize + pad)
    msg.is_bigendian = int(bigendian)
    return msg


def camera_info(width=8, height=6, frame='cam', fx=5.0, cx=None, cy=None):
    cx = width / 2.0 if cx is None else cx
    cy = height / 2.0 if cy is None else cy
    return NS(header=header(frame), width=width, height=height,
              k=[fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0], d=[0.0] * 5)


def rangefinder(value=50.0, low=0.2, high=180.0):
    return NS(header=header('rangefinder'), range=value, min_range=low, max_range=high)


def origin(lat=37.9234, lon=-122.5967, alt=748.8):
    return NS(header=header('earth'), position=NS(latitude=lat, longitude=lon, altitude=alt))


def scalar(value):
    return NS(data=value)


# --- message checks -----------------------------------------------------------
def test_healthy_messages_pass():
    assert V.check_navsatfix(fix()) is None
    assert V.check_navsatfix(fix(), require_covariance=True, require_fix=True,
                             max_sigma_m=1000.0) is None
    assert V.check_odometry(odom()) is None
    assert V.check_odometry(odom(), require_covariance=True, max_sigma_m=1000.0,
                            expected_frame='odom') is None
    # MAVROS imu/data: no acceleration (covariance -1), still a valid stream.
    assert V.check_imu(imu()) is None
    assert V.check_image(image()) is None
    assert V.check_image(depth(np.full((6, 8), 100.0)), depth=True) is None
    assert V.check_camera_info(camera_info()) is None
    assert V.check_range(rangefinder()) is None
    assert V.check_origin(origin(), expected=(37.9234, -122.5967)) is None
    assert V.check_confidence(scalar(0.0)) is None
    assert V.check_confidence(scalar(1.0)) is None
    for source in V.CORRECTION_SOURCES:
        assert V.check_correction_source(scalar(source)) is None


@pytest.mark.parametrize('msg,kwargs,expected', [
    (fix(lat=math.nan), {}, 'non-finite'),
    (fix(lat=95.0), {}, 'out of range'),
    (fix(lat=0.0, lon=0.0), {}, 'null position'),
    (fix(cov_type=0), {'require_covariance': True}, 'type UNKNOWN'),
    (fix(variance=-1.0), {}, 'negative variance'),
    (fix(variance=math.inf), {}, 'not finite'),
    (fix(variance=0.0), {'require_covariance': True}, 'zero variance'),
    (fix(variance=2000.0 ** 2), {'max_sigma_m': 1000.0}, 'sigma 2000 m exceeds 1000 m'),
    (fix(cov=[1, 0.5, 0, 0, 1, 0, 0, 0, 1]), {'require_covariance': True}, 'not symmetric'),
    (fix(status=-1), {'require_fix': True}, 'NO_FIX'),
    (fix(frame=''), {}, 'empty header.frame_id'),
])
def test_navsatfix_problems(msg, kwargs, expected):
    assert expected in V.check_navsatfix(msg, **kwargs)


def test_raw_gps_with_unknown_covariance_is_accepted():
    # Raw receivers may not know their covariance; outputs must.
    msg = fix(cov_type=0, cov=[math.nan] * 9)
    assert V.check_navsatfix(msg) is None
    assert 'UNKNOWN' in V.check_navsatfix(msg, require_covariance=True)


@pytest.mark.parametrize('msg,kwargs,expected', [
    (odom(x=math.nan), {}, 'non-finite position'),
    (odom(q=(0.0, 0.0, 0.0, 0.0)), {}, 'quaternion norm 0.000'),
    (odom(q=(0.0, 0.0, 0.0, math.inf)), {}, 'non-finite orientation'),
    (odom(variance=-4.0), {}, 'negative variance'),
    (odom(variance=0.0), {'require_covariance': True}, 'zero variance'),
    (odom(variance=1e8), {'require_covariance': True, 'max_sigma_m': 1000.0}, 'exceeds'),
    (odom(frame='map'), {'expected_frame': 'odom'}, "'map', expected 'odom'"),
    (odom(child=''), {}, 'empty child_frame_id'),
    (odom(stamp=0), {'require_covariance': True}, 'zero header.stamp'),
])
def test_odometry_problems(msg, kwargs, expected):
    assert expected in V.check_odometry(msg, **kwargs)


def test_raw_odometry_may_have_zero_covariance():
    assert V.check_odometry(odom(variance=0.0)) is None


def test_imu_problems():
    assert 'angular velocity' in V.check_imu(imu(gyro=(math.nan, 0.0, 0.0)))
    assert 'linear acceleration' in V.check_imu(imu(accel=(0.0, math.inf, 0.0)))
    assert 'orientation' in V.check_imu(imu(q=(math.nan, 0.0, 0.0, 1.0)))
    no_orientation = imu(q=(math.nan, 0.0, 0.0, 1.0))
    no_orientation.orientation_covariance[0] = -1.0
    assert V.check_imu(no_orientation) is None


def test_image_problems():
    assert 'empty image' in V.check_image(image(width=0))
    assert 'data bytes' in V.check_image(image(data=bytes(10)))
    assert 'empty encoding' in V.check_image(image(encoding=''))
    assert 'zero header.stamp' in V.check_image(image(stamp=0))
    assert 'empty header.frame_id' in V.check_image(image(frame=''))
    assert "expected 'cam_optical'" in V.check_image(image(), expected_frame='cam_optical')


def test_depth_needs_a_quarter_of_valid_pixels_like_demnav():
    values = np.full((6, 8), np.inf, dtype=np.float32)
    assert 'only 0% valid depth pixels' in V.check_image(depth(values), depth=True)
    values[:2, :] = 50.0  # 16 of 48 pixels
    assert V.check_image(depth(values), depth=True) is None
    values[1, :] = np.nan  # 8 of 48
    assert '17% valid' in V.check_image(depth(values), depth=True)


def test_depth_decoding_honours_row_padding_endianness_and_encoding():
    values = np.full((4, 5), 30.0)
    assert V.check_image(depth(values, pad=4), depth=True) is None
    assert V.check_image(depth(values, bigendian=True), depth=True) is None
    millimetres = np.zeros((4, 5))
    assert 'valid depth' in V.check_image(depth(millimetres, encoding='16UC1'), depth=True)
    millimetres[:, :2] = 1500
    assert V.check_image(depth(millimetres, encoding='16UC1'), depth=True) is None
    assert 'unsupported depth encoding' in V.check_image(image(), depth=True)


def test_camera_info_problems():
    assert 'empty camera_info' in V.check_camera_info(camera_info(width=0))
    assert 'focal length' in V.check_camera_info(camera_info(fx=0.0))
    assert 'principal point' in V.check_camera_info(camera_info(cx=-3.0))
    info = camera_info()
    info.k[0] = math.nan
    assert 'intrinsics' in V.check_camera_info(info)
    assert "expected 'other'" in V.check_camera_info(camera_info(), expected_frame='other')


def test_camera_pairs():
    assert V.check_camera_pair(image(), camera_info()) is None
    assert 'frame_id' in V.check_camera_pair(image(frame='a'), camera_info(frame='b'))
    assert 'image is 8x6 but camera_info is 16x12' in V.check_camera_pair(
        image(), camera_info(16, 12))
    # DemNav scales the intrinsics to the depth size: same aspect is enough.
    assert V.check_camera_pair(image(), camera_info(16, 12), exact_size=False) is None
    assert 'aspect' in V.check_camera_pair(image(), camera_info(16, 9), exact_size=False)


def test_rangefinder_follows_rep117():
    assert V.check_range(rangefinder(math.inf)) is None  # nothing in range
    assert 'NaN' in V.check_range(rangefinder(math.nan))
    assert 'outside' in V.check_range(rangefinder(500.0))
    assert 'limits' in V.check_range(rangefinder(low=5.0, high=1.0))


def test_origin_must_be_near_the_deployment_site():
    assert 'null origin' in V.check_origin(origin(0.0, 0.0))
    assert 'non-finite' in V.check_origin(origin(alt=math.nan))
    assert 'implausible' in V.check_origin(origin(alt=-5000.0))
    problem = V.check_origin(origin(38.9234), expected=(37.9234, -122.5967))
    assert '111.' in problem and 'limit 50 km' in problem
    assert V.check_origin(origin(38.9234)) is None  # no expectation configured


def test_scalar_checks():
    assert 'outside [0, 1]' in V.check_confidence(scalar(1.5))
    assert 'non-finite' in V.check_confidence(scalar(math.nan))
    assert 'unknown correction source' in V.check_correction_source(scalar('gps'))
    assert 'dead_reckoning' in V.check_correction_source(
        scalar('dead_reckoning'), require_correction=True)
    assert V.check_correction_source(scalar('demnav'), require_correction=True) is None


# --- bookkeeping ----------------------------------------------------------------
def test_stream_lifecycle_missing_one_message_fresh_stale_invalid():
    check = V.TopicCheck('fused fix', '/navigation/fix', 'navsatfix',
                         V.check_navsatfix, freshness_s=10.0)
    assert check.problem(0.0).startswith('missing')
    check.observe(fix(), 1.0)
    assert 'only one message' in check.problem(1.5)
    check.observe(fix(), 2.0)
    assert check.problem(5.0) is None
    assert 'stale: no valid message for 10.5 s (limit 10 s)' == check.problem(12.5)
    check.observe(fix(lat=math.nan), 13.0)
    assert check.problem(13.0) == 'invalid: non-finite latitude/longitude/altitude'
    # Still invalid while only bad messages arrive; recovers on a good one.
    check.observe(fix(), 14.0)
    assert check.problem(14.0) is None
    assert check.summary(14.0) == '4 msgs (0.2 Hz), 1 invalid'


def test_latched_and_optional_checks():
    latched = V.TopicCheck('EKF origin', '/navigation/origin', 'geopoint',
                           V.check_origin, freshness_s=None, latched=True)
    latched.observe(origin(), 1.0)
    assert latched.problem(1000.0) is None  # once is enough
    optional = V.TopicCheck('WildNav fix', '/wildnav/fix', 'navsatfix',
                            V.check_navsatfix, freshness_s=None, required=False)
    assert optional.problem(10.0) is None
    optional.observe(fix(lon=200.0), 11.0)
    assert 'out of range' in optional.problem(11.0)


def test_a_check_that_raises_is_reported_not_crashing():
    check = V.TopicCheck('x', '/x', 'image', V.check_image)
    check.observe(NS(), 1.0)
    assert 'unreadable message (AttributeError' in check.problem(1.0)


# --- configuration --------------------------------------------------------------
def build(argv=()):
    args = V.parse_args(list(argv))
    return args, V.build_validator(args)


def by_label(validator):
    return {check.label: check for check in validator.checks}


def test_topics_come_from_the_stack_environment(monkeypatch):
    monkeypatch.setenv('GPS_TOPIC', '/vehicle/gps')
    monkeypatch.setenv('RAW_ODOM_TOPIC', '/vehicle/odom')
    monkeypatch.setenv('DEPTH_TOPIC', '/vehicle/depth')
    monkeypatch.setenv('NAVIGATION_ODOM_TOPIC', '/vehicle/navigation/odometry')
    monkeypatch.setenv('RANGEFINDER_TOPIC', '/vehicle/range')
    monkeypatch.setenv('WILDNAV_RATE_HZ', '0.1')
    monkeypatch.setenv('INITIAL_LAT', '10.5')
    monkeypatch.setenv('INITIAL_LON', '20.5')
    monkeypatch.setenv('NAVSIM_REQUIRED_TF', 'earth:vehicle/map,vehicle/map:vehicle/odom')
    args, validator = build()
    checks = by_label(validator)
    assert checks['MAVROS GPS'].topic == '/vehicle/gps'
    assert checks['MAVROS odometry'].topic == '/vehicle/odom'
    assert checks['DemNav depth'].topic == '/vehicle/depth'
    assert checks['rangefinder'].topic == '/vehicle/range'
    # The fusion node's sibling outputs follow its odometry topic.
    assert checks['fused fix'].topic == '/vehicle/navigation/fix'
    assert checks['correction source'].topic == '/vehicle/navigation/correction_source'
    assert checks['WildNav valid'].freshness_s == 40.0  # four 0.1 Hz periods
    assert checks['DemNav valid'].freshness_s == 10.0
    assert checks['EKF origin'].latched and checks['EKF origin'].freshness_s is None
    assert checks['DemNav confidence'].freshness_s is None  # only on a match
    assert not checks['DemNav fix'].required
    assert (args.expected_lat, args.expected_lon) == (10.5, 20.5)
    assert args.required_tf == [('earth', 'vehicle/map'), ('vehicle/map', 'vehicle/odom')]


def test_command_line_overrides_and_disables_the_rangefinder(monkeypatch):
    monkeypatch.setenv('RANGEFINDER_TOPIC', '/aerodrone/rangefinder')
    _, validator = build(['--rangefinder-topic', '', '--depth-topic', '/d',
                          '--require-valid', 'demnav'])
    checks = by_label(validator)
    assert 'rangefinder' not in checks
    assert checks['DemNav depth'].topic == '/d'
    assert validator.require_valid == [checks['DemNav valid']]


def test_bad_tf_pair_is_a_usage_error():
    with pytest.raises(SystemExit):
        V.parse_args(['--required-tf', 'earth-base_link'])


class Feed:
    """Healthy messages for every topic of a validator, by kind."""

    def __init__(self, validator):
        self.validator = validator
        self.silent = set()

    def message(self, check):
        rgb = image(8, 6, 'rgb8', 'cam')
        return {
            'navsatfix': fix(), 'odometry': odom(), 'imu': imu(),
            'image': (depth(np.full((6, 8), 80.0)) if 'depth' in check.label else rgb),
            'camera_info': camera_info(8, 6, 'cam'), 'range': rangefinder(),
            'geopoint': origin(), 'bool': scalar(True), 'float32': scalar(0.6),
            'string': scalar('demnav'),
        }[check.kind]

    def tick(self, now):
        for check in self.validator.checks:
            if check.topic not in self.silent:
                self.validator.observe(check, self.message(check), now)


def test_healthy_stack_passes_and_stopping_demnav_is_named(monkeypatch):
    monkeypatch.setenv('INITIAL_LAT', '37.9234')
    monkeypatch.setenv('INITIAL_LON', '-122.5967')
    _, validator = build(['--require-valid', 'demnav', '--require-correction',
                          '--rangefinder-topic', ''])
    feed = Feed(validator)
    for now in (0.0, 1.0, 2.0):
        feed.tick(now)
    assert validator.problems(2.0) == []
    feed.silent.add('/demnav/valid')
    feed.tick(12.0)
    problems = validator.problems(12.5)
    assert problems == [('DemNav valid [/demnav/valid]: stale: no valid message for '
                         '10.5 s (limit 10 s)')]
    assert any('/demnav/valid' in line for line in validator.report(12.5))


def test_demnav_confidence_is_required_only_with_demnav_matches(monkeypatch):
    # DemNav publishes a confidence only for an accepted terrain match.
    monkeypatch.setenv('INITIAL_LAT', '37.9234')
    monkeypatch.setenv('INITIAL_LON', '-122.5967')
    for argv, problems in (
            ((), []),
            (('--require-valid', 'demnav'),
             ['DemNav confidence [/demnav/confidence]: missing: no message received'])):
        _, validator = build(['--rangefinder-topic', '', *argv])
        feed = Feed(validator)
        feed.silent.add('/demnav/confidence')
        for now in (0.0, 1.0):
            feed.tick(now)
        assert validator.problems(1.0) == problems
        assert by_label(validator)['WildNav confidence'].required
        validator.observe(by_label(validator)['DemNav confidence'], scalar(1.5), 1.0)
        assert 'outside [0, 1]' in validator.problems(1.0)[0]


def test_missing_topics_say_whether_anything_publishes_them():
    _, validator = build(['--depth-topic', '/typo'])
    problems = validator.problems(1.0, publishers=lambda topic: 0 if topic == '/typo' else 1)
    assert ('DemNav depth [/typo]: missing: no message received '
            '(no publisher on this topic)') in problems
    assert any('(1 publisher(s), none delivered)' in line for line in problems)


def test_pairs_and_required_matches_are_reported():
    _, validator = build(['--require-valid', 'demnav', 'wildnav'])
    feed = Feed(validator)
    feed.silent.add('/wildnav/valid')
    checks = by_label(validator)
    for now in (0.0, 1.0):
        feed.tick(now)
        validator.observe(checks['WildNav camera_info'], camera_info(16, 12, 'other'), now)
        validator.observe(checks['WildNav valid'], scalar(False), now)
    problems = validator.problems(1.0)
    assert problems == [
        ("WildNav image vs camera_info: image frame_id 'cam' differs from camera_info "
         "frame_id 'other'"),
        'WildNav valid [/wildnav/valid]: never reported a valid match (data: true)']


# --- stack wiring ---------------------------------------------------------------
def compose(path):
    return yaml.safe_load(path.read_text())['services']


def defaults(command):
    """A compose command with its ${VAR:-default} fallbacks substituted."""
    return re.sub(r'\$\{[A-Z_]+:-([^}]*)\}', r'\1', command)


def options(command):
    """--flag values of a validator command (a flag may take several)."""
    parsed, flag = {}, None
    for token in shlex.split(defaults(command)):
        if token.startswith('--'):
            flag = token[2:]
            parsed[flag] = []
        elif flag:
            parsed[flag].append(token)
    return parsed


def launch_args(command):
    return dict(token.split(':=', 1) for token in shlex.split(command) if ':=' in token)


@pytest.mark.parametrize('name', HEAVY_STACKS)
def test_heavy_stacks_run_the_validator_in_a_validation_profile(name):
    services = compose(REPO / 'compositions' / name)
    service = services['navsim_validator']
    assert service['profiles'] == ['validation']
    assert service['depends_on'] == {
        'flight_ready': {'condition': 'service_completed_successfully'}}
    assert service['network_mode'] == 'host' and service['ipc'] == 'host'
    # Topic names come from the same env files DemNav and WildNav use.
    assert service['env_file'] == services['demnav']['env_file']
    assert '../applications/aerostack2/scripts:/workspace/applications/aerostack2/scripts:ro' \
        in service['volumes']
    command = service['command']
    assert 'validate_aerodrone_navsim.py' in command
    opts = options(command)
    assert opts['rangefinder-topic'] == ['']  # no simulated rangefinder
    assert 'aerodrone/odom:aerodrone/base_link' in opts['required-tf']
    assert V.parse_args(shlex.split(defaults(command))[2:]).required_tf  # parses


def test_lite_stack_validator_watches_the_topics_its_matchers_consume():
    services = compose(REPO / 'tools/gps_denied_sitl.yaml')
    validator = services['validator']
    assert validator['profiles'] == ['validation']
    opts = {key: value[0] if len(value) == 1 else value
            for key, value in options(validator['command']).items()}
    demnav = launch_args(services['demnav']['command'])
    wildnav = launch_args(services['wildnav']['command'])
    assert opts['depth-topic'] == demnav['depth_topic']
    assert opts['depth-info-topic'] == demnav['camera_info_topic']
    assert opts['gps-topic'] == demnav['gps_topic'] == wildnav['gps_topic']
    assert opts['odom-topic'] == demnav['odom_topic'] == wildnav['raw_odom_topic']
    assert opts['image-topic'] == wildnav['image_topic']
    assert opts['image-info-topic'] == wildnav['camera_info_topic']
    assert opts['navigation-odom-topic'] == wildnav['navigation_output_topic']
    assert opts['camera-frame'] in services['camera']['command']
    assert opts['rangefinder-topic'] == '' and demnav['rangefinder_topic'] == "''"
    script = (REPO / 'tools/gps_denied_sitl_test.sh').read_text()
    assert 'run --rm -T --no-deps validator' in script
    assert 'EVENT at_altitude' in script


@pytest.mark.parametrize('name', HEAVY_STACKS)
def test_every_ros_service_joins_the_stacks_ros_domain(name):
    """With ROS_DOMAIN_ID set, a ROS service without it runs on domain 0,
    splitting the stack (the validator then sees no DemNav/WildNav/fusion)."""
    not_ros = {'demnav_assets', 'wildnav_assets', 'terrain_assets', 'navigator_sitl',
               'isaacsim_cache_init'}
    for service, spec in compose(REPO / 'compositions' / name).items():
        if service in not_ros:
            continue
        environment = spec.get('environment') or {}
        assert environment.get('ROS_DOMAIN_ID') == '${ROS_DOMAIN_ID:-0}', (name, service)
