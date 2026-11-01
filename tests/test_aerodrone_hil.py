"""Aerodrone hardware-in-the-loop (tools/aerodrone_hil_test.md): the two
compose layers (workstation: tools/aerodrone_hil_sim.yaml over the
Gazebo stack; Pi: tools/aerodrone_hil_pi.yaml over the aircraft's),
their shared env file, the OAK-1 stand-in's logic, the probe, the resource
sampler's summary and the mission scorer."""
import json
import math
import os
import re
import shlex
import shutil
import subprocess

import numpy as np
import pytest
import yaml
from conftest import REPO, load_script

COMPOSITIONS = REPO / 'compositions'
HIL_ENV = REPO / 'applications/aerostack2/config/aerodrone_hil.env'
ENV_FILES = ['systems/aerodrone/config/topics.env',
             'applications/aerostack2/config/mount_tamalpais.env',
             'applications/aerostack2/config/aerodrone_hil.env']
SIM_FILES = ['compositions/aerostack2_sim.yaml', 'tools/aerodrone_hil_sim.yaml',
             'tools/tracking_sim_test.yaml', 'tools/aerodrone_hil_test.yaml']
PI_FILES = ['compositions/aerodrone_hardware.yaml', 'tools/aerodrone_hil_pi.yaml']
# What the Pi runs: the aircraft's companion stack, the router and the
# readiness gate (and the tree and the tracker with their profiles).
PI_SERVICES = {'hil_router', 'mavros', 'navigation_bridge', 'demnav', 'wildnav', 'aerostack',
               'aerodrone_message_rates', 'rosbridge', 'aerostack_gui', 'flight_ready'}
ONBOARD = {'mavros', 'navigation_bridge', 'demnav', 'wildnav', 'aerostack2_autonomy',
           'flight_ready', 'wilderness_mission', 'tracker', 'tracking_rosbridge', 'tracking_ui'}

standin = load_script(REPO / 'applications/aerostack2/scripts/hil_oak1_standin.py', 'hil_standin')
probe = load_script(REPO / 'tools/aerodrone_hil_probe.py', 'hil_probe')
resources = load_script(REPO / 'tools/aerodrone_hil_resources.py', 'hil_resources')
mission = load_script(REPO / 'tools/aerodrone_hil_mission_score.py', 'hil_mission_score')


def env_values(path=HIL_ENV):
    values = {}
    for line in path.read_text().splitlines():
        if line and not line.startswith('#') and '=' in line:
            key, _, value = line.partition('=')
            values[key] = value
    return values


def compose_config(files, profiles=()):
    if not shutil.which('docker'):
        pytest.skip('Docker Compose not installed')
    command = ['docker', 'compose', '-p', 'hil_test']
    for path in ENV_FILES:
        command += ['--env-file', str(REPO / path)]
    for path in files:
        command += ['-f', str(REPO / path)]
    for profile in profiles:
        command += ['--profile', profile]
    result = subprocess.run(command + ['config', '--format', 'json'], cwd=REPO, text=True,
                            capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    return json.loads(result.stdout)['services']


def command_text(service):
    command = service.get('command') or []
    return ' '.join(command) if isinstance(command, list) else command


def launch_args(command):
    """{name: value} of the name:=value arguments of a command."""
    return dict(re.findall(r'"?([A-Za-z_]+):=([^\s"]*)', command))


def ros_services(services):
    return {name: s for name, s in services.items()
            if 'ROS_DOMAIN_ID' in (s.get('environment') or {})
            or 'rmw_zenohd' in command_text(s)}


# --- the shared env file ----------------------------------------------------
def test_env_pins_every_link_to_the_wifi_addresses():
    env = env_values()
    sim, pi, port = env['HIL_SIM_HOST'], env['HIL_PI_HOST'], env['HIL_ZENOH_PORT']
    assert sim != pi and not sim.startswith('10.') and not pi.startswith('10.')
    assert env['ROS_DOMAIN_ID'] == '68'
    assert port not in ('7447', '7448', '7449')  # BlueOS's; another project's here
    zenoh = env['ZENOH_CONFIG_OVERRIDE']
    assert 'mode="peer"' in zenoh
    assert 'scouting/multicast/enabled=false' in zenoh
    assert 'listen/endpoints=["tcp/127.0.0.1:0"]' in zenoh
    assert 'connect/endpoints=["tcp/127.0.0.1:${HIL_ZENOH_PORT}"]' in zenoh
    # Gossip may connect a peer to other peers, never to a router (the
    # other host's) it hears of.
    assert 'scouting/gossip/autoconnect={router:[],peer:["peer"]}' in zenoh
    assert env['ZENOH_ROUTER_CHECK_ATTEMPTS'] == '0'
    assert env['BLUEOS_FCU_URL'] == 'tcp://${HIL_SIM_HOST}:5760'
    assert env['TRACKING_USE_SIM_TIME'] == 'false'


def test_layers_name_no_wired_or_wildcard_address():
    for path in ('tools/aerodrone_hil_sim.yaml', 'tools/aerodrone_hil_pi.yaml',
                 'tools/aerodrone_hil_test.yaml', 'applications/aerostack2/config/aerodrone_hil.env'):
        text = '\n'.join(line.split('#', 1)[0] for line in (REPO / path).read_text().splitlines())
        assert '10.0.0.' not in text, path
        assert '0.0.0.0' not in text, path


def test_hil_layers_are_plain_yaml_layers():
    """Layers, not stacks (no include), in plain YAML: the repository's
    checks (tools/check_compose_env_defaults.py, the composition tests) read
    every composition with yaml.safe_load, which has no Compose tags
    (!reset, !override). Only the test layer in tools/ touches the denial
    profile's services."""
    for path in ('tools/aerodrone_hil_sim.yaml', 'tools/aerodrone_hil_pi.yaml',
                 'tools/aerodrone_hil_test.yaml'):
        data = yaml.safe_load((REPO / path).read_text())
        assert 'include' not in data, path
    assert 'gps_denier' not in (REPO / 'tools/aerodrone_hil_sim.yaml').read_text()
    assert 'gps_denier' in (REPO / 'tools/aerodrone_hil_test.yaml').read_text()


# --- the workstation's side -------------------------------------------------
@pytest.fixture(scope='module')
def sim_stack():
    return {profile: compose_config(SIM_FILES, profiles)
            for profile, profiles in (('denial', ('validation', 'denial')),
                                      ('tracking', ('tracking',)))}


def running(services):
    """The services `up` starts (scale 0: none of their containers)."""
    return {name for name, service in services.items() if service.get('scale', 1) != 0}


def test_the_companion_stack_does_not_run_on_the_workstation(sim_stack):
    for services in sim_stack.values():
        assert not ONBOARD & running(services)
        assert ONBOARD & set(services) <= {n for n, s in services.items() if s.get('scale') == 0}
        assert {'gazebo', 'navigator_sitl', 'gazebo_ardupilot_backend', 'hil_router',
                'hil_oak1_standin'} <= running(services)
    assert {'gps_denier', 'gps_denial_recorder', 'navsim_validator'} <= running(sim_stack['denial'])
    # The runner starts the tracking driver (--scale) once the Pi is ready.
    assert sim_stack['tracking']['tracking_sim_test']['scale'] == 0
    assert {'tracking_detector', 'tracking_truth_boxes', 'tracking_launcher'} <= \
        running(sim_stack['tracking'])


def test_workstation_ros_nodes_are_loopback_peers_of_the_router(sim_stack):
    env = env_values()
    port = env['HIL_ZENOH_PORT']
    services = sim_stack['tracking'] | sim_stack['denial']
    for name, service in ros_services(services).items():
        if name in ONBOARD:
            continue  # scale 0 here
        assert service.get('ulimits', {}).get('nofile') == {'soft': 1024, 'hard': 524288}, name
        values = service['environment']
        if name == 'hil_router':
            assert values['ZENOH_CONFIG_OVERRIDE'].startswith(
                f'listen/endpoints=["tcp/127.0.0.1:{port}","tcp/{env["HIL_SIM_HOST"]}:{port}"];'
                'connect/endpoints=[]')
            assert 'scouting/gossip/target={router:["peer"],peer:["router"]}' in \
                values['ZENOH_CONFIG_OVERRIDE']
            continue
        assert values['ROS_DOMAIN_ID'] == '68', name
        assert values['RMW_IMPLEMENTATION'] == 'rmw_zenoh_cpp', name
        assert values['ZENOH_CONFIG_OVERRIDE'] == env['ZENOH_CONFIG_OVERRIDE'].replace(
            '${HIL_ZENOH_PORT}', port), name


def test_workstation_harness_waits_for_its_router_not_the_pi(sim_stack):
    """What depended on the companion stack depends on services with no
    container here (met at once) and on the router the Pi's nodes come
    through."""
    services = sim_stack['denial'] | sim_stack['tracking']
    for name in ('gps_denier', 'gps_denial_recorder', 'tracking_sim_test'):
        assert 'hil_router' in services[name]['depends_on'], name
    for name, service in services.items():
        if service.get('scale', 1) == 0:
            continue
        for dependency in service.get('depends_on') or {}:
            assert dependency not in ONBOARD or services[dependency]['scale'] == 0, name


def test_the_oak1_standin_turns_the_simulated_cameras_into_the_oak1_topics(sim_stack):
    services = sim_stack['tracking']
    args = dict(re.findall(r'-p (\w+):=(\S+)', command_text(services['hil_oak1_standin'])))
    assert args['output_root'] == '/oak1'
    assert args['depth_topic'].endswith('/downward_rgbd/depth')
    assert args['image_topic'].endswith('/downward_rgb/image_raw')
    assert args['rangefinder_topic'] == '/aerodrone/rangefinder'
    assert args['tracking_demand_topic'] == '/tracking/detections'
    prefix = env_values()['HIL_TRACKING_INPUT_PREFIX']
    assert args['tracking_input_prefix'] == prefix
    # The detector and the truth boxes read the same wall-stamped frames,
    # so the driver pairs them by stamp, and the Pi's tracker (wall time)
    # takes the detections.
    detector = dict(re.findall(r'-p (\w+):=(\S+)', command_text(services['tracking_detector'])))
    truth = dict(re.findall(r'-p (\w+):=(\S+)', command_text(services['tracking_truth_boxes'])))
    assert detector['image_topic'] == f'{prefix}/image_raw'
    assert detector['camera_info_topic'] == truth['camera_info_topic'] == f'{prefix}/camera_info'
    launcher = command_text(services['tracking_launcher'])
    assert '-p use_sim_time:=false' in launcher


def test_the_validator_checks_what_the_pis_navigation_reads(sim_stack):
    command = command_text(sim_stack['denial']['navsim_validator'])
    for flag, topic in (('--depth-topic', '/oak1/relative_depth'),
                        ('--depth-info-topic', '/oak1/camera_info'),
                        ('--image-topic', '/oak1/image_highres'),
                        ('--image-info-topic', '/oak1/image_highres/camera_info'),
                        ('--rangefinder-topic', '/aerodrone/rangefinder')):
        assert f'{flag} {topic}' in command


# --- the Pi's side ----------------------------------------------------------
@pytest.fixture(scope='module')
def pi_stack():
    return {'base': compose_config(PI_FILES),
            'mission': compose_config(PI_FILES, ('mission',)),
            'tracking': compose_config(PI_FILES, ('tracking',)),
            'aircraft': compose_config(['compositions/aerodrone_hardware.yaml'],
                                       ('tracking',))}


def test_the_pi_runs_the_aircrafts_companion_stack_without_its_sensors(pi_stack):
    assert running(pi_stack['base']) == PI_SERVICES
    assert running(pi_stack['mission']) == PI_SERVICES | {'behavior_tree'}
    assert running(pi_stack['tracking']) == PI_SERVICES | {'tracker'}
    assert {name for name, s in pi_stack['base'].items() if s.get('scale') == 0} == \
        {'oak1', 'rangefinder'}
    aircraft = pi_stack['aircraft']
    for name, service in pi_stack['tracking'].items():
        if service.get('scale') == 0:
            continue
        if name in aircraft:
            assert service['image'] == aircraft[name]['image'], name
            assert service['image'].endswith('-arm64'), name
        assert service.get('container_name', 'hil_').startswith('hil_'), name


def test_mavros_flies_the_workstations_sitl_over_wifi(pi_stack):
    env = env_values()
    command = command_text(pi_stack['base']['mavros'])
    assert f'fcu_url:=tcp://{env["HIL_SIM_HOST"]}:5760' in command
    assert '/workspace/mavros_config/blueos_mavros.launch.xml' in command  # the aircraft's


def test_pi_ros_nodes_are_loopback_peers_of_a_router_that_dials_out(pi_stack):
    env = env_values()
    port = env['HIL_ZENOH_PORT']
    services = pi_stack['mission'] | pi_stack['tracking']
    for name, service in ros_services(services).items():
        if service.get('scale') == 0:
            continue
        assert service.get('ulimits', {}).get('nofile') == {'soft': 1024, 'hard': 524288}, name
        values = service['environment']
        if name == 'hil_router':
            assert values['ZENOH_CONFIG_OVERRIDE'].startswith(
                f'listen/endpoints=["tcp/127.0.0.1:{port}"];'
                f'connect/endpoints=["tcp/{env["HIL_SIM_HOST"]}:{port}"]')
            continue
        assert values['ROS_DOMAIN_ID'] == '68', name
        assert values['ZENOH_CONFIG_OVERRIDE'] == env['ZENOH_CONFIG_OVERRIDE'].replace(
            '${HIL_ZENOH_PORT}', port), name


@pytest.mark.parametrize('name,added', [
    ('demnav', {'depth_is_metric': 'true', 'dem_resolution_m': '10.0'}),
    ('wildnav', {'attitude_sigma_deg': '0.5', 'match_sigma_floor_m': '1.0'}),
    ('aerostack', {'go_to_threshold': '3.0'}),
])
def test_pi_commands_are_the_aircrafts_plus_the_simulated_sensors(pi_stack, name, added):
    """Every argument the aircraft passes, with the aircraft's value
    rendered from the same env files, and only the listed additions."""
    aircraft = launch_args(command_text(pi_stack['aircraft'][name]))
    hil = launch_args(command_text(pi_stack['base'][name]))
    assert {k: v for k, v in hil.items() if k not in added} == aircraft
    assert {k: hil[k] for k in added} == added


def test_pi_navigation_reads_the_oak1_topics_on_the_simulated_site(pi_stack):
    demnav = launch_args(command_text(pi_stack['base']['demnav']))
    wildnav = launch_args(command_text(pi_stack['base']['wildnav']))
    assert demnav['depth_topic'] == '/oak1/relative_depth'
    assert demnav['rangefinder_topic'] == '/aerodrone/rangefinder'
    assert (demnav['initial_lat'], demnav['initial_lon']) == ('37.9234', '-122.5967')
    assert wildnav['image_topic'] == '/oak1/image_highres'
    assert wildnav['zoom'] == '17'
    assert wildnav['max_tile_downloads_per_cycle'] == '0'
    tracker = dict(re.findall(r'-p (\w+):=(\S+)', command_text(pi_stack['tracking']['tracker'])))
    assert tracker['use_sim_time'] == 'false'
    assert tracker['launcher_fitted'] == 'true'


def test_pi_mission_tree_is_the_wilderness_mission_once(pi_stack):
    tree = pi_stack['mission']['behavior_tree']
    assert tree['restart'] == 'no'
    # After flight_ready has passed (not only started), beside the
    # aircraft's own dependencies.
    assert tree['depends_on']['flight_ready']['condition'] == 'service_completed_successfully'
    assert {'mavros', 'aerostack'} <= set(tree['depends_on'])
    assert 'tree:=/hil/assets/wilderness_mission.xml' in command_text(tree)
    ready = pi_stack['base']['flight_ready']
    assert ready['restart'] == 'no'
    assert '--behavior-tree /hil/assets/wilderness_mission.xml' in command_text(ready)
    assert any(v['target'] == '/hil/assets' for v in ready['volumes'])


# --- the OAK-1 stand-in -----------------------------------------------------
def test_simulation_time_maps_to_the_wall_time_clock_reached_it():
    clock = standin.SimToWall()
    for i in range(101):  # 10 s of simulation at half real time
        clock.add(100.0 + 0.1 * i, 5000.0 + 0.2 * i)
    assert clock.to_wall(105.05) == pytest.approx(5010.1)
    assert clock.rate() == pytest.approx(0.5)
    assert clock.to_wall(110.5) == pytest.approx(5020.0 + 0.5 / 0.5)  # ahead: extrapolated
    assert clock.to_wall(99.0) is None
    clock.add(3.0, 6000.0)  # the simulation restarted
    assert clock.to_wall(105.0) is None


@pytest.mark.parametrize('rate', [4.6, 2.9, 0.5])
def test_rate_limiter_keeps_the_oak1_rate_from_the_cameras_5_hz(rate):
    limiter = standin.RateLimiter(rate)
    passed = sum(limiter.accept(0.2 * i) for i in range(5000))  # 1000 s at 5 Hz
    assert passed / 1000.0 == pytest.approx(rate, rel=0.01)


def test_depth_resized_to_megadepths_size_keeps_values_and_scales_intrinsics():
    depth = np.arange(240 * 320, dtype=np.float32).reshape(240, 320)
    depth[0, 0] = np.inf
    small = standin.resize_nearest(depth, 256, 192)
    assert small.shape == (192, 256) and small.dtype == np.float32
    assert set(np.unique(small[np.isfinite(small)])) <= set(np.unique(depth))
    assert np.isinf(small[0, 0])
    k = [277.0, 0.0, 159.5, 0.0, 277.0, 119.5, 0.0, 0.0, 1.0]
    p = [277.0, 0.0, 159.5, 0.0, 0.0, 277.0, 119.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    k2, p2 = standin.scale_intrinsics(k, p, 320, 240, 256, 192)
    assert k2[0] == pytest.approx(277.0 * 0.8) and k2[4] == pytest.approx(277.0 * 0.8)
    assert (k2[2], k2[5]) == pytest.approx((127.5, 95.5))  # still the image centre
    assert (p2[0], p2[2], p2[6]) == pytest.approx((k2[0], k2[2], k2[5]))
    # The field of view is unchanged.
    assert math.atan(160 / k[0]) == pytest.approx(math.atan(128 / k2[0]))


def test_rangefinder_reading_follows_the_tfmini_convention():
    depth = np.full((240, 320), 7.0, dtype=np.float32)
    assert standin.centre_range(depth) == pytest.approx(7.0)
    assert standin.centre_range(depth * 10.0) == math.inf     # beyond 12 m
    assert standin.centre_range(depth * 0.01) == -math.inf    # under 0.1 m
    depth[115:126, 155:166] = np.nan
    assert standin.centre_range(depth) == math.inf             # no return


def test_image_arrays_handle_row_padding_and_float_depth():
    class Msg:
        pass
    msg = Msg()
    msg.encoding, msg.height, msg.width, msg.is_bigendian = '32FC1', 2, 3, 0
    data = np.array([[1, 2, 3, 0], [4, 5, 6, 0]], dtype=np.float32)
    msg.step, msg.data = 16, data.tobytes()
    assert standin.image_array(msg).tolist() == [[1, 2, 3], [4, 5, 6]]
    msg.encoding, msg.step = 'rgb8', 12
    msg.data = bytes(range(24))
    assert standin.image_array(msg).shape == (2, 3, 3)
    assert standin.image_array(msg)[1, 0].tolist() == [12, 13, 14]


# --- probe, resources, mission score -----------------------------------------
def test_probe_reads_the_stamp_of_serialized_messages_and_reports_ages():
    raw = bytes([0, 1, 0, 0]) + (1_000).to_bytes(4, 'little') + (500_000_000).to_bytes(4, 'little')
    assert probe.cdr_stamp(raw + b'rest') == (1000, 500_000_000)
    big = bytes([0, 0, 0, 0]) + (7).to_bytes(4, 'big') + (0).to_bytes(4, 'big')
    assert probe.cdr_stamp(big) == (7, 0)
    stats = probe.TopicStats(stamped=True)
    for i in range(11):
        stamp = 2_000_000_000 + i * 0.2
        sec = int(stamp)
        payload = bytes([0, 1, 0, 0]) + sec.to_bytes(4, 'little') + \
            round((stamp - sec) * 1e9).to_bytes(4, 'little')
        stats.add(stamp + 0.15 + 0.02, payload + bytes(100), offset_s=0.02)
    summary = stats.summary(2.0)
    assert summary['count'] == 11 and summary['mean_bytes'] == 112
    assert summary['age_ms']['median'] == pytest.approx(150.0, abs=0.5)
    assert summary['max_gap_s'] == pytest.approx(0.2)
    sim = probe.TopicStats(stamped=True)
    sim.add(2_000_000_000.0, bytes([0, 1, 0, 0]) + (300).to_bytes(4, 'little') + bytes(4))
    assert sim.summary(1.0)['sim_stamped'] == 1 and 'age_ms' not in sim.summary(1.0)


def test_resource_summary_keeps_every_throttling_flag():
    rows = [{'t': float(i), 'cores': 4, 'cpu_pct': 40.0 + i, 'load1': 1.0, 'mem_used_mib': 2000,
             'mem_avail_mib': 6000 - i, 'temp_c': 70.0 + i, 'freq_mhz': [2400, 1500 + i],
             'throttled': '0x80000' if i == 3 else '0x0' if i < 9 else '0x80008',
             'containers': {'hil_demnav': {'cpu_pct': 30.0, 'mem_mib': 120.0}},
             'processes': {'ardupilot_navig': {'cpu_pct': 12.0}},
             'net': {'wlan0': {'rx_Bps': 1_000_000 + i, 'tx_Bps': 50_000}},
             'links': {'192.168.1.116:7468': {'rx_Bps': 900_000, 'tx_Bps': 40_000 + i}}}
            for i in range(10)]
    summary = resources.summarize(rows, 2.0, 8.0)
    assert summary['samples'] == 6
    assert summary['throttled_or'] == '0x80000'
    assert summary['throttled_now_share'] == 0.0
    assert summary['temp_c']['max'] == 77.0
    assert summary['freq_mhz_min'] == 1502
    assert summary['containers']['hil_demnav']['cpu_pct']['median'] == 30.0
    assert summary['processes']['ardupilot_navig']['cpu_pct']['max'] == 12.0
    assert summary['net']['wlan0']['rx_Bps']['max'] == 1_000_007
    assert summary['links']['192.168.1.116:7468']['tx_Bps']['median'] == 40_004.5


def test_resource_sampler_counts_the_bytes_of_tcp_sessions(monkeypatch):
    out = ('ESTAB 0 0 192.168.1.193:34812 192.168.1.116:7468\n'
           '\t cubic bytes_sent:5000 bytes_acked:4800 bytes_received:900000 segs_out:10\n'
           'ESTAB 0 0 192.168.1.193:34813 192.168.1.116:7468\n'
           '\t cubic bytes_acked:200 bytes_received:100\n')
    monkeypatch.setattr(resources.subprocess, 'run',
                        lambda *a, **k: type('R', (), {'stdout': out})())
    assert resources.tcp_bytes('192.168.1.116:7468') == (900_100, 5000)


def mission_rows(offset=(0.3, -0.4)):
    """A flight from home to two waypoints and back, in a recorder's rows."""
    lat0, lon0 = 37.9234, -122.5967
    home = (-720.0, -260.0)
    lat = lat0 + home[1] / 111_320.0
    lon = lon0 + home[0] / (111_320.0 * math.cos(math.radians(lat0)))
    rows = [{'k': 'meta', 'rx': 0.0, 'origin_lat': lat0, 'origin_lon': lon0},
            {'k': 'origin', 'rx': 1.0, 'lat': lat, 'lon': lon, 'alt': 719.0},
            {'k': 'state', 'rx': 0.0, 'mode': 'GUIDED', 'armed': False},
            {'k': 'state', 'rx': 10.0, 'mode': 'GUIDED', 'armed': True},
            {'k': 'state', 'rx': 100.0, 'mode': 'LAND', 'armed': False}]
    targets = [(10.0, (0.0, 0.0, 35.0)), (30.0, (100.0, 50.0, 60.0)), (60.0, (0.0, 0.0, 35.0))]
    rows += [{'k': 'setpoint', 'rx': rx, 'x': x, 'y': y, 'z': z} for rx, (x, y, z) in targets]
    for i in range(1100):
        rx = i * 0.1
        rows.append({'k': 'clock', 'rx': rx, 'sim': rx * 0.95})
        if rx < 10.0 or rx >= 95.0:
            local = (0.0, 0.0, 0.0)
        elif rx < 30.0:
            local = (0.0, 0.0, min(35.0, 3.5 * (rx - 10.0)))
        elif rx < 60.0:
            f = min(1.0, (rx - 30.0) / 20.0)
            local = (100.0 * f + offset[0] * f, 50.0 * f + offset[1] * f, 35.0 + 25.0 * f)
        else:
            f = min(1.0, (rx - 60.0) / 20.0)
            local = (100.0 * (1 - f), 50.0 * (1 - f), 60.0 - 25.0 * f if rx < 85 else 0.0)
        rows.append({'k': 'truth', 'rx': rx, 'e': home[0] + local[0], 'n': home[1] + local[1],
                     'u': -29.6 + local[2]})
    return rows


def test_mission_score_closest_approaches_timing_and_result():
    lines = ['[1.0]: WildernessMission  IDLE -> RUNNING', '[2.0]: GoTo  RUNNING -> SUCCESS',
             '[3.0]: GoTo  RUNNING -> SUCCESS', '[9.0]: WildernessMission  RUNNING -> SUCCESS']
    result = mission.score_mission(mission_rows(), lines, tree_exit=0)
    assert result['waypoints'] == 3 and result['waypoints_reached'] == 3
    second = result['per_waypoint'][1]
    assert second['closest_horizontal_m'] == pytest.approx(0.5, abs=0.02)
    assert second['to_reach_s'] == pytest.approx(19.5, abs=0.15)
    assert second['to_reach_sim_s'] == pytest.approx(0.95 * second['to_reach_s'], abs=0.1)
    assert result['armed_to_disarmed_s'] == 90.0
    assert result['tree_transitions']['GoTo']['SUCCESS'] == 2
    assert result['tree_last_transition'] == ('WildernessMission', 'SUCCESS')
    assert result['landed_and_disarmed'] and result['pass']


def test_mission_score_fails_a_missed_waypoint_or_a_failed_tree():
    assert not mission.score_mission(mission_rows(offset=(8.0, 0.0)), [], tree_exit=0)['pass']
    assert not mission.score_mission(mission_rows(), ['[9.0]: Root  RUNNING -> FAILURE'],
                                     tree_exit=1)['pass']
    with pytest.raises(mission.Unusable):
        mission.score_mission([{'k': 'meta', 'rx': 0.0}])


def test_runner_tears_both_sides_down_and_never_touches_blueos():
    text = (REPO / 'tools/aerodrone_hil_test.sh').read_text()
    assert 'trap teardown EXIT' in text and 'trap control_down EXIT' in text
    # The Pi, and the control mission, always with their volumes; the
    # workstation's keep them only with HIL_KEEP_VOLUMES=true.
    assert '$all_pi_profiles down -v --remove-orphans' in text
    assert '--profile denial down -v --remove-orphans' in text
    assert '"${all_sim_profiles[@]}" down "${down_args[@]}"' in text
    # Every profile is torn down, so a service started by name is too.
    for profile in ('validation', 'denial', 'tracking', 'tracking_truth', 'hil_probe'):
        assert f'--profile {profile}' in text.split('all_sim_profiles=(', 1)[1].split(')', 1)[0]
    for profile in ('mission', 'tracking', 'launcher', 'hil_probe'):
        assert f'--profile {profile}' in text.split('all_pi_profiles="', 1)[1].split('"', 1)[0]
    # The workstation's CPU sampler runs until stopped: never waited for.
    assert 'cpu_pid=$!' in text and 'bg_pids+=($!)\n' not in text.split('cpu --project', 1)[1][:200]
    assert 'docker compose -p hil_sim' in text and 'docker compose -p hil_pi' in text
    for forbidden in ('blueos-core', 'ardupilot-manager', 'sudo ', 'systemctl', 'reboot'):
        assert forbidden not in text, forbidden
    # Every SSH command goes through the one non-interactive form.
    assert all('BatchMode=yes' in line for line in text.splitlines()
               if re.search(r'(^|\s)ssh\s', line) and not line.lstrip().startswith('#'))
    shlex.split(text.split('SSH=(', 1)[1].split(')', 1)[0])


def test_tracking_test_layer_loads_the_middleware_env():
    """Every ROS service loads systems/ros2/middleware.env, so that
    ZENOH_CONFIG_OVERRIDE (a router, the HIL link) reaches the tracking
    test's driver and truth boxes too; without it they kept the image's
    multicast peer default and never met a routed graph."""
    services = yaml.safe_load((REPO / 'tools/tracking_sim_test.yaml').read_text())['services']
    for name in ('tracking_sim_test', 'tracking_truth_boxes'):
        assert '../systems/ros2/middleware.env' in services[name]['env_file'], name


def test_control_layer_is_the_gazebo_stack_with_the_hil_middleware():
    """tools/aerodrone_hil_control.yaml: every node on the workstation, as
    loopback peers of a loopback-only router, the stack otherwise unchanged."""
    if not shutil.which('docker'):
        pytest.skip('Docker Compose not installed')
    env = env_values()
    port = env['HIL_ZENOH_PORT']
    zenoh = env['ZENOH_CONFIG_OVERRIDE'].replace('${HIL_ZENOH_PORT}', port)
    environment = {**os.environ, 'HIL_ZENOH_PORT': port, 'ROS_DOMAIN_ID': '68',
                   'ZENOH_CONFIG_OVERRIDE': zenoh, 'ZENOH_ROUTER_CHECK_ATTEMPTS': '0'}
    command = ['docker', 'compose', '-p', 'hil_test', '-f', 'compositions/aerostack2_sim.yaml',
               '-f', 'tools/tracking_sim_test.yaml', '-f', 'tools/aerodrone_hil_control.yaml',
               '--profile', 'validation', '--profile', 'denial', '--profile', 'tracking',
               'config', '--format', 'json']
    result = subprocess.run(command, cwd=REPO, env=environment, text=True,
                            capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    services = json.loads(result.stdout)['services']
    base = compose_config(['compositions/aerostack2_sim.yaml'], ('validation', 'denial', 'tracking'))
    assert set(services) - set(base) == {'hil_router', 'tracking_sim_test', 'tracking_truth_boxes'}
    router = services['hil_router']['environment']['ZENOH_CONFIG_OVERRIDE']
    assert router.startswith(f'listen/endpoints=["tcp/127.0.0.1:{port}"];connect/endpoints=[]')
    assert 'scouting/multicast/enabled=false' in router
    for name, service in ros_services(services).items():
        if name != 'hil_router':
            assert service['environment']['ZENOH_CONFIG_OVERRIDE'] == zenoh, name
            assert service['environment']['ROS_DOMAIN_ID'] == '68', name
