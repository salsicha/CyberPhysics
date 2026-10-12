"""GPS-denied simulation test wiring: the `denial` profile of the Gazebo
and Isaac compositions, its env defaults, the denier's simulation-time
schedule, the recorder's rows, the wrapper and the control override
(tools/gps_denied_sim_test.md)."""
import re
import shlex
import subprocess
from types import SimpleNamespace

import pytest
import yaml
from conftest import REPO, load_script

denier = load_script(REPO / 'applications/aerostack2/scripts/gps_denier.py', 'gps_denier_review')
recorder = load_script(REPO / 'applications/aerostack2/scripts/gps_denial_recorder.py',
                       'gps_denial_recorder_review')
env_check = load_script(REPO / 'tools/check_compose_env_defaults.py', 'env_defaults_review')

STACKS = {'gazebo': 'aerostack2_sim.yaml', 'isaac': 'aerostack2_isaac.yaml'}
TRUTH = {'gazebo': ('/aerodrone/ground_truth/pose', 'pose'),
         'isaac': ('/aerodrone/simulation/ground_truth/odom', 'odometry')}
ENV = REPO / 'applications/aerostack2/config/mount_tamalpais.env'
WINDOW = ('GPS_DENIAL_START_S', 'GPS_DENIAL_DURATION_S', 'GPS_DENIAL_RECOVERY_S')


def services(stack):
    return yaml.safe_load((REPO / 'compositions' / STACKS[stack]).read_text())['services']


def env_values():
    return env_check.parse_env_file(str(ENV))


def options(command):
    """--name value pairs of a command line."""
    words = shlex.split(command)
    return {word[2:]: words[i + 1] for i, word in enumerate(words[:-1])
            if word.startswith('--')}


# --- compose -------------------------------------------------------------
@pytest.mark.parametrize('stack', sorted(STACKS))
def test_denial_profile_adds_only_the_denier_and_recorder(stack):
    stack_services = services(stack)
    profiled = {name for name, service in stack_services.items()
                if 'denial' in (service.get('profiles') or [])}
    assert profiled == {'gps_denier', 'gps_denial_recorder'}
    for name in profiled:
        service = stack_services[name]
        assert service['profiles'] == ['denial']
        assert service['image'] == 'salsicha/cyberphysics:aerostack2-${CYBERPHYSICS_TAG:-latest}'
        assert service['network_mode'] == 'host' and service['ipc'] == 'host'
        assert service['depends_on'] == ['mavros']
        assert service['restart'] == 'no'
        volumes = service['volumes']
        assert ('../applications/aerostack2/scripts:'
                '/workspace/applications/aerostack2/scripts:ro') in volumes
        assert '${GPS_DENIED_OUTPUT:-../generated/gps_denied_sim}:/out' in volumes
        assert f'/workspace/applications/aerostack2/scripts/{name}.py' in service['command']


@pytest.mark.parametrize('stack', sorted(STACKS))
def test_recorder_records_this_stacks_ground_truth_about_the_deployment_origin(stack):
    command = options(services(stack)['gps_denial_recorder']['command'])
    assert command['stack'] == stack
    assert (command['truth-topic'], command['truth-type']) == TRUTH[stack]
    assert command['output'] == '/out/recording.jsonl'
    values = env_values()
    assert command['origin-lat'] == f"${{INITIAL_LAT:-{values['INITIAL_LAT']}}}"
    assert command['origin-lon'] == f"${{INITIAL_LON:-{values['INITIAL_LON']}}}"
    args = recorder.parse_args(['--origin-lat', '1', '--origin-lon', '2',
                                '--truth-type', command['truth-type']])
    assert args.clock_topic == '/clock'


@pytest.mark.parametrize('stack', sorted(STACKS))
def test_denier_window_comes_from_the_env_file_in_simulation_time(stack):
    command = options(services(stack)['gps_denier']['command'])
    values = env_values()
    for flag, variable in zip(('start-s', 'duration-s', 'recovery-s'), WINDOW):
        assert command[flag] == f'${{{variable}:-{values[variable]}}}'
    assert command['events'] == '/out/denial_events.jsonl'
    assert float(command['cruise-height-m']) == 140.0
    args = denier.parse_args(['--start-s', values[WINDOW[0]]])
    assert args.clock_topic == '/clock' and args.param == 'SIM_GPS_DISABLE'


def test_window_defaults_are_the_measured_setting_and_match_every_fallback():
    values = env_values()
    assert [float(values[name]) for name in WINDOW] == [60.0, 150.0, 60.0]
    assert env_check.main() == 0


def test_other_compositions_have_no_denial_profile():
    for path in sorted((REPO / 'compositions').glob('*.yaml')):
        if path.name in STACKS.values():
            continue
        assert 'gps_denier' not in path.read_text(), path.name


# --- denier schedule -------------------------------------------------------
def run_schedule(schedule, samples):
    """Feed (sim, height, armed) samples, carrying out every due action;
    returns [(action, sim)]."""
    done = []
    for sim, height, armed in samples:
        action = schedule.due(sim, height, armed)
        if action:
            schedule.mark(action, sim)
            done.append((action, sim))
    return done


def test_denial_window_is_simulation_time_after_cruise():
    schedule = denier.DenialSchedule(60.0, 150.0, 60.0, 140.0)
    # 0.1 s of simulation per sample, climbing 1 m per second; the
    # schedule never sees wall time, so a 0.2x simulator changes nothing.
    samples = [(round(0.1 * i, 1), 0.1 * i, True) for i in range(5000)]
    assert run_schedule(schedule, samples) == [
        (denier.CRUISE, 140.0), (denier.DENY, 200.0), (denier.RESTORE, 350.0),
        (denier.DONE, 410.0)]
    assert schedule.phase == 'GPS restored'
    assert schedule.due(1000.0, 500.0, True) is None


def test_nothing_happens_before_cruise_or_without_the_clock():
    schedule = denier.DenialSchedule(0.0, 10.0, 0.0, 140.0)
    assert schedule.due(None, 300.0, True) is None          # no /clock yet
    assert schedule.due(50.0, None, True) is None           # no position yet
    assert schedule.due(50.0, 300.0, False) is None         # not armed
    assert schedule.due(50.0, 139.9, True) is None
    assert schedule.phase == 'waiting for cruise'
    assert schedule.due(50.0, 140.0, True) == denier.CRUISE


def test_an_action_not_carried_out_stays_due():
    schedule = denier.DenialSchedule(5.0, 10.0, 0.0, 100.0)
    schedule.mark(denier.CRUISE, 0.0)
    assert schedule.due(4.9, 200.0, True) is None
    assert schedule.due(5.0, 200.0, True) == denier.DENY
    assert schedule.due(9.0, 200.0, True) == denier.DENY   # the parameter write failed
    schedule.mark(denier.DENY, 9.0)
    # The window runs from when GPS 1 was really disabled.
    assert schedule.due(18.9, 200.0, True) is None
    assert schedule.due(19.0, 200.0, True) == denier.RESTORE


@pytest.mark.parametrize('start,duration,recovery', [(-1, 10, 0), (0, 0, 0), (0, 10, -1)])
def test_invalid_windows_are_refused(start, duration, recovery):
    with pytest.raises(ValueError):
        denier.DenialSchedule(start, duration, recovery, 140.0)


def test_denier_and_recorder_import_without_ros():
    source = (REPO / 'applications/aerostack2/scripts/gps_denier.py').read_text()
    head = source.split('def run(')[0]
    assert 'rclpy' not in re.sub(r'""".*?"""', '', head, flags=re.DOTALL)
    assert 'time.monotonic' not in source.split('class DenialSchedule')[1].split('class EventLog')[0]


# --- recorder rows -----------------------------------------------------------
def stamp(seconds):
    return SimpleNamespace(sec=int(seconds), nanosec=round((seconds % 1) * 1e9))


def navsatfix(lat, lon, cov, status=0, t=12.5):
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp(t)), latitude=lat, longitude=lon,
                           altitude=800.0, status=SimpleNamespace(status=status),
                           position_covariance=cov)


def test_fix_rows_keep_the_horizontal_covariance_and_the_stamp():
    cov = [4.0, 0.5, 0.0, 0.5, 9.0, 0.0, 0.0, 0.0, 100.0]
    row = recorder.fix_row('demnav', navsatfix(37.9, -122.6, cov, status=2), 1000.1234)
    assert row == {'k': 'demnav', 'rx': 1000.123, 't': 12.5, 'lat': 37.9, 'lon': -122.6,
                   'alt': 800.0, 'status': 2, 'cov': [4.0, 0.5, 9.0]}


def test_truth_rows_from_odometry_and_pose():
    position = SimpleNamespace(x=1.0, y=2.0, z=3.0)
    header = SimpleNamespace(stamp=stamp(7.25))
    odometry = SimpleNamespace(header=header,
                               pose=SimpleNamespace(pose=SimpleNamespace(position=position)))
    pose = SimpleNamespace(header=header, pose=SimpleNamespace(position=position))
    expected = {'k': 'truth', 'rx': 5.0, 't': 7.25, 'e': 1.0, 'n': 2.0, 'u': 3.0}
    assert recorder.truth_row(odometry, 5.0) == expected
    assert recorder.truth_row(pose, 5.0) == expected


def test_thinning_follows_simulation_time_once_the_clock_is_known():
    thin = recorder.Thinner(0.2)
    # Before /clock: wall time.
    assert [thin.keep(None, rx) for rx in (0.0, 0.1, 0.2)] == [True, False, True]
    # Simulation at 0.2x: one row per 0.2 s of simulation = 1 s of wall time.
    kept = [thin.keep(round(0.04 * i, 2), 10.0 + 0.2 * i) for i in range(20)]
    assert sum(kept) == 4
    changes = recorder.ChangeFilter()
    assert [changes.changed(v) for v in ('a', 'a', 'b', 'a')] == [True, False, True, True]


# --- wrapper and control override ----------------------------------------------
WRAPPER = REPO / 'tools/gps_denied_sim_test.sh'


def test_wrapper_runs_each_stack_with_the_validation_and_denial_profiles():
    text = WRAPPER.read_text()
    subprocess.run(['bash', '-n', str(WRAPPER)], check=True)
    for stack, composition in STACKS.items():
        assert re.search(rf'{stack}\) composition=compositions/{composition};', text)
    # ... and no other stack: every one it accepts has a composition here.
    assert sorted(re.findall(r'^\s+(\w+)\) composition=', text, re.MULTILINE)) == sorted(STACKS)
    assert '--profile validation --profile denial' in text
    for variable in ('ROS_DOMAIN_ID', 'GPS_DENIED_PROJECT', 'GPS_DENIED_COMPOSE_OVERRIDE',
                     'GPS_DENIED_OUTPUT', 'GPS_DENIED_TIMEOUT_S'):
        assert variable in text
    assert 'tools/gps_denied_sim_scorer.py "$GPS_DENIED_OUTPUT" --stack "$stack"' in text
    assert 'trap teardown EXIT' in text
    result = subprocess.run(['bash', str(WRAPPER), 'lite'], capture_output=True, text=True,
                            check=False)
    assert result.returncode == 2 and 'usage' in result.stderr


def test_control_override_stops_demnav_and_wildnav_but_keeps_the_fusion():
    control = yaml.safe_load((REPO / 'tools/gps_denied_sim_control.yaml').read_text())
    overridden = control['services']
    assert set(overridden) == {'demnav', 'wildnav'}
    assert overridden['demnav']['command'] == 'sleep infinity'
    wildnav = overridden['wildnav']['command']
    assert 'navigation_fusion_node' in wildnav and 'wildnav_node' not in wildnav
    for stack in STACKS:
        launch = services(stack)['wildnav']['command']
        assert 'raw_odom_topic:=${RAW_ODOM_TOPIC:-/aerodrone/sensor_measurements/odom}' in launch
        assert 'navigation_output_topic:=${NAVIGATION_ODOM_TOPIC:-/navigation/odometry}' in launch
    assert '-p raw_odom_topic:=/aerodrone/sensor_measurements/odom' in wildnav
    assert '-p output_topic:=/navigation/odometry' in wildnav
