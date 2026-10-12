"""The tracking detector's wiring: the OAK-1 service of the aircraft (on by
default, off for the airplane) and the simulators' tracking_detector
(profile `tracking`), checked on the rendered stacks."""
import json
import os
import shutil
import subprocess
from functools import cache

import pytest

from conftest import REPO

COMPOSITIONS = REPO / 'compositions'
SIMULATORS = ('aerostack2_sim.yaml', 'aerostack2_genesis.yaml', 'aerostack2_isaac.yaml')


def env_file(path):
    values = {}
    for line in (REPO / path).read_text().splitlines():
        if line and not line.startswith('#') and '=' in line:
            key, _, value = line.partition('=')
            values[key.strip()] = value.strip()
    return values


TOPICS = env_file('systems/aerodrone/config/topics.env')


def compose_available():
    return bool(shutil.which('docker')) and subprocess.run(
        ['docker', 'compose', 'version'], capture_output=True).returncode == 0


needs_compose = pytest.mark.skipif(not compose_available(),
                                   reason='Docker Compose is not installed')


@cache
def rendered(name, profiles=(), extra=()):
    env = {k: v for k, v in os.environ.items()
           if not k.startswith(('TRACKING_', 'OAK1_', 'COMPOSE_', 'WILDNAV_'))
           and k not in ('CYBERPHYSICS_TAG', 'CAMERA_TOPIC_ROOT')}
    env.update(dict(extra))
    args = ['docker', 'compose', '-f', str(COMPOSITIONS / name)]
    for profile in profiles:
        args += ['--profile', profile]
    out = subprocess.run(args + ['config', '--format', 'json'], env=env,
                         check=True, capture_output=True, text=True)
    return json.loads(out.stdout)['services']


def ros_args(service):
    command = service['command']
    return dict(arg.split(':=', 1) for arg in command if ':=' in arg)


def test_the_configured_model_ships_with_the_oak1_image():
    model = TOPICS['TRACKING_DETECTOR_MODEL']
    meta = json.loads((REPO / f'applications/oak1/models/{model}.json').read_text())
    for name in (meta['blob'], meta['onnx']):
        assert (REPO / 'applications/oak1/models' / name).is_file(), name
    assert (meta['input_width'], meta['input_height']) == (640, 352)
    assert TOPICS['TRACKING_TOPIC_PREFIX'] == '/tracking'


@needs_compose
def test_aircraft_oak1_runs_megadepth_and_the_detector_by_default():
    args = ros_args(rendered('aerodrone_hardware.yaml')['oak1'])
    # MegaDepth and the stills keep their topics (DemNav, WildNav).
    assert args['depth_topic'] == '/oak1/relative_depth'
    assert args['camera_info_topic'] == '/oak1/camera_info'
    assert args['highres_image_topic'] == '/oak1/image_highres'
    assert args['blob_path'] == '/models/megadepth_192x256_openvino_2021.4_6shave.blob'
    assert 'megadepth_threads' not in args and 'megadepth_rate_hz' not in args
    assert args['tracking_enabled'] == 'true'
    assert args['tracking_topic_prefix'] == TOPICS['TRACKING_TOPIC_PREFIX']
    assert args['tracking_model'] == f"/models/{TOPICS['TRACKING_DETECTOR_MODEL']}.json"
    assert float(args['tracking_rate_hz']) == 5.0


@needs_compose
def test_aircraft_tracking_can_be_turned_off_and_the_airplane_has_it_off():
    args = ros_args(rendered('aerodrone_hardware.yaml',
                             extra=(('OAK1_TRACKING', 'false'),))['oak1'])
    assert args['tracking_enabled'] == 'false'
    assert ros_args(rendered('airplane_hardware.yaml')['oak1'])['tracking_enabled'] == 'false'


@needs_compose
@pytest.mark.parametrize('name', SIMULATORS)
def test_simulators_run_the_detector_with_the_tracking_profile(name):
    assert 'tracking_detector' not in rendered(name)
    assert 'tracking_detector' not in rendered(name, ('tracking_truth',))
    detector = rendered(name, ('tracking',))['tracking_detector']
    assert detector['image'] == 'salsicha/cyberphysics:oak1-latest'
    assert detector['command'][:2] == ['python3', '/scripts/sim_tracking_detector.py']
    args = ros_args(detector)
    assert args['image_topic'] == TOPICS['WILDNAV_IMAGE_TOPIC']
    assert args['camera_info_topic'] == TOPICS['WILDNAV_CAMERA_INFO_TOPIC']
    assert args['topic_prefix'] == TOPICS['TRACKING_TOPIC_PREFIX']
    assert args['model'] == f"/models/{TOPICS['TRACKING_DETECTOR_MODEL']}.json"
    scripts = next(v for v in detector['volumes'] if v['target'] == '/scripts')
    assert scripts['source'] == str(REPO / 'applications/oak1/scripts')
    assert scripts['read_only']
