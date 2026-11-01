"""Hardware compositions cap the open-file limit of their ROS 2 Jazzy containers.

Docker on some hosts (the UNO Q's Debian 13 Docker 26, or any host whose
containerd runs with LimitNOFILE=infinity) gives containers an open-file limit
of about 2^30. LTTng-UST, which every Jazzy process loads through tracetools,
sizes a descriptor bitmap by that limit and zeroes it: 128 MiB of RAM per
node. Every ROS image here is Jazzy, the aerostack2 family included since
the native Zenoh migration.
"""
import re

import pytest
import yaml

from conftest import REPO

# Images built on ROS 2 Jazzy (or Space ROS, which is Jazzy-based)
# Every ROS image here is Jazzy: the aerostack2 family (aerostack2 and the
# images built FROM it) moved from Humble with the native Zenoh migration
JAZZY_IMAGES = {'ros2', 'rangefinder', 'pantiltros', 'realsense', 'spaceros', 'racecarneo',
                'depthanything', 'nvblox', 'nicegui', 'so101', 'unoq', 'aerostack2', 'demnav',
                'wildnav', 'oak1', 'boat', 'submarine', 'navigator'}
IMAGE = re.compile(r'salsicha/cyberphysics:([a-z0-9_]+)-')
# The aircraft's BlueOS stacks and the tracker are included by aerodrone_hardware.yaml
HARDWARE = sorted([*REPO.glob('compositions/*_hardware.yaml'), *REPO.glob('compositions/*_uno_q.yaml'),
                   *REPO.glob('compositions/blueos_*.yaml'), REPO / 'compositions/aerodrone_tracking.yaml',
                   REPO / 'compositions/rangefinder.yaml'])


def jazzy_services(path):
    services = yaml.safe_load(path.read_text()).get('services') or {}
    for name, service in services.items():
        match = IMAGE.match(service.get('image', ''))
        if match and match.group(1) in JAZZY_IMAGES:
            yield name, service


def test_the_hardware_compositions_run_jazzy_services():
    assert sum(1 for path in HARDWARE for _ in jazzy_services(path)) >= 20


@pytest.mark.parametrize('path', HARDWARE, ids=lambda path: path.stem)
def test_jazzy_services_cap_the_open_file_limit(path):
    for name, service in jazzy_services(path):
        nofile = service.get('ulimits', {}).get('nofile')
        assert nofile, f'{path.name}: {name} has no ulimits.nofile'
        # The bitmap is hard-limit bits: 524288 is 64 KiB
        assert 1024 <= nofile['soft'] <= nofile['hard'] <= 1 << 20, f'{path.name}: {name}'


def test_standalone_rangefinder_starts_without_its_adapter():
    """Like the aircraft's: the host's /dev and USB serial cgroup rules,
    not `devices:`, which Docker refuses while the adapter is unplugged."""
    service = yaml.safe_load((REPO / 'compositions/rangefinder.yaml').read_text())['services']['rangefinder']
    assert 'devices' not in service
    assert '/dev:/host/dev:ro' in service['volumes']
    assert set(service['device_cgroup_rules']) == {'c 188:* rmw', 'c 166:* rmw'}
    assert '-p port:=/host${RANGEFINDER_DEVICE:-/dev/ttyUSB0}' in service['command']
