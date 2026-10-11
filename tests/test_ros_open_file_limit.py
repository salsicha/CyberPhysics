"""Hardware compositions cap the open-file limit of their ROS 2 Jazzy containers.

Docker on some hosts (the UNO Q's Debian 13 Docker 26, or any host whose
containerd runs with LimitNOFILE=infinity) gives containers an open-file limit
of about 2^30. LTTng-UST, which every Jazzy process loads through tracetools,
sizes a descriptor bitmap by that limit and zeroes it: 128 MiB of RAM per
node. The Humble images (aerostack2 and those built on it) do not link
LTTng-UST.
"""
import re

import pytest
import yaml

from conftest import REPO

# Images built on ROS 2 Jazzy (or Space ROS, which is Jazzy-based)
JAZZY_IMAGES = {'ros2', 'rangefinder', 'pantiltros', 'realsense', 'spaceros', 'racecarneo',
                'depthanything', 'nvblox', 'nicegui', 'so101', 'unoq'}
IMAGE = re.compile(r'salsicha/cyberphysics:([a-z0-9_]+)-')
HARDWARE = sorted([*REPO.glob('compositions/*_hardware.yaml'), *REPO.glob('compositions/*_uno_q.yaml')])


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
