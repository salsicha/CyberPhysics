"""Gazebo aerodrone: the aerostack2 image carries the pinned Aerostack2 Gazebo
assets whose layout compositions/aerostack2_sim.yaml runs and bind-mounts
over, and asserts that layout at build time."""
import re

import yaml
from conftest import REPO

DOCKERFILE = REPO / 'applications/aerostack2/Dockerfile'
COMPOSE = REPO / 'compositions/aerostack2_sim.yaml'
SHARE = '/opt/aerostack2/share/as2_gazebo_assets/'
GAZEBO_SERVICES = ('gazebo', 'gazebo_bridges')


def dockerfile():
    return DOCKERFILE.read_text()


def services():
    return yaml.safe_load(COMPOSE.read_text())['services']


def bind_mounts():
    """(repo source, as2_gazebo_assets-relative target) of every bind mount
    over the installed package."""
    mounts = []
    for name in GAZEBO_SERVICES:
        for volume in services()[name]['volumes']:
            source, target = volume.split(':')[:2]
            if target.startswith(SHARE):
                mounts.append((source, target[len(SHARE):]))
    return mounts


def test_aerostack_source_is_pinned_on_jazzy_for_every_architecture():
    text = dockerfile()
    revision = re.search(r'^ARG AS2_REVISION=([0-9a-f]{40})$', text, re.MULTILINE)
    assert revision, 'Jazzy source builds must use an immutable upstream commit'
    assert 'FROM ros:jazzy-ros-base' in text
    assert 'rev-parse HEAD)' in text and '= "$AS2_REVISION"' in text
    assert '--install-base /opt/aerostack2' in text
    assert 'snapshots.ros.org/humble' not in text


def test_every_bind_mount_over_the_package_exists_in_repo_and_image():
    mounts = bind_mounts()
    targets = {target for _, target in mounts}
    assert {'launch/drone_bridges.py', 'models/quadrotor_base/quadrotor_base.sdf.jinja',
            'models/hd_camera/hd_camera.sdf.jinja',
            'models/rgbd_camera/rgbd_camera.sdf.jinja'} <= targets
    text = dockerfile()
    for source, target in mounts:
        assert (COMPOSE.parent / source).resolve().is_file(), source
        # Replacing a file the package does not have would only add a
        # template nothing renders: the image build checks each target.
        assert target in text, f'Dockerfile does not assert {target}'


def test_the_downward_cameras_use_the_cad_lens_without_a_generic_landing_box():
    """The old centred landing visual occluded depth. The real legs are
    allowed below the lens: CAD assembly tests check their field-of-view
    clearance. No generic quadrotor geometry may be layered over that model.
    """
    import json
    data = json.loads((REPO / 'applications/aerostack2/models/aerodrone/airframe.json').read_text())
    generated = (REPO / 'applications/demnav/scripts/build_georeferenced_terrain.py').read_text()
    fixture = (REPO / 'applications/gazebo/config/world_gazebo.yaml').read_text()
    assert "CAMERA_POSITION = AIRFRAME['camera_position_flu_m']" in generated
    assert generated.count('xyz: {CAMERA_POSITION}') == 2
    mounts = re.findall(r'model_type: (?:hd|rgbd)_camera\n.*\n\s*xyz: \[([^\]]+)\]', fixture)
    assert len(mounts) == 2
    for mount in mounts:
        assert [float(v) for v in mount.split(',')] == data['camera_position_flu_m']
    template = (REPO / 'applications/gazebo/models/quadrotor_ardupilot.sdf.jinja').read_text()
    assert 'model://aerodrone' in template
    assert 'model://quadrotor_base' not in template
    assert '<visual' not in template


def test_launch_files_the_stack_runs_are_checked_at_build_time():
    text = dockerfile()
    for name in GAZEBO_SERVICES:
        launch = re.search(r'ros2 launch as2_gazebo_assets (\S+)',
                           services()[name]['command']).group(1)
        assert f'launch/{launch}' in text
        assert f'ros2 launch as2_gazebo_assets {launch} --show-args' in text
