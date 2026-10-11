"""Gazebo aerodrone: the aerostack2 image carries the pinned Aerostack2 Gazebo
assets whose layout compositions/aerostack2_sim.yaml runs and bind-mounts
over, and asserts that layout at build time."""
import re

import yaml
from conftest import REPO

DOCKERFILE = REPO / 'applications/aerostack2/Dockerfile'
COMPOSE = REPO / 'compositions/aerostack2_sim.yaml'
SHARE = '/opt/ros/humble/share/as2_gazebo_assets/'
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


def test_gazebo_assets_are_pinned_like_mavros_on_every_architecture():
    text = dockerfile()
    version = re.search(r'^ARG AS2_GAZEBO_ASSETS_VERSION=(\S+)$', text, re.MULTILINE)
    assert version, 'as2-gazebo-assets needs a pinned version'
    # The Aerostack2 release of the other as2 packages. Upstream version only:
    # the build date in the Debian version differs between amd64 and arm64.
    assert version.group(1) == '1.1.3'
    assert '"Package: ros-${ROS_DISTRO}-as2-gazebo-assets"' in text
    assert '"Pin: version ${AS2_GAZEBO_ASSETS_VERSION}-1jammy.*"' in text
    assert 'http://snapshots.ros.org/humble/${ROS_SNAPSHOT}/ubuntu' in text
    # No architecture-specific Debian version is pinned anywhere
    assert not re.search(r'=\d[\w.~+-]*-1jammy\.\d{8}', text)
    # The build checks each pin landed
    assert 'as2-gazebo-assets:${AS2_GAZEBO_ASSETS_VERSION}' in text
    assert 'mavros:${MAVROS_VERSION}' in text


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


def camera_mount_heights():
    """z (m, relative to base_link) of every downward camera payload in the
    generated world and the development fixture."""
    heights = []
    generated = (REPO / 'applications/demnav/scripts/build_georeferenced_terrain.py').read_text()
    fixture = (REPO / 'applications/gazebo/config/world_gazebo.yaml').read_text()
    for text in (generated, fixture):
        for match in re.finditer(r'model_type: (?:hd|rgbd)_camera\n.*\n\s*xyz: \[([^\]]+)\]', text):
            heights.append(float(match.group(1).split(',')[2]))
    assert len(heights) == 4
    return heights


def test_nothing_rendered_hangs_below_the_downward_cameras():
    """Gazebo's depth camera reports geometry inside its near clip as -inf:
    a visual under the lenses blanks every depth frame DemNav matches."""
    template = (REPO / 'applications/gazebo/models/quadrotor_ardupilot.sdf.jinja').read_text()
    lowest_camera = min(camera_mount_heights())
    links = re.findall(r'<link name="([^"]+)">(.*?)</link>', template, re.DOTALL)
    assert links
    for name, body in links:
        pose = re.search(r'<pose relative_to="base_link">([^<]+)</pose>', body)
        z = float(pose.group(1).split()[2]) if pose else 0.0
        if z < lowest_camera:
            assert '<visual' not in body, name
            assert '<collision' in body, name


def test_launch_files_the_stack_runs_are_checked_at_build_time():
    text = dockerfile()
    for name in GAZEBO_SERVICES:
        launch = re.search(r'ros2 launch as2_gazebo_assets (\S+)',
                           services()[name]['command']).group(1)
        assert f'launch/{launch}' in text
        assert f'ros2 launch as2_gazebo_assets {launch} --show-args' in text
