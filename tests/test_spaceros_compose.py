"""Space ROS image: the Curiosity demo is built in the image, not via docker."""
import re

import yaml

from conftest import REPO

APP = REPO / 'applications/spaceros'
DOCKERFILE = (APP / 'Dockerfile').read_text()
STACK = yaml.safe_load((REPO / 'compositions/spaceros.yaml').read_text())['services']


def test_demo_workspace_is_built_inside_the_image():
    # Upstream build.sh/run.sh only wrap `docker compose`, which is not
    # available inside a docker build
    instructions = '\n'.join(line for line in DOCKERFILE.splitlines()
                             if not line.lstrip().startswith('#'))
    for wrapper in ('build.sh', 'run.sh', 'docker '):
        assert wrapper not in instructions, wrapper
    release = re.search(r'^ARG SPACEROS_RELEASE=(\S+)$', DOCKERFILE, re.MULTILINE)
    assert release and release.group(1).startswith('jazzy-')
    assert 'FROM osrf/space-ros:${SPACEROS_RELEASE}' in DOCKERFILE
    assert '--branch "${SPACEROS_RELEASE}"' in DOCKERFILE
    assert 'rosdep install --from-paths' in DOCKERFILE
    assert 'colcon build' in DOCKERFILE
    # Core and generated message dispatchers must be rebuilt together.
    assert 'RMW_IMPLEMENTATION=rmw_zenoh_cpp' in DOCKERFILE
    assert 'RUN bash /tmp/enable_zenoh.sh' in DOCKERFILE


def test_compose_and_cmd_run_the_launch_file_shipped_in_the_image():
    assert 'COPY launch/ /launch/' in DOCKERFILE
    assert '"/launch/curiosity_sim.launch.py"' in DOCKERFILE
    assert (APP / 'launch/curiosity_sim.launch.py').is_file()
    service = STACK['spaceros']
    assert service['image'].startswith('salsicha/cyberphysics:spaceros-')
    command = service['command'].split()
    assert command[:3] == ['ros2', 'launch', '/launch/curiosity_sim.launch.py']
    assert 'gui:=${SPACEROS_GUI:-true}' in command


def test_launch_file_exposes_headless_and_demo_switches():
    launch = (APP / 'launch/curiosity_sim.launch.py').read_text()
    declared = re.findall(r'DeclareLaunchArgument\(\s*"(\w+)"', launch)
    assert declared == ['gui', 'demo', 'gz_verbosity']
    # Server-only Gazebo still renders the camera and gpu_lidar sensors
    assert '-s --headless-rendering' in launch
    assert 'mars_rover.launch.py' in launch
