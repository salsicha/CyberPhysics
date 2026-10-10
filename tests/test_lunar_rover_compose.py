"""Lunar rover stacks: every script and parameter file they run exists."""
import yaml

from conftest import REPO

SIM = yaml.safe_load(
    (REPO / 'compositions/lunar_rover_sim.yaml').read_text())['services']
HARDWARE = yaml.safe_load(
    (REPO / 'compositions/lunar_rover_hardware.yaml').read_text())['services']
CONFIG_MOUNT = '../systems/lunar_rover/config:/workspace/config:ro'


def test_sim_nodes_run_shipped_scripts_with_matching_params():
    assert set(SIM) == {'lunar_rover_sim', 'lunar_waypoint_nav'}
    for name, service in SIM.items():
        assert service['image'].startswith('salsicha/cyberphysics:spaceros-')
        assert CONFIG_MOUNT in service['volumes'], name
        command = service['command'].split()
        assert command[:2] == ['python3', f'/scripts/{name}.py'], name
        assert (REPO / 'applications/spaceros/scripts' / f'{name}.py').is_file()
        params = command[command.index('--params-file') + 1]
        assert params.startswith('/workspace/config/'), name
        host = REPO / 'systems/lunar_rover/config' / params.rsplit('/', 1)[1]
        # The node reads its own section of the parameter file
        assert name in yaml.safe_load(host.read_text()), name


def test_all_lunar_services_share_one_ros_domain_and_dds():
    for name, service in {**SIM, **HARDWARE}.items():
        assert 'ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}' in service['environment'], name
        # Space ROS ships only Cyclone DDS; Fast DDS settings would be inert
        assert not any(env.startswith('FASTDDS_')
                       for env in service['environment']), name
