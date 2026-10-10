"""Aerial stacks: every ROS container loads the shared FastDDS profile whose
shared-memory segments and UDP socket buffers hold the camera frames
(applications/aerostack2/README.md, "DDS transport")."""
import posixpath
import xml.etree.ElementTree as ET

import pytest
import yaml
from conftest import REPO

STACKS = ['compositions/aerostack2_genesis.yaml', 'compositions/aerostack2_isaac.yaml',
          'compositions/aerostack2_sim.yaml', 'tools/gps_denied_sitl.yaml']
CONFIG = 'systems/aerodrone/config'
SHM_PROFILE = f'/workspace/{CONFIG}/fastdds.xml'
UDP_PROFILE = f'/workspace/{CONFIG}/fastdds_udp.xml'
# Isaac Sim runs as the image's user 1234, every other container as root:
# FastDDS shared memory between them delivers nothing, so it uses UDP only.
UDP_ONLY = {('compositions/aerostack2_isaac.yaml', 'isaacsim')}
# Largest simulated camera burst: 640x480 rgb8 plus 640x480 32FC1 depth.
CAMERA_BURST_BYTES = 640 * 480 * 3 + 640 * 480 * 4
NS = {'p': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}


def ros_services(stack):
    path = REPO / stack
    services = yaml.safe_load(path.read_text())['services']
    return path, {name: service for name, service in services.items()
                  if 'ROS_DOMAIN_ID' in (service.get('environment') or {})}


def mounts(path, service):
    """{container path: host path} of the service's bind mounts."""
    result = {}
    for volume in service.get('volumes') or []:
        source, target = volume.split(':')[:2]
        if source.startswith('.'):
            result[target] = (path.parent / source).resolve()
    return result


def mounted_file(path, service, container_path):
    """Host file a container path resolves to through the bind mounts."""
    for target, source in mounts(path, service).items():
        if container_path == target:
            return source
        if container_path.startswith(target.rstrip('/') + '/'):
            return source / posixpath.relpath(container_path, target)
    return None


def transports(profile):
    root = ET.parse(REPO / CONFIG / profile).getroot()
    participant = root.find("p:participant[@is_default_profile='true']", NS)
    assert participant.findtext('p:rtps/p:useBuiltinTransports', namespaces=NS) == 'false'
    used = [element.text for element in
            participant.findall('p:rtps/p:userTransports/p:transport_id', NS)]
    descriptors = {element.findtext('p:transport_id', namespaces=NS): element for element
                   in root.findall('p:transport_descriptors/p:transport_descriptor', NS)}
    return {descriptors[name].findtext('p:type', namespaces=NS): descriptors[name]
            for name in used}


@pytest.mark.parametrize('stack', STACKS)
def test_every_ros_container_loads_the_mounted_aerodrone_profile(stack):
    path, services = ros_services(stack)
    assert len(services) >= 8
    for name, service in services.items():
        environment = service['environment']
        expected = UDP_PROFILE if (stack, name) in UDP_ONLY else SHM_PROFILE
        assert environment.get('FASTRTPS_DEFAULT_PROFILES_FILE') == expected, (stack, name)
        # Fast DDS 2.14 (Isaac Sim with ISAAC_ROS_DISTRO=jazzy) also reads the newer name.
        assert environment.get('FASTDDS_DEFAULT_PROFILES_FILE', expected) == expected
        assert 'FASTDDS_BUILTIN_TRANSPORTS' not in environment, (stack, name)
        assert mounted_file(path, service, expected) == REPO / expected[len('/workspace/'):], (
            stack, name)


@pytest.mark.parametrize('stack', STACKS)
def test_shared_memory_containers_share_the_host_dev_shm_as_root(stack):
    _, services = ros_services(stack)
    for name, service in services.items():
        if (stack, name) in UDP_ONLY:
            continue
        volumes = service.get('volumes') or []
        assert service.get('ipc') == 'host' or '/dev/shm:/dev/shm' in volumes, (stack, name)
        assert service.get('user', '0:0') in ('0:0', 'root'), (stack, name)


def test_isaac_sim_is_the_only_udp_only_container_and_runs_as_another_user():
    _, services = ros_services('compositions/aerostack2_isaac.yaml')
    assert 'user' not in services['isaacsim']  # the image's isaac-sim user
    for stack in STACKS:
        _, services = ros_services(stack)
        for name, service in services.items():
            if service['environment']['FASTRTPS_DEFAULT_PROFILES_FILE'] == UDP_PROFILE:
                assert (stack, name) in UDP_ONLY


def test_shared_memory_profile_holds_camera_bursts():
    """FastDDS's default 512 KiB segment holds less than one frame."""
    used = transports('fastdds.xml')
    assert list(used) == ['SHM', 'UDPv4']
    segment = int(used['SHM'].findtext('p:segment_size', namespaces=NS))
    assert segment >= 3 * CAMERA_BURST_BYTES
    for field in ('sendBufferSize', 'receiveBufferSize'):
        assert int(used['UDPv4'].findtext(f'p:{field}', namespaces=NS)) >= CAMERA_BURST_BYTES


def test_udp_profile_has_the_same_socket_buffers_and_no_shared_memory():
    shm = transports('fastdds.xml')['UDPv4']
    used = transports('fastdds_udp.xml')
    assert list(used) == ['UDPv4']
    for field in ('sendBufferSize', 'receiveBufferSize'):
        assert (used['UDPv4'].findtext(f'p:{field}', namespaces=NS) ==
                shm.findtext(f'p:{field}', namespaces=NS))


def test_racecar_ros_containers_use_udp_with_the_same_socket_buffers():
    """The RACECAR containers do not share /dev/shm: UDP only, with the
    buffers of the aerial profiles instead of the 208 KiB default."""
    size = transports('fastdds.xml')['UDPv4'].findtext('p:receiveBufferSize', namespaces=NS)
    services = yaml.safe_load((REPO / 'compositions/racecarneo.yaml').read_text())['services']
    values = {name: entry.split('=', 1)[1] for name, service in services.items()
              for entry in service.get('environment') or []
              if entry.startswith('FASTDDS_BUILTIN_TRANSPORTS=')}
    assert len(values) >= 10
    assert set(values.values()) == {f'UDPv4?sockets_size={size}'}
