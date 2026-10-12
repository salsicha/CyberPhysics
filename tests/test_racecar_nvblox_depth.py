"""What nvblox maps (applications/depthanything/README.md, "Accuracy" and
"On the car"; applications/isaac/RACECAR_TURRET.md, "Depth for nvblox"):
in the simulators the simulated camera's own depth rather than Depth
Anything's, the RacecarSim camera mount measured from that depth, and the
Isaac arena's dynamic mapping of the walking people; on the car the
RealSense's aligned depth in dynamic mode (nvblox_hardware.yaml), with
Depth Anything only under its `depthanything` profile."""
import math
import re
import shlex
import xml.etree.ElementTree as ET

import pytest
import yaml

from conftest import REPO

LAUNCH = REPO / 'applications/nvblox/racecarneo_nvblox.launch.py'
CONFIG = REPO / 'systems/racecarneo/config'
TOPICS = yaml.safe_load((CONFIG / 'topics.yaml').read_text())['topics']
HARDWARE = ('racecarneo_hardware.yaml', 'racecar_turret_hardware.yaml')


def services(composition):
    return yaml.safe_load((REPO / 'compositions' / composition).read_text())['services']


def command(composition, service):
    text = services(composition)[service].get('command', '')
    return ' '.join(text) if isinstance(text, list) else text


def argument(composition, service, name):
    """A launch argument of a service's command, its ${VAR:-default}
    resolved to the default, or the launch file's default."""
    match = re.search(rf'{name}:=(\S+)', command(composition, service))
    if match is None:
        return re.search(rf"\('{name}', '([^']+)'", LAUNCH.read_text()).group(1)
    return re.sub(r'\$\{\w+:-([^}]+)\}', r'\1', match.group(1))


def test_simulators_map_the_cameras_depth():
    assert argument('racecarneo.yaml', 'racecarneo_nvblox', 'depth_image_topic') == \
        '/camera/depth/image_rect_raw'
    assert argument('racecar_turret_isaac.yaml', 'racecar_turret_nvblox', 'depth_image_topic') == \
        '/camera/depth/image_rect_raw'
    # Depth Anything still runs in the simulators, publishing as on the car.
    for composition, service in (('racecarneo.yaml', 'racecarneo_depthanything'),
                                 ('racecar_turret_isaac.yaml', 'racecar_turret_depthanything')):
        assert '/depth_anything/depth/image' in command(composition, service)
    assert 'racecarneo_depthanything' not in services('racecarneo.yaml')['racecarneo_nvblox']['depends_on']


def test_the_nvblox_launch_takes_an_overrides_file_that_defaults_to_nothing_new():
    launch = LAUNCH.read_text()
    assert "'overrides_file'" in launch and 'default_value=config_file' in launch
    assert "LaunchConfiguration('overrides_file')" in launch.split('parameters=[')[1].split(']')[0]
    # RacecarSim maps with the car's static nvblox.yaml alone.
    assert 'overrides_file' not in command('racecarneo.yaml', 'racecarneo_nvblox')


# --- On the car -------------------------------------------------------------

def launch_arguments(composition, service):
    return dict(arg.split(':=', 1) for arg in shlex.split(command(composition, service)) if ':=' in arg)


def test_the_car_maps_the_realsenses_aligned_depth():
    """nvblox reads the RealSense's depth aligned to the colour camera with
    that image's own camera_info (one frame, one stamp: nvblox syncs them
    exactly); the topics are realsense2_camera's for the camera's
    namespace and name."""
    camera = launch_arguments('racecarneo_hardware.yaml', 'racecarneo_camera')
    assert camera['align_depth.enable'] == 'true' and camera['enable_depth'] == 'true'
    node = '/' + '/'.join(part.strip('/') for part in (camera['camera_namespace'], camera['camera_name'])
                          if part.strip('/'))
    assert node + '/aligned_depth_to_color/image_raw' == TOPICS['aligned_depth_image']
    assert node + '/aligned_depth_to_color/camera_info' == TOPICS['aligned_depth_camera_info']
    assert argument('racecarneo_hardware.yaml', 'racecarneo_nvblox', 'depth_image_topic') == \
        TOPICS['aligned_depth_image']
    assert argument('racecarneo_hardware.yaml', 'racecarneo_nvblox', 'depth_camera_info_topic') == \
        TOPICS['aligned_depth_camera_info']
    # nvblox does not wait for Depth Anything.
    assert services('racecarneo_hardware.yaml')['racecarneo_nvblox']['depends_on'] == ['racecarneo_camera']
    assert '/depth_anything' not in re.sub(r'\$\{\w+:-([^}]+)\}', '',
                                           command('racecarneo_hardware.yaml', 'racecarneo_nvblox'))


def test_the_realsense_streams_at_the_rate_nvblox_was_evaluated_at():
    """Aligned depth takes the colour camera's size: 640x480, as Isaac's
    camera (fx 617 px); 15 Hz as Isaac's camera, half the driver's 30."""
    camera = launch_arguments('racecarneo_hardware.yaml', 'racecarneo_camera')
    profile = {name: tuple(int(v) for v in re.sub(r'\$\{\w+:-([^}]+)\}', r'\1', camera[key]).split('x'))
               for name, key in (('color', 'rgb_camera.color_profile'),
                                 ('depth', 'depth_module.depth_profile'))}
    assert profile['color'] == (640, 480, 15)
    assert profile['depth'] == (848, 480, 15)
    # Alignment pairs one depth with one colour frame.
    assert profile['color'][2] == profile['depth'][2]


def test_the_car_maps_in_dynamic_mode_over_its_static_config():
    assert argument('racecarneo_hardware.yaml', 'racecarneo_nvblox', 'config_file') == \
        '/workspace/config/nvblox.yaml'
    assert argument('racecarneo_hardware.yaml', 'racecarneo_nvblox', 'overrides_file') == \
        '/workspace/config/nvblox_hardware.yaml'
    assert any(v.startswith('../systems/racecarneo/config:/workspace/config')
               for v in services('racecarneo_hardware.yaml')['racecarneo_nvblox']['volumes'])
    # Nav2 reads nav2.yaml as it is: the static slice, people from the lidar.
    nav2 = command('racecarneo_hardware.yaml', 'racecarneo_nav2')
    assert 'params_file:=/workspace/config/nav2.yaml' in nav2 and 'sed' not in nav2
    slices = re.findall(r'nvblox_map_slice_topic: (\S+)', (CONFIG / 'nav2.yaml').read_text())
    assert slices == [TOPICS['nvblox_static_slice']] * 2


def test_the_hardware_overrides_are_isaacs_dynamic_mode_within_the_realsenses_range():
    """nvblox_hardware.yaml sets what nvblox_isaac.yaml sets, with the same
    values except the integration range: a stereo camera's error grows
    with the square of the distance, and from 0.22 m a floor point's depth
    must be within about 9 % to stay out of the slice."""
    def flat(path):
        params = yaml.safe_load(path.read_text())['/**']['ros__parameters']
        out = {}
        for key, value in params.items():
            if isinstance(value, dict):
                out.update({f'{key}.{k}': v for k, v in value.items()})
            else:
                out[key] = value
        return out
    hardware = flat(CONFIG / 'nvblox_hardware.yaml')
    isaac = flat(REPO / 'systems/racecar_turret/config/nvblox_isaac.yaml')
    base = flat(CONFIG / 'nvblox.yaml')
    assert hardware['mapping_type'] == 'dynamic' and base['mapping_type'] == 'static_tsdf'
    ranged = {'static_mapper.projective_integrator_max_integration_distance_m'}
    assert set(hardware) == set(isaac) | ranged
    assert {k: v for k, v in hardware.items() if k not in ranged} == isaac
    assert base['static_mapper.projective_integrator_max_integration_distance_m'] == 12.0
    assert hardware['static_mapper.projective_integrator_max_integration_distance_m'] == \
        hardware['dynamic_mapper.projective_integrator_max_integration_distance_m'] == 5.0
    # The dynamic slice is the static one (not nvblox_sim.yaml's 0.09-0.65 m).
    for key in ('esdf_slice_height', 'esdf_slice_min_height', 'esdf_slice_max_height'):
        assert hardware[f'dynamic_mapper.{key}'] == base[f'static_mapper.{key}'], key
    assert 0.0 < hardware['static_mapper.tsdf_decay_factor'] < 0.95


@pytest.mark.parametrize('composition', HARDWARE)
def test_depth_anything_runs_only_under_its_profile(composition):
    stack = services(composition)
    assert stack['racecarneo_depthanything'].get('profiles') == ['depthanything']
    for name, service in stack.items():
        if name == 'racecarneo_depthanything':
            continue
        depends = service.get('depends_on', [])
        assert 'racecarneo_depthanything' not in depends, name
        assert not service.get('profiles'), name
    # Still the checkpoint's own scale, for comparison runs.
    if composition == 'racecarneo_hardware.yaml':
        depth = command(composition, 'racecarneo_depthanything')
        assert 'depth_topic:=/depth_anything/depth/image' in depth
        assert 'metric_dataset:=vkitti' in depth and 'max_depth:=80.0' in depth


def test_the_turret_runs_the_cars_nvblox_and_camera_unchanged():
    stack = services('racecar_turret_hardware.yaml')
    for name in ('racecarneo_nvblox', 'racecarneo_camera', 'racecarneo_nav2'):
        assert stack[name]['extends'] == {'file': 'racecarneo_hardware.yaml', 'service': name}
        assert set(stack[name]) == {'extends', 'environment'}, name


def test_the_isaac_arena_maps_people_as_dynamic_and_nav2_reads_the_static_slice():
    overrides = yaml.safe_load((REPO / 'systems/racecar_turret/config/nvblox_isaac.yaml').read_text())
    params = overrides['/**']['ros__parameters']
    assert params['mapping_type'] == 'dynamic'
    assert 0.0 < params['static_mapper']['tsdf_decay_factor'] < 0.95
    base = yaml.safe_load((REPO / 'systems/racecarneo/config/nvblox.yaml').read_text())
    static = base['/**']['ros__parameters']['static_mapper']
    for key in ('esdf_slice_height', 'esdf_slice_min_height', 'esdf_slice_max_height'):
        assert params['dynamic_mapper'][key] == static[key], key
    assert base['/**']['ros__parameters']['mapping_type'] == 'static_tsdf'  # the car's
    nvblox = command('racecar_turret_isaac.yaml', 'racecar_turret_nvblox')
    assert argument('racecar_turret_isaac.yaml', 'racecar_turret_nvblox',
                    'overrides_file') == '/workspace/nvblox_isaac.yaml'
    assert any(v.endswith(':/workspace/nvblox_isaac.yaml:ro')
               for v in services('racecar_turret_isaac.yaml')['racecar_turret_nvblox']['volumes'])
    assert 'use_sim_time: true' in nvblox
    nav2 = command('racecar_turret_isaac.yaml', 'racecar_turret_nav2')
    # Nav2 and the map recorder read the same slice, by default nav2.yaml's
    # static one (the people come from the lidar).
    assert 's#/nvblox_node/static_map_slice#${RACECAR_TURRET_NVBLOX_SLICE:-/nvblox_node/static_map_slice}#' in nav2
    recorder = command('racecar_turret_isaac.yaml', 'racecar_turret_map_recorder')
    assert '--slice-topic ${RACECAR_TURRET_NVBLOX_SLICE:-/nvblox_node/static_map_slice}' in recorder
    assert '/nvblox_node/static_map_slice' in (REPO / 'systems/racecarneo/config/nav2.yaml').read_text()


def test_racecarsim_camera_mount_is_the_measured_one():
    """RacecarSim's depth puts the floor 0.120 m below its camera, pitched
    0.81 degrees up; the prefab's level 0.145 m lifted the near floor into
    nvblox's 0.02 m obstacle band."""
    root = ET.parse(REPO / 'systems/racecarneo/urdf/racecarneo.urdf.xacro').getroot()
    xacro = '{http://www.ros.org/wiki/xacro}'
    sim = next(b for b in root.findall(xacro + 'if') if b.get('value') == '$(arg sim_geometry)')
    origin = next(j for j in sim.findall('joint') if j.get('name') == 'camera_joint').find('origin')
    x, y, z = (float(v) for v in origin.get('xyz').split())
    roll, pitch, yaw = (float(v) for v in origin.get('rpy').split())
    assert (x, y, z) == (0.10, 0.0, 0.120)
    # Negative pitch about y: the optical axis tilts up (ROS rpy).
    assert math.degrees(pitch) == pytest.approx(-0.81, abs=0.01) and roll == yaw == 0.0


def test_the_cars_depth_light_follows_the_realsense_depth_nvblox_reads():
    # The page's depth light would stay red on the car if it watched Depth
    # Anything, which runs there only under its profile
    import yaml
    for name in ('racecarneo_hardware.yaml', 'racecar_turret_hardware.yaml'):
        text = (REPO / 'compositions' / name).read_text()
        if name == 'racecarneo_hardware.yaml':
            env = yaml.safe_load(text)['services']['racecarneo_nicegui']['environment']
            assert ('UI_DEPTH_TOPIC=${UI_DEPTH_TOPIC:-/camera/aligned_depth_to_color/camera_info}'
                    in env)
    main = (REPO / 'applications/nicegui/scripts/main.py').read_text()
    assert "DEPTH_TOPIC = os.environ.get('UI_DEPTH_TOPIC', '/depth_anything/depth/image')" in main
    assert "CameraInfo if DEPTH_TOPIC.endswith('camera_info') else Image" in main
