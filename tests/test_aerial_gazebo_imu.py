"""Gazebo aerodrone: the simulator IMU that drives ArduPilot SITL must not
share a topic with the platform's MAVROS IMU republish."""
import sys
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import pytest
from conftest import REPO, load_script


class Bridge:
    def __init__(self, gz_topic, ros_topic):
        self.gz_topic, self.ros_topic = gz_topic, ros_topic

    def remapping(self):
        return self.gz_topic, self.ros_topic

    def argument(self):
        return f'{self.gz_topic}@x'


@pytest.fixture
def bridges_module():
    drone = NS(model_name='aerodrone', bridges=lambda world: ([
        Bridge('/world/w/model/aerodrone/link/internal/sensor/imu/imu', 'sensor_measurements/imu'),
        Bridge('/model/aerodrone/pose', '/tf'),
        Bridge('/world/w/model/aerodrone/model/downward_rgb/link/camera', 'sensor_measurements/downward_rgb/image_raw'),
        Bridge('/gz/rgbd/camera_info', 'sensor_measurements/downward_rgbd/camera_info'),
        Bridge('/gz/rgbd/camera_info', 'sensor_measurements/downward_rgbd/depth/camera_info'),
    ], ['ground_truth_custom_bridge']))
    with patch.dict(sys.modules):
        for name in ('as2_gazebo_assets', 'as2_gazebo_assets.utils',
                     'as2_gazebo_assets.utils.launch_exception', 'as2_gazebo_assets.world',
                     'launch', 'launch.actions', 'launch.substitutions',
                     'launch_ros', 'launch_ros.actions'):
            sys.modules[name] = MagicMock()
        sys.modules['as2_gazebo_assets.world'].World = lambda **config: NS(
            drones=[drone], world_name='w')
        sys.modules['launch.substitutions'].LaunchConfiguration = lambda name: NS(
            perform=lambda context: {'namespace': 'aerodrone',
                                     'simulation_config_file': context}[name])
        sys.modules['launch_ros.actions'].Node = lambda **kwargs: NS(**kwargs)
        yield load_script(REPO / 'applications/gazebo/scripts/drone_bridges_isolated.py',
                          'drone_bridges_isolated_review')


def test_gazebo_imu_is_bridged_to_a_simulator_only_topic(bridges_module, tmp_path):
    config = tmp_path / 'world.yaml'
    config.write_text('world_name: w\n')
    nodes = bridges_module.drone_bridges(str(config))
    remappings = dict(nodes[0].remappings)
    assert remappings['/world/w/model/aerodrone/link/internal/sensor/imu/imu'] == 'ground_truth/imu'
    assert remappings['/model/aerodrone/pose'] == '/simulation/tf'
    assert 'sensor_measurements/imu' not in remappings.values()
    assert remappings['/world/w/model/aerodrone/model/downward_rgb/link/camera'] == \
        'sensor_measurements/downward_rgb/image_raw'


def test_backend_reads_the_simulator_imu_not_the_mavros_republish():
    backend = (REPO / 'applications/gazebo/scripts/gazebo_ardupilot_backend.py').read_text()
    assert '"GAZEBO_IMU_TOPIC", "/aerodrone/ground_truth/imu"' in backend
    topics = dict(
        line.split('=', 1) for line in
        (REPO / 'systems/aerodrone/config/topics.env').read_text().splitlines()
        if '=' in line and not line.startswith('#'))
    assert topics['GAZEBO_IMU_TOPIC'] == '/aerodrone/ground_truth/imu'
    assert topics['GAZEBO_IMU_TOPIC'] != topics['IMU_TOPIC']


def test_both_camera_info_topics_survive_jazzy_duplicate_source(bridges_module, tmp_path):
    config = tmp_path / 'world.yaml'
    config.write_text('world_name: w\n')
    nodes = [node for node in bridges_module.drone_bridges(str(config))
             if node.package == 'ros_gz_bridge']
    destinations = []
    for node in nodes:
        sources = [source for source, _ in node.remappings]
        assert len(sources) == len(set(sources))
        destinations.extend(target for source, target in node.remappings
                            if source == '/gz/rgbd/camera_info')
    assert set(destinations) == {'sensor_measurements/downward_rgbd/camera_info',
                                 'sensor_measurements/downward_rgbd/depth/camera_info'}
