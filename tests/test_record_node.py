"""Record node: status calls must not hang, no per-call ptp4l, opt-in /shutdown.

Runs record.py's real methods with ROS stubbed out; no subprocess is allowed to
source the container entrypoint (which starts ptp4l in the background).
"""
import json
import re
import sys
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import pytest

from conftest import REPO, install_ros_stubs, load_script

RECORD = REPO / 'applications/record'


class FakeNode:
    def __init__(self, name):
        self.services = {}
        self.graph = []

    def create_service(self, srv_type, name, callback):
        self.services[name] = callback

    def create_subscription(self, *args):
        pass

    def create_publisher(self, *args):
        return MagicMock()

    def create_timer(self, *args):
        pass

    def get_logger(self):
        return MagicMock()

    def get_name(self):
        return 'python_backend'

    def get_node_names_and_namespaces(self):
        return self.graph


@pytest.fixture(scope='module')
def record():
    with patch.dict(sys.modules):
        install_ros_stubs()
        for name in ('rclpy.executors', 'std_srvs', 'std_srvs.srv'):
            sys.modules[name] = MagicMock()
        sys.modules['rclpy.node'].Node = FakeNode
        yield load_script(RECORD / 'scripts/record.py', 'record_review')


def make(record, **kwargs):
    return record.RosbagRecord('proj', '10.0.0.1', '10.0.0.2', **kwargs)


def test_shutdown_service_is_opt_in(record):
    assert '/shutdown' not in make(record).services
    assert '/shutdown' in make(record, enable_shutdown_service=True).services
    assert not record.parse_args([]).enable_shutdown_service
    assert record.parse_args(['--enable_shutdown_service']).enable_shutdown_service
    with patch.dict('os.environ', {'RECORD_ENABLE_SHUTDOWN_SERVICE': 'true'}):
        assert record.parse_args([]).enable_shutdown_service


def test_status_uses_ros_graph_without_subprocess(record):
    node = make(record)
    response = NS(success=False, message='')
    with patch.object(record.subprocess, 'Popen', side_effect=AssertionError('no subprocess')):
        node.backend_status_response(None, response)
        assert response.success and response.message.startswith('recording:false,')
        node.graph = [('python_backend', '/'), ('rosbag2_recorder', '/')]
        node.backend_status_response(None, response)
        assert response.message.startswith('recording:true,')


def test_recording_commands_do_not_source_entrypoint(record):
    node = make(record)
    node.flight_number = '7'
    node.save_config_files = MagicMock()
    commands = []

    def fake_popen(command, **kwargs):
        commands.append(command)
        return MagicMock(poll=MagicMock(return_value=None))

    with patch.object(record.subprocess, 'Popen', side_effect=fake_popen), \
            patch.object(record.os, 'makedirs'), patch.object(record.os, 'system'):
        response = node.start_recording(None, NS(success=False, message=''))
    assert response.success
    bag_commands = [c for c in commands if isinstance(c, str) and 'ros2 bag record' in c]
    assert len(bag_commands) == 2
    assert not any('entrypoint' in c or 'source' in c for c in bag_commands)


def test_dockerfile_cmd_is_accepted_by_the_parser(record):
    dockerfile = (RECORD / 'Dockerfile').read_text()
    cmd = json.loads(re.search(r'^CMD (\[.*\])$', dockerfile, re.M).group(1))
    assert cmd[:2] == ['python3', 'record.py']
    assert record.parse_args(cmd[2:]).proj_name == 'example_system'


def test_entrypoint_and_node_agree_on_env_list(record):
    entrypoint = (RECORD / 'entrypoint.sh').read_text()
    assert 'ENV_LIST="${ENV_LIST:-/env/env.list}"' in entrypoint
    assert record.ENVLIST == '/env/env.list'
    assert '/env.list' not in (RECORD / 'Dockerfile').read_text().replace('/env/env.list', '')


def test_compose_files_do_not_mount_host_systemd_by_default():
    for path in (REPO / 'compositions/blackfly.yaml', REPO / 'systems/components/record.yaml'):
        text = path.read_text()
        assert 'system_bus_socket' not in text and '/bin/systemctl:' not in text, path
