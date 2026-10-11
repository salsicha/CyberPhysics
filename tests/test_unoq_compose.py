"""UNO Q compositions: what the board-side ROS containers need on the real board.

Checked on an UNO Q (Debian 13, Docker 26.1.5): containers there get an
open-file limit of about 2^30, and LTTng-UST, which every ROS 2 Jazzy process
loads, allocates and zeroes a descriptor bitmap sized by the hard limit
(128 MiB), so each node used about 200 MB of the board's 1.7 GB instead of
66 MB.
"""
import yaml

from conftest import REPO

TURRET = yaml.safe_load((REPO / 'compositions/turret_uno_q.yaml').read_text())['services']
LED = yaml.safe_load((REPO / 'compositions/arduino_uno_q.yaml').read_text())['services']

BOARD_SERVICES = {
    'turret_unoq_bridge': TURRET['turret_unoq_bridge'],
    'unoq_sub': LED['unoq_sub'],
    'unoq_led_on': LED['unoq_led_on'],
}
ROUTER_SOCKET = '/var/run/arduino-router.sock'


def test_board_services_cap_the_open_file_limit():
    for name, service in BOARD_SERVICES.items():
        nofile = service['ulimits']['nofile']
        # LTTng-UST's bitmap is hard-limit bits: 524288 is 64 KiB
        assert 1024 <= nofile['soft'] <= nofile['hard'] <= 1 << 20, name


def test_long_running_services_restart_on_a_recreated_router_socket():
    for name in ('turret_unoq_bridge', 'unoq_sub'):
        service = BOARD_SERVICES[name]
        # The node exits after router_lost_exit_s; the restart mounts the new socket
        assert service['restart'] == 'unless-stopped', name
        mounts = [v for v in service['volumes']
                  if isinstance(v, dict) and v['target'] == ROUTER_SOCKET]
        assert len(mounts) == 1, name
        # The socket file itself, never /var/run (which would expose docker.sock)
        assert mounts[0]['source'] == ROUTER_SOCKET, name
        assert mounts[0]['bind']['create_host_path'] is False, name
