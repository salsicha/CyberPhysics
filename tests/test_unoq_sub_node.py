"""UNO Q Sub demo, Linux side: Int32 messages become set_led RPCs.

Runs applications/arduino/scripts/unoq_sub/unoq_sub_node.py with ROS stubbed
out against a fake arduino-router on a Unix socket.
"""
import shutil
import socket
import sys
import tempfile
import threading
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import pytest
from conftest import REPO, install_ros_stubs, load_script

msgpack = pytest.importorskip('msgpack')


@pytest.fixture(scope='module')
def module():
    with patch.dict(sys.modules):
        install_ros_stubs()
        yield load_script(REPO / 'applications/arduino/scripts/unoq_sub/unoq_sub_node.py',
                          'unoq_sub_node')


class FakeRouter(threading.Thread):
    """Answers set_led like the unoq_sub sketch; 'boom' fails; closes on 'drop'."""

    def __init__(self, path):
        super().__init__(daemon=True)
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.server.bind(path)
        self.server.listen(4)
        self.calls = []

    def run(self):
        while True:
            try:
                conn, _ = self.server.accept()
            except OSError:
                return
            unpacker = msgpack.Unpacker(raw=False)
            with conn:
                while data := conn.recv(4096):
                    unpacker.feed(data)
                    for _, msgid, method, params in unpacker:
                        self.calls.append((method, params))
                        if params == ['drop']:
                            break
                        if method == 'set_led':
                            reply = [1, msgid, None, params[0] != 0]
                        else:
                            reply = [1, msgid, [2, f'method {method} not available'], None]
                        conn.sendall(msgpack.packb(reply))
                    else:
                        continue
                    break


@pytest.fixture
def router():
    directory = tempfile.mkdtemp(prefix='unoq-', dir='/tmp')
    fake = FakeRouter(str(Path(directory) / 'router.sock'))
    fake.start()
    yield fake, str(Path(directory) / 'router.sock')
    fake.server.close()
    shutil.rmtree(directory, ignore_errors=True)


def test_router_client_calls_and_reports_errors(module, router):
    fake, path = router
    client = module.RouterClient(path, 1.0)
    assert client.call('set_led', 1) is True
    assert client.call('set_led', 0) is False
    with pytest.raises(RuntimeError, match='not available'):
        client.call('boom')
    with pytest.raises(OSError):
        client.call('set_led', 'drop')  # the router closes the connection
    assert client.sock is None
    assert client.call('set_led', 5) is True  # reconnects
    assert fake.calls == [('set_led', [1]), ('set_led', [0]), ('boom', []),
                          ('set_led', ['drop']), ('set_led', [5])]
    client.close()


def test_int32_messages_call_set_led(module, router):
    fake, path = router
    node = module.UnoQSubNode.__new__(module.UnoQSubNode)
    node.get_logger = MagicMock()
    node.router = module.RouterClient(path, 1.0)
    node._led(NS(data=1))
    node._led(NS(data=0))
    assert fake.calls == [('set_led', [1]), ('set_led', [0])]
    node.router.close()
    node.router = module.RouterClient(path + '.missing', 0.2)
    node._led(NS(data=1))  # logged, not raised
    assert node.get_logger().error.called
