"""whereami WiFi localization: scanners, model reloading, prediction, the
ROS node's publishing, and the CLI wrapper's scanner injection.

scikit-learn, access_points and rclpy are not installed in CI: the model is
a fake with classes_/predict_proba, scans come from injected scanners, and
ROS modules are stubbed.
"""
import json
import math
import os
import subprocess
import sys
import types
from types import SimpleNamespace as NS
from unittest import mock

import pytest

from conftest import REPO, install_ros_stubs, load_script

APP = REPO / 'applications/whereami'
SCRIPTS = APP / 'scripts'

KITCHEN = [dict(ssid='home', bssid='aa:aa', quality=80, security='WPA2'),
           dict(ssid='guest', bssid='bb:bb', quality=30, security='')]


@pytest.fixture
def wl(monkeypatch):
    monkeypatch.syspath_prepend(str(SCRIPTS))
    return load_script(SCRIPTS / 'whereami_localizer.py', 'whereami_localizer')


class FakeModel:
    """Nearest-fingerprint classifier with whereami's pipeline interface."""
    classes_ = ['kitchen', 'lab']

    def __init__(self):
        self.seen = []

    def predict_proba(self, X):
        self.seen.append(X)
        (sample,) = X
        return [[0.9, 0.1]] if 'home aa:aa' in sample else [[0.2, 0.8]]


class ListScanner:
    def __init__(self, *scans):
        self.scans = list(scans)

    def get_access_points(self):
        item = self.scans.pop(0) if len(self.scans) > 1 else self.scans[0]
        if isinstance(item, Exception):
            raise item
        return item


def write_model(model_dir, content=b'model'):
    model_dir.mkdir(parents=True, exist_ok=True)
    (model_dir / 'model.pkl').write_bytes(content)


# Features ---------------------------------------------------------------------

def test_features_match_the_vendored_whereami(wl):
    """aps_to_dict must build the same features whereami trained on."""
    stubs = {'access_points': mock.MagicMock(), 'whereami': types.ModuleType('whereami'),
             'whereami.utils': mock.MagicMock()}
    with mock.patch.dict(sys.modules, stubs):
        vendored = load_script(APP / 'whereami/whereami/get_data.py', 'vendored_get_data')
    assert wl.aps_to_dict(KITCHEN) == vendored.aps_to_dict(KITCHEN) == {
        'home aa:aa': 80, 'guest bb:bb': 30}


def test_vendored_license_is_kept():
    text = (APP / 'whereami/LICENSE').read_text()
    assert 'GNU AFFERO GENERAL PUBLIC LICENSE' in text
    assert 'AGPL' in (APP / 'README.md').read_text()


# Scanners ---------------------------------------------------------------------

def test_nmcli_and_iwlist_commands(wl):
    assert wl.nmcli_command() == ['nmcli', '-t', '-f', 'ssid,bssid,signal,security',
                                  'device', 'wifi', 'list', '--rescan', 'auto']
    assert wl.nmcli_command('wlan0', 'yes')[-4:] == ['ifname', 'wlan0', '--rescan', 'yes']
    assert wl.iwlist_command() == ['iwlist', 'scanning']
    assert wl.iwlist_command('wlan0') == ['iwlist', 'wlan0', 'scanning']


def test_auto_prefers_nmcli_with_the_host_dbus(wl):
    parsers = {'nmcli': 'parse-nm', 'iwlist': 'parse-iw'}
    tools = {'nmcli', 'iwlist'}
    which = tools.__contains__
    scanner = wl.make_scanner('auto', parsers=parsers, which=which, exists=lambda p: True)
    assert scanner.command[0] == 'nmcli' and scanner.parse == 'parse-nm'
    # No D-Bus socket mounted: iwlist
    scanner = wl.make_scanner('auto', 'wlan0', parsers=parsers, which=which,
                              exists=lambda p: False)
    assert scanner.command == ['iwlist', 'wlan0', 'scanning']
    with pytest.raises(wl.ScanError, match='No WiFi scanner'):
        wl.make_scanner('auto', parsers=parsers, which=lambda t: False, exists=lambda p: True)
    with pytest.raises(ValueError):
        wl.make_scanner('airport')
    with pytest.raises(ValueError):
        wl.make_scanner('file')


def test_command_scanner_runs_with_timeout_and_parses(wl):
    calls = []

    def run(command, **kwargs):
        calls.append((command, kwargs))
        return NS(returncode=0, stdout=b'home:aa\\:aa:80:WPA2\n', stderr=b'')

    def parse(output):
        return [dict(ssid='home', bssid='aa:aa', quality=80, raw=output)]

    scanner = wl.CommandScanner(['nmcli', 'x'], parse, timeout_s=5.0, run=run)
    aps = scanner.get_access_points()
    assert aps[0]['raw'] == 'home:aa\\:aa:80:WPA2\n'
    assert calls[0][1]['timeout'] == 5.0 and calls[0][1]['capture_output']


@pytest.mark.parametrize('failure, match', [
    (FileNotFoundError(), 'not installed'),
    (subprocess.TimeoutExpired('nmcli', 5), 'timed out'),
    (NS(returncode=10, stdout=b'', stderr=b'Error: NetworkManager is not running.'),
     'NetworkManager is not running'),
])
def test_command_scanner_errors(wl, failure, match):
    def run(command, **kwargs):
        if isinstance(failure, Exception):
            raise failure
        return failure
    with pytest.raises(wl.ScanError, match=match):
        wl.CommandScanner(['nmcli'], lambda out: [], run=run).get_access_points()


def test_file_scanner_replays_access_points_json(wl, tmp_path):
    path = tmp_path / 'scan.json'
    path.write_text(json.dumps(KITCHEN))
    scanner = wl.make_scanner('file', scan_file=str(path))
    assert wl.aps_to_dict(scanner.get_access_points()) == {'home aa:aa': 80, 'guest bb:bb': 30}
    path.write_text('{"not": "a list"}')
    with pytest.raises(wl.ScanError):
        scanner.get_access_points()
    path.write_text('[{"ssid": "x"}]')
    with pytest.raises(wl.ScanError):
        scanner.get_access_points()
    with pytest.raises(wl.ScanError):
        wl.FileScanner(str(tmp_path / 'missing.json')).get_access_points()


# Model store and prediction ------------------------------------------------------

def test_model_store_loads_and_reloads_on_change(wl, tmp_path):
    loads = []

    def loader(path):
        loads.append(open(path, 'rb').read())
        return FakeModel()

    store = wl.ModelStore(str(tmp_path / 'models'), loader=loader)
    assert store.get() == (None, False) and store.error is None
    write_model(tmp_path / 'models', b'v1')
    model, reloaded = store.get()
    assert isinstance(model, FakeModel) and reloaded
    assert store.labels == ['kitchen', 'lab']
    assert store.get() == (model, False)  # unchanged: cached
    write_model(tmp_path / 'models', b'v2-retrained')
    model2, reloaded = store.get()
    assert reloaded and model2 is not model and loads == [b'v1', b'v2-retrained']


def test_model_store_reports_a_broken_model(wl, tmp_path):
    def loader(path):
        raise EOFError('Ran out of input')
    write_model(tmp_path, b'')
    store = wl.ModelStore(str(tmp_path), loader=loader)
    assert store.get() == (None, False)
    assert 'Ran out of input' in store.error and 'train' in store.error


def test_predict_with_probabilities(wl):
    label, confidence, probabilities = wl.predict(FakeModel(), {'home aa:aa': 80})
    assert (label, confidence) == ('kitchen', 0.9)
    assert probabilities == {'kitchen': 0.9, 'lab': 0.1}
    model = NS(predict=lambda X: ['lab'])  # no predict_proba
    label, confidence, probabilities = wl.predict(model, {})
    assert label == 'lab' and math.isnan(confidence) and probabilities == {}


# Localizer ----------------------------------------------------------------------

def make_localizer(wl, tmp_path, scanner, model=True):
    if model:
        write_model(tmp_path)
    store = wl.ModelStore(str(tmp_path), loader=lambda path: FakeModel())
    factory_calls = []

    def factory():
        factory_calls.append(1)
        if isinstance(scanner, Exception):
            raise scanner
        return scanner
    localizer = wl.Localizer(factory, store)
    localizer.factory_calls = factory_calls
    return localizer


def test_localizer_without_model_explains_training(wl, tmp_path):
    localizer = make_localizer(wl, tmp_path, ListScanner(KITCHEN), model=False)
    result = localizer.step()
    assert result.status == wl.NO_MODEL
    assert 'model.pkl' in result.message and 'whereami_train learn -l' in result.message
    assert localizer.factory_calls == []  # no scanning without a model


def test_localizer_predicts_location(wl, tmp_path):
    lab = [dict(ssid='lab', bssid='cc:cc', quality=60, security='')]
    localizer = make_localizer(wl, tmp_path, ListScanner(KITCHEN, lab))
    result = localizer.step()
    assert (result.status, result.label, result.confidence) == (wl.OK, 'kitchen', 0.9)
    assert result.access_points == 2 and result.model_reloaded
    result = localizer.step()
    assert (result.label, result.probabilities) == ('lab', {'kitchen': 0.2, 'lab': 0.8})
    assert not result.model_reloaded and localizer.factory_calls == [1]


def test_localizer_reports_scan_problems(wl, tmp_path):
    localizer = make_localizer(wl, tmp_path, ListScanner(wl.ScanError('nmcli timed out'), []))
    assert localizer.step().status == wl.SCAN_ERROR
    assert localizer.step().status == wl.NO_ACCESS_POINTS
    broken = make_localizer(wl, tmp_path / 'x', wl.ScanError('No WiFi scanner'))
    result = broken.step()
    assert result.status == wl.SCAN_ERROR and 'No WiFi scanner' in result.message


def test_localizer_reports_prediction_failure(wl, tmp_path):
    store = wl.ModelStore(str(tmp_path), loader=lambda path: NS(
        classes_=['a'], predict_proba=mock.Mock(side_effect=ValueError('bad features'))))
    write_model(tmp_path)
    result = wl.Localizer(lambda: ListScanner(KITCHEN), store).step()
    assert result.status == wl.MODEL_ERROR and 'bad features' in result.message


# ROS node -------------------------------------------------------------------------

class FakeNode:
    params = {}

    def __init__(self, name):
        self.published = {}
        self.timers = []
        self.log = mock.MagicMock()

    def declare_parameter(self, name, default):
        self.params.setdefault(name, default)

    def get_parameter(self, name):
        return NS(value=self.params[name])

    def create_publisher(self, msg_type, topic, qos):
        self.published[topic] = []
        return NS(publish=self.published[topic].append)

    def create_timer(self, period, callback):
        self.timers.append((period, callback))

    def get_logger(self):
        return self.log


@pytest.fixture
def node_module(wl):
    with mock.patch.dict(sys.modules):
        install_ros_stubs()
        sys.modules['rclpy.node'].Node = FakeNode
        sys.modules['rclpy.executors'] = mock.MagicMock()
        sys.modules['std_msgs.msg'].String = lambda data: NS(data=data)
        sys.modules['std_msgs.msg'].Float32 = lambda data: NS(data=data)
        yield load_script(SCRIPTS / 'whereami_node.py', 'whereami_node_test')


def test_node_publishes_location_confidence_and_probabilities(wl, node_module, tmp_path):
    FakeNode.params = {'scan_period_s': 5.0}
    localizer = make_localizer(wl, tmp_path, ListScanner(KITCHEN))
    node = node_module.WhereamiNode(localizer=localizer)
    # Scans once at start-up, then on the timer
    assert node.timers[0][0] == 5.0
    assert [m.data for m in node.published['/whereami/location']] == ['kitchen']
    assert [m.data for m in node.published['/whereami/confidence']] == [0.9]
    assert json.loads(node.published['/whereami/probabilities'][0].data) == {
        'kitchen': 0.9, 'lab': 0.1}
    node.timers[0][1]()
    assert len(node.published['/whereami/location']) == 2
    logged = ' '.join(str(c) for c in node.log.info.call_args_list)
    assert 'locations kitchen, lab' in logged and 'Location: kitchen' in logged


def test_node_logs_missing_model_and_publishes_nothing(wl, node_module, tmp_path):
    FakeNode.params = {}
    localizer = make_localizer(wl, tmp_path, ListScanner(KITCHEN), model=False)
    node = node_module.WhereamiNode(localizer=localizer)
    node.timers[0][1]()
    assert all(not msgs for msgs in node.published.values())
    # One warning for the state, not one per scan
    assert node.log.warn.call_count == 1
    assert 'No whereami model' in node.log.warn.call_args[0][0]
    # Training while the node runs: the next scan picks the model up
    write_model(tmp_path)
    node.timers[0][1]()
    assert [m.data for m in node.published['/whereami/location']] == ['kitchen']


def test_node_builds_scanner_from_parameters_and_env(wl, node_module, tmp_path, monkeypatch):
    scan = tmp_path / 'scan.json'
    scan.write_text(json.dumps(KITCHEN))
    monkeypatch.setenv('WHEREAMI_SCANNER', 'file')
    monkeypatch.setenv('WHEREAMI_SCAN_FILE', str(scan))
    FakeNode.params = {'model_dir': str(tmp_path / 'models')}
    node = node_module.WhereamiNode()
    assert node.localizer.model_store.path == str(tmp_path / 'models' / 'model.pkl')
    assert node.localizer._scanner_factory().path == str(scan)
    FakeNode.params = {'scanner': 'sonar'}
    with pytest.raises(ValueError):
        node_module.WhereamiNode()


# CLI wrapper ----------------------------------------------------------------------

def test_cli_installs_the_container_scanner(wl, monkeypatch, tmp_path):
    scan = tmp_path / 'scan.json'
    scan.write_text(json.dumps(KITCHEN))
    monkeypatch.setenv('WHEREAMI_SCANNER', 'file')
    monkeypatch.setenv('WHEREAMI_SCAN_FILE', str(scan))
    package = types.ModuleType('whereami')
    get_data = types.ModuleType('whereami.get_data')
    predict = types.ModuleType('whereami.predict')
    get_data.get_scanner = predict.get_scanner = 'access_points.get_scanner'
    package.get_data, package.predict = get_data, predict
    with mock.patch.dict(sys.modules, {'whereami': package, 'whereami.get_data': get_data,
                                       'whereami.predict': predict}):
        cli = load_script(SCRIPTS / 'whereami_cli.py', 'whereami_cli_test')
        cli.install_scanner()
    assert get_data.get_scanner is cli.scanner_from_env
    assert predict.get_scanner is cli.scanner_from_env
    # whereami's sample() calls get_scanner(device).get_access_points()
    assert get_data.get_scanner('').get_access_points() == KITCHEN
    monkeypatch.setenv('WHEREAMI_SCANNER', 'nmcli')
    monkeypatch.setattr(cli, 'make_scanner', lambda *a, **kw: (a, kw))
    args, kwargs = cli.scanner_from_env('wlan1')
    assert args == ('nmcli', 'wlan1') and kwargs['rescan'] == 'yes'


def test_compose_mounts_model_dir_and_dbus_read_only():
    yaml = pytest.importorskip('yaml')
    services = yaml.safe_load((REPO / 'compositions/whereami.yaml').read_text())['services']
    node, train = services['whereami'], services['whereami_train']
    assert 'privileged' not in node and 'privileged' not in train
    assert train['profiles'] == ['train']
    assert '/scripts/whereami_cli.py' in train['entrypoint']

    def mounts(service):
        out = {}
        for volume in service['volumes']:
            if isinstance(volume, str):
                parts = volume.split(':')
                out[parts[1]] = 'ro' in parts[2:]
            else:
                out[volume['target']] = volume.get('read_only', False)
        return out
    assert mounts(node)['/root/.whereami'] is True
    assert mounts(train)['/root/.whereami'] is False
    for service in (node, train):
        assert mounts(service)['/run/dbus/system_bus_socket'] is True


def test_node_default_model_dir_is_whereamis_learn_dir(node_module):
    assert node_module.DEFAULT_MODEL_DIR == os.path.expanduser('~/.whereami')
