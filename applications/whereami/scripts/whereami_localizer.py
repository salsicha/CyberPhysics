"""WiFi-fingerprint room localization around the vendored whereami package.

Plain Python: no ROS, and neither scikit-learn nor access_points is imported
at module load. The ROS node (whereami_node.py) and the CLI wrapper
(whereami_cli.py) build a scanner with make_scanner() and a ModelStore for
the model directory; tests inject fakes for both.

Scanners return access_points-style dicts (ssid, bssid, quality, security).
access_points.get_scanner() only picks nmcli when `systemctl status
NetworkManager` succeeds, which never happens in a container, and its
iwlist command runs through sudo; the scanners here run nmcli or iwlist
directly, with a timeout, and reuse access_points' output parsers.
"""
import json
import math
import os
import pickle
import shutil
import subprocess
from dataclasses import dataclass, field

SCANNERS = ('auto', 'nmcli', 'iwlist', 'file')
DBUS_SYSTEM_SOCKET = '/run/dbus/system_bus_socket'
MODEL_FILE = 'model.pkl'
TRAIN_COMMAND = 'docker compose -f compositions/whereami.yaml run --rm whereami_train'
TRAIN_HINT = TRAIN_COMMAND + ' learn -l <room> -n 10'

# Result status values
OK = 'ok'
NO_MODEL = 'no_model'
MODEL_ERROR = 'model_error'
SCAN_ERROR = 'scan_error'
NO_ACCESS_POINTS = 'no_access_points'


class ScanError(RuntimeError):
    """A WiFi scan could not be taken."""


def aps_to_dict(aps):
    """Feature dict of one scan: 'ssid bssid' -> quality.

    Must match whereami.get_data.aps_to_dict, which built the training
    samples (tests/test_whereami_node.py checks the vendored source).
    """
    return {ap["ssid"] + " " + ap["bssid"]: ap["quality"] for ap in aps}


# Scanners -------------------------------------------------------------------

def nmcli_command(device='', rescan='auto'):
    """NetworkManager's cached/rescanned access point list (terse output,
    the fields access_points.NetworkManagerWifiScanner parses)."""
    command = ['nmcli', '-t', '-f', 'ssid,bssid,signal,security', 'device', 'wifi', 'list']
    if device:
        command += ['ifname', device]
    if rescan:
        command += ['--rescan', rescan]
    return command


def iwlist_command(device=''):
    return ['iwlist', device, 'scanning'] if device else ['iwlist', 'scanning']


class CommandScanner:
    """Runs a scan command and parses its output into access points."""

    def __init__(self, command, parse, timeout_s=20.0, run=subprocess.run):
        self.command = list(command)
        self.parse = parse
        self.timeout_s = timeout_s
        self._run = run

    def get_access_points(self):
        try:
            proc = self._run(self.command, capture_output=True,
                             timeout=self.timeout_s, check=False)
        except FileNotFoundError as exc:
            raise ScanError(f'{self.command[0]} is not installed') from exc
        except subprocess.TimeoutExpired as exc:
            raise ScanError(f'{self.command[0]} timed out after {self.timeout_s:.0f} s') from exc
        output = (proc.stdout or b'').decode('utf8', errors='ignore')
        if proc.returncode != 0 and not output.strip():
            error = (proc.stderr or b'').decode('utf8', errors='ignore').strip()
            raise ScanError(f'{" ".join(self.command)} failed with exit code '
                            f'{proc.returncode}: {error or "no output"}')
        return self.parse(output)


class FileScanner:
    """Replays a scan from a JSON file: a list of access point dicts, as
    printed by the `access_points` command. Re-read on every scan."""

    def __init__(self, path):
        self.path = path

    def get_access_points(self):
        try:
            with open(self.path) as f:
                data = json.load(f)
        except (OSError, ValueError) as exc:
            raise ScanError(f'Cannot read scan file {self.path}: {exc}') from exc
        if not isinstance(data, list):
            raise ScanError(f'{self.path} must hold a JSON list of access points')
        try:
            return [dict(ssid=str(ap['ssid']), bssid=str(ap['bssid']),
                         quality=ap['quality'], security=ap.get('security', ''))
                    for ap in data]
        except (KeyError, TypeError, AttributeError) as exc:
            raise ScanError(f'{self.path}: access points need ssid, bssid and quality') from exc


def access_points_parsers():
    """Output parsers of the access_points package (imported on use)."""
    import access_points
    return {
        'nmcli': access_points.NetworkManagerWifiScanner('').parse_output,
        'iwlist': access_points.IwlistWifiScanner('').parse_output,
    }


def resolve_backend(backend, which=shutil.which, exists=os.path.exists):
    """'auto' -> nmcli when the host's system D-Bus is mounted, else iwlist."""
    if backend != 'auto':
        return backend
    if which('nmcli') and exists(DBUS_SYSTEM_SOCKET):
        return 'nmcli'
    if which('iwlist'):
        return 'iwlist'
    raise ScanError(f'No WiFi scanner: mount the host {DBUS_SYSTEM_SOCKET} for nmcli, '
                    'or install wireless-tools for iwlist')


def make_scanner(backend='auto', device='', scan_file='', timeout_s=20.0,
                 rescan='auto', parsers=None, which=shutil.which, exists=os.path.exists):
    """Scanner for `backend` (one of SCANNERS)."""
    if backend not in SCANNERS:
        raise ValueError(f'Unknown scanner {backend!r}; expected one of {SCANNERS}')
    if backend == 'file':
        if not scan_file:
            raise ValueError('scanner "file" needs a scan_file')
        return FileScanner(scan_file)
    backend = resolve_backend(backend, which, exists)
    parse = (parsers or access_points_parsers())[backend]
    if backend == 'nmcli':
        return CommandScanner(nmcli_command(device, rescan), parse, timeout_s)
    return CommandScanner(iwlist_command(device), parse, timeout_s)


# Model ----------------------------------------------------------------------

def load_pickle(path):
    # The model directory is the user's own training output (whereami's
    # format is a pickled scikit-learn pipeline); never mount untrusted files.
    with open(path, 'rb') as f:
        return pickle.load(f)


class ModelStore:
    """whereami's model.pkl in `model_dir`, reloaded whenever it changes."""

    def __init__(self, model_dir, loader=load_pickle):
        self.model_dir = model_dir
        self.path = os.path.join(model_dir, MODEL_FILE)
        self._loader = loader
        self._model = None
        self._stamp = None
        self.error = None

    def get(self):
        """(model or None, True if it was (re)loaded by this call)."""
        try:
            stat = os.stat(self.path)
        except OSError:
            self._model, self._stamp, self.error = None, None, None
            return None, False
        stamp = (stat.st_mtime_ns, stat.st_size)
        if stamp == self._stamp:
            return self._model, False
        self._stamp = stamp
        try:
            self._model = self._loader(self.path)
        except Exception as exc:  # unpickling runs arbitrary model code
            self._model = None
            self.error = (f'Cannot load {self.path}: {type(exc).__name__}: {exc}. '
                          f'Rebuild it from the samples with `{TRAIN_COMMAND} train`.')
            return None, False
        self.error = None
        return self._model, True

    @property
    def labels(self):
        """Locations the loaded model knows."""
        return [str(label) for label in getattr(self._model, 'classes_', [])]


def predict(model, sample):
    """(label, confidence, {label: probability}) for one feature dict."""
    if hasattr(model, 'predict_proba') and hasattr(model, 'classes_'):
        row = model.predict_proba([sample])[0]
        probabilities = {str(label): float(p) for label, p in zip(model.classes_, row)}
        label = max(probabilities, key=probabilities.get)
        return label, probabilities[label], probabilities
    return str(model.predict([sample])[0]), math.nan, {}


# Localizer ------------------------------------------------------------------

@dataclass
class Result:
    status: str
    label: str = ''
    confidence: float = math.nan
    probabilities: dict = field(default_factory=dict)
    access_points: int = 0
    message: str = ''
    model_reloaded: bool = False


class Localizer:
    """One scan-and-predict step per call; never raises for runtime faults."""

    def __init__(self, scanner_factory, model_store):
        self._scanner_factory = scanner_factory
        self._scanner = None
        self.model_store = model_store

    def step(self):
        model, reloaded = self.model_store.get()
        if model is None:
            if self.model_store.error:
                return Result(MODEL_ERROR, message=self.model_store.error)
            return Result(NO_MODEL, message=(
                f'No whereami model at {self.model_store.path} yet: learn each room '
                f'first, e.g. `{TRAIN_HINT}`'))
        try:
            if self._scanner is None:
                self._scanner = self._scanner_factory()
            aps = self._scanner.get_access_points()
        except ScanError as exc:
            return Result(SCAN_ERROR, message=str(exc), model_reloaded=reloaded)
        if not aps:
            return Result(NO_ACCESS_POINTS, message='The WiFi scan found no access points',
                          model_reloaded=reloaded)
        try:
            label, confidence, probabilities = predict(model, aps_to_dict(aps))
        except Exception as exc:
            return Result(MODEL_ERROR, message=f'Prediction failed: {type(exc).__name__}: {exc}',
                          model_reloaded=reloaded)
        return Result(OK, label, confidence, probabilities, len(aps),
                      model_reloaded=reloaded)
