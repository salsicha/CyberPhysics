"""Aerodrone web UI for object tracking (applications/aerostack2/src/
as2_platform_blueos/web): pinned CDN libraries, the page's wiring against
the contract (systems/aerodrone/tracking.md), its logic module under node
(tests/js/tracking_logic.test.js), and the mock tracker of
applications/aerostack2/scripts/mock_tracking_publisher.py."""
import json
import re
import shutil
import subprocess
from html.parser import HTMLParser

import pytest
import yaml

from conftest import REPO, load_script

WEB = REPO / 'applications/aerostack2/src/as2_platform_blueos/web'
CONTRACT = REPO / 'systems/aerodrone/tracking.md'
mock = load_script(REPO / 'applications/aerostack2/scripts/mock_tracking_publisher.py',
                   'mock_tracking_publisher')


class Page(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = set()
        self.scripts = []
        self.styles = []

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if 'id' in attrs:
            self.ids.add(attrs['id'])
        if tag == 'script' and 'src' in attrs:
            self.scripts.append(attrs)
        if tag == 'link' and attrs.get('rel') == 'stylesheet':
            self.styles.append(attrs)


def page():
    parsed = Page()
    parsed.feed((WEB / 'index.html').read_text())
    return parsed


def contract_examples():
    row = next(line for line in CONTRACT.read_text().splitlines()
               if line.startswith('| `/tracking/command`'))
    return [json.loads(text) for text in re.findall(r'`(\{[^`]*\})`', row)]


def test_cdn_libraries_are_pinned_with_subresource_integrity():
    parsed = page()
    external = [(item['src'], item) for item in parsed.scripts if '://' in item['src']] + \
        [(item['href'], item) for item in parsed.styles]
    assert {url.split('/npm/')[1].split('/')[0] for url, _ in external} == {
        'leaflet@1.9.4', 'roslib@1.4.1'}
    for url, attrs in external:
        assert re.match(r'https://cdn\.jsdelivr\.net/npm/[a-z]+@\d+\.\d+\.\d+/', url), url
        assert re.fullmatch(r'sha(256|384|512)-[A-Za-z0-9+/]+=*', attrs.get('integrity', '')), url
        assert attrs.get('crossorigin') == 'anonymous', url
    local = [item['src'] for item in parsed.scripts if '://' not in item['src']]
    assert local == ['tracking_logic.js', 'app.js']  # logic first: app.js uses it
    for name in local:
        assert (WEB / name).is_file()


def test_every_element_the_script_uses_is_on_the_page():
    ids = page().ids
    used = set(re.findall(r"\$\('([\w-]+)'\)", (WEB / 'app.js').read_text()))
    used |= set(re.findall(r"getElementById\('([\w-]+)'\)", (WEB / 'app.js').read_text()))
    assert {'camera', 'stop', 'apply', 'map', 'send', 'altitude-agl', 'slant-range',
            'depression'} <= used
    assert used <= ids, used - ids


def test_page_subscribes_and_publishes_with_the_contract_types():
    app = (WEB / 'app.js').read_text()
    contract = CONTRACT.read_text()
    types = {
        'detections': 'vision_msgs/msg/Detection2DArray',
        'image': 'sensor_msgs/msg/CompressedImage',
        'cameraInfo': 'sensor_msgs/msg/CameraInfo',
        'status': 'std_msgs/msg/String',
        'command': 'std_msgs/msg/String',
    }
    for key, message_type in types.items():
        assert f'topic(topicNames.{key}, \'{message_type}\'' in app, key
        assert f'`{message_type}`' in contract, message_type
    # The map goal of the original page is kept.
    assert "topic('/aerostack/map_goal', 'std_msgs/msg/String')" in app
    assert '`${goal.east};${goal.north};${z}`' in app
    # Nothing queued while disconnected reaches the drone later.
    assert 'if (!state.connected)' in app.split('function publishCommand')[1][:400]
    assert 'goalTopic.publish' in app.split("if (!state.connected) {\n      $('selection')")[1]


def test_depression_and_height_inputs_carry_the_contract_limits():
    html = (WEB / 'index.html').read_text()
    assert re.search(r'id="depression"[^>]*min="60" max="90"', html)
    assert re.search(r'id="altitude-agl"[^>]*min="15" max="120"', html)
    logic = (WEB / 'tracking_logic.js').read_text()
    for text in ('minAglM: 15', 'maxAglM: 120', 'minDepressionDeg: 60', 'maxDepressionDeg: 90'):
        assert text in logic
    contract = CONTRACT.read_text()
    assert '`depression_deg` is limited to [60, 90]' in contract
    assert 'minimum height above ground 15 m, maximum 120 m' in contract


@pytest.mark.skipif(shutil.which('node') is None, reason='node is not installed')
def test_logic_module_under_node():
    result = subprocess.run(['node', str(REPO / 'tests/js/tracking_logic.test.js')],
                            capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr


# The mock tracker -----------------------------------------------------------

def test_mock_accepts_the_contract_examples():
    examples = contract_examples()
    assert len(examples) == 3
    for command in examples:
        assert mock.validate_command(command) == (True, '')


@pytest.mark.parametrize('command,error', [
    ({'action': 'track', 'target_id': 7}, 'target_id'),
    ({'action': 'track', 'target_id': '7', 'altitude_agl_m': 10}, 'altitude_agl_m'),
    ({'action': 'track', 'target_id': '7', 'altitude_agl_m': '40'}, 'altitude_agl_m'),
    ({'action': 'track', 'target_id': '7', 'mode': 'slant', 'slant_range_m': 60,
      'depression_deg': 59}, 'depression_deg'),
    ({'action': 'track', 'target_id': '7', 'mode': 'slant', 'depression_deg': 70},
     'slant_range_m'),
    ({'action': 'track', 'target_id': '7', 'mode': 'orbit'}, 'mode'),
    ({'action': 'land'}, 'action'),
    ([], 'object'),
])
def test_mock_rejects_invalid_commands(command, error):
    ok, message = mock.validate_command(command)
    assert not ok and error in message


def test_mock_tracker_states_follow_the_commands():
    world = mock.MockWorld()
    tracker = mock.MockTracker(world)
    assert tracker.status()['state'] == 'idle'
    tracker.handle(json.dumps({'action': 'track', 'target_id': '3', 'mode': 'centered',
                               'altitude_agl_m': 30}), now=10.0)
    status = tracker.status()
    assert (status['state'], status['target_id'], status['mode']) == ('acquiring', '3', 'centered')
    tracker.update(11.0)
    assert tracker.status()['state'] == 'tracking'
    car = next(item for item in world.objects() if item['id'] == '3')
    assert tracker.goal() == (car['east'], car['north'], 30)
    for _ in range(200):
        world.step(0.1, tracker.goal())
    status = tracker.status()
    assert status['depression_deg'] == pytest.approx(90.0, abs=1.0)
    assert status['slant_range_m'] == pytest.approx(30.0, abs=0.5)
    u, v = world.to_pixel(status['target']['east'], status['target']['north'])
    assert (u, v) == pytest.approx((320, 240), abs=10)  # centered in the image
    tracker.handle('{"action": "stop"}', now=30.0)
    assert tracker.status()['state'] == 'idle' and tracker.goal() is None
    tracker.handle('{"action": "track", "target_id": "99"}', now=31.0)
    assert tracker.status()['state'] == 'error' and '99' in tracker.status()['message']
    tracker.handle('not json', now=32.0)
    assert tracker.status()['state'] == 'error'


def test_mock_slant_goal_has_the_contract_geometry():
    world = mock.MockWorld(static=False)
    tracker = mock.MockTracker(world)
    tracker.handle(json.dumps({'action': 'track', 'target_id': '12', 'mode': 'slant',
                               'slant_range_m': 60, 'depression_deg': 70}), now=0.0)
    for _ in range(400):
        tracker.update(world.t)
        world.step(0.1, tracker.goal())
    status = tracker.status()
    assert status['state'] == 'tracking'
    assert status['slant_range_m'] == pytest.approx(60.0, abs=1.5)
    assert status['depression_deg'] == pytest.approx(70.0, abs=1.5)


def test_mock_boxes_have_stable_ids_inside_the_image():
    world = mock.MockWorld(static=True)
    boxes = world.boxes()
    assert {box['id'] for box in boxes} == {'3', '7', '12', '21'}
    for box in boxes:
        assert 0 <= box['cx'] <= world.width and 0 <= box['cy'] <= world.height
        assert box['size_x'] > 5 and box['size_y'] > 5
    world.step(5.0)
    assert world.boxes() == boxes  # static


# Serving the page ------------------------------------------------------------

def test_aircraft_serves_the_repository_page_next_to_rosbridge():
    """The page connects to rosbridge on its own host's port 9090."""
    compositions = REPO / 'compositions'
    gui = yaml.safe_load((compositions / 'blueos_aerostack.yaml').read_text())[
        'services']['aerostack_gui']
    assert gui['network_mode'] == 'host'
    assert '../applications/aerostack2/src/as2_platform_blueos/web:/workspace/web:ro' in \
        gui['volumes']
    assert gui['command'] == 'python3 -m http.server 8080 --directory /workspace/web'
    assert 'rosbridge' in gui['depends_on']
    rosbridge = yaml.safe_load((compositions / 'blueos_nav_core.yaml').read_text())[
        'services']['rosbridge']
    assert rosbridge['network_mode'] == 'host'
    assert rosbridge['command'] == 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml'
    assert ':9090`' in (WEB / 'app.js').read_text()
