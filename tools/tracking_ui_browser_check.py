#!/usr/bin/env python3
"""Headless-browser check of the aerodrone web UI against the mock tracker.

Run by tools/tracking_ui_test.sh inside the Playwright image, next to
rosbridge, the page's HTTP server and
applications/aerostack2/scripts/mock_tracking_publisher.py (``--static
--navigation --command-log``). With a desktop and a phone browser it checks
that

- the page connects and draws a box, in the box colour, around every object
  of the frame it shows;
- clicking (desktop) or tapping (phone) a box publishes the track command of
  the contract (systems/aerodrone/tracking.md) in each mode, with the
  entered values, and that invalid values publish nothing;
- the tracker's target is drawn highlighted and shown in the status;
- Apply re-sends the mode to the current target, Stop publishes
  {"action": "stop"}, and the map goal is still published;
- the layout fits a phone without horizontal scrolling, with Stop on screen.

Screenshots go to --out. Exits non-zero on the first failed check.
"""

import argparse
import json
import sys
import time
from pathlib import Path

from playwright.sync_api import sync_playwright

BOX_RGB = (0, 229, 255)
TARGET_RGB = (255, 179, 0)


class Commands:
    """The mock's received-message log, read incrementally."""

    def __init__(self, path):
        self.path = Path(path)
        self.offset = self.path.stat().st_size if self.path.exists() else 0

    def next(self, timeout_s=5.0, topic='/tracking/command'):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self.path.open() as stream:
                stream.seek(self.offset)
                line = stream.readline()
                if line.endswith('\n'):
                    self.offset = stream.tell()
                    record = json.loads(line)
                    if record['topic'] == topic:
                        return record
                    continue
            time.sleep(0.1)
        return None

    def none_within(self, seconds):
        record = self.next(timeout_s=seconds)
        return record is None, record


def check(condition, message):
    if not condition:
        raise AssertionError(message)
    print(f'ok: {message}', flush=True)


def wait_for_boxes(page, count):
    page.wait_for_function(
        f"""() => {{
          const ui = window.__trackingUi;
          return ui && ui.state.connected && ui.state.frame && ui.state.drawn &&
            ui.state.drawn.boxes.length >= {count};
        }}""", timeout=30000)


def drawn(page):
    """The boxes on screen, in camera_info pixels, and the canvas geometry."""
    return page.evaluate("""() => {
      const s = window.__trackingUi.state;
      const r = document.getElementById('camera').getBoundingClientRect();
      return {boxes: s.drawn.boxes, imageWidth: s.drawn.imageWidth,
              imageHeight: s.drawn.imageHeight, alpha: s.drawn.alpha, exact: s.drawn.exact,
              rect: {left: r.left, top: r.top, width: r.width, height: r.height}};
    }""")


def client_point(view, box, dx=0.0, dy=0.0):
    """The page point of an image pixel, computed here from the canvas rect
    (not by the page), so the page's own mapping is what is under test."""
    rect = view['rect']
    return (rect['left'] + (box['cx'] + dx) * rect['width'] / view['imageWidth'],
            rect['top'] + (box['cy'] + dy) * rect['height'] / view['imageHeight'])


def box_by_id(page, track_id):
    view = drawn(page)
    for box in view['boxes']:
        if box['id'] == track_id:
            return view, box
    raise AssertionError(f'box #{track_id} is not drawn: {view["boxes"]}')


def edge_colour_distance(page, view, box, rgb):
    """Smallest distance to rgb of the canvas pixels on the box's left edge."""
    return page.evaluate("""([view, box, rgb]) => {
      const canvas = document.getElementById('camera');
      const context = canvas.getContext('2d');
      const ratio = canvas.width / view.rect.width;
      const x = (box.cx - box.width / 2) * view.rect.width / view.imageWidth * ratio;
      const y = box.cy * view.rect.height / view.imageHeight * ratio;
      const data = context.getImageData(Math.round(x) - 3, Math.round(y) - 3, 7, 7).data;
      let best = Infinity;
      for (let i = 0; i < data.length; i += 4) {
        best = Math.min(best, Math.hypot(data[i] - rgb[0], data[i + 1] - rgb[1],
                                         data[i + 2] - rgb[2]));
      }
      return best;
    }""", [view, box, list(rgb)])


def wait_status(page, state, target_id=None):
    page.wait_for_function(
        """([state, target]) => {
          const s = window.__trackingUi.state.status;
          return s && s.state === state && (target === null || s.target_id === target);
        }""", arg=[state, target_id], timeout=10000)


def select_mode(page, mode):
    page.locator(f'input[name="mode"][value="{mode}"]').check(force=True)


def click_box(page, track_id):
    view, box = box_by_id(page, track_id)
    page.mouse.click(*client_point(view, box))


def expect_command(record, expected):
    check(record is not None, f'a command was published for {expected}')
    command = json.loads(record['data'])
    check(command == expected, f'command {record["data"]} == {json.dumps(expected)}')
    for key in ('altitude_agl_m', 'slant_range_m', 'depression_deg'):
        if key in command:
            check(isinstance(command[key], (int, float)), f'{key} is a JSON number')
    if 'target_id' in command:
        check(isinstance(command['target_id'], str), 'target_id is a JSON string')


def desktop(browser, url, commands, out):
    context = browser.new_context(viewport={'width': 1440, 'height': 900})
    page = context.new_page()
    errors = []
    page.on('console', lambda message: message.type == 'error' and errors.append(
        (message.text, message.location.get('url', ''))))
    page.on('pageerror', lambda error: errors.append((str(error), '')))
    page.goto(url)
    wait_for_boxes(page, 4)
    page.wait_for_function('() => window.__trackingUi.state.drawn.exact', timeout=5000)
    view = drawn(page)
    check(view['imageWidth'] == 640 and view['imageHeight'] == 480,
          'boxes are in the camera_info resolution, 640x480')
    check(view['exact'] and view['alpha'] == 1, 'the boxes are the shown frame\'s own detections')
    check(abs(view['rect']['width'] / view['rect']['height'] - 640 / 480) < 0.01,
          'the camera keeps its aspect ratio')
    for box in view['boxes']:
        distance = edge_colour_distance(page, view, box, BOX_RGB)
        check(distance < 40, f'box #{box["id"]} ({box["className"]}) is drawn '
              f'(edge colour distance {distance:.0f})')
    page.screenshot(path=str(out / 'desktop_boxes.png'))
    # A healthy GPS without vision corrections (the fusion then says
    # dead_reckoning) is GPS navigation.
    page.wait_for_function("() => document.getElementById('nav-badge').textContent"
                           " === 'Navigation: GPS'", timeout=10000)
    check('ok' in page.get_attribute('#nav-badge', 'class'),
          'the navigation badge shows GPS with the fusion at dead_reckoning')

    # Centered, with a height above ground.
    page.fill('#altitude-agl', '40')
    click_box(page, '3')
    expect_command(commands.next(),
                   {'action': 'track', 'target_id': '3', 'mode': 'centered',
                    'altitude_agl_m': 40})
    wait_status(page, 'tracking', '3')
    page.wait_for_timeout(700)
    view, box = box_by_id(page, '3')
    check(edge_colour_distance(page, view, box, TARGET_RGB) < 40,
          'the tracked target #3 is drawn highlighted')
    check('#3' in page.inner_text('#st-target'), 'the status shows target #3')
    check(page.inner_text('#state-chip').strip().lower() == 'tracking',
          'the state chip shows tracking')
    page.wait_for_function("() => document.querySelectorAll('path.leaflet-interactive')"
                           ".length >= 2", timeout=5000)
    check(True, 'the drone and the target are on the map')
    page.screenshot(path=str(out / 'desktop_tracking_centered.png'))

    # Centered, empty height: the field is left out (current height).
    page.fill('#altitude-agl', '')
    click_box(page, '12')
    expect_command(commands.next(), {'action': 'track', 'target_id': '12', 'mode': 'centered'})

    # Invalid heights publish nothing.
    for text in ('200', '10'):
        page.fill('#altitude-agl', text)
        click_box(page, '12')
        quiet, record = commands.none_within(1.5)
        check(quiet, f'height {text} m publishes nothing (got {record})')
        check('15-120' in page.inner_text('#mode-error'), f'height {text} m shows the limits')
    page.fill('#altitude-agl', '')

    # Slant range.
    select_mode(page, 'slant')
    check(page.is_visible('#slant-range') and not page.is_visible('#altitude-agl'),
          'slant mode shows its own inputs')
    page.fill('#slant-range', '60')
    page.fill('#depression', '70')
    check('Stand-off 20.5 m' in page.inner_text('#slant-derived'),
          'slant mode shows the stand-off')
    click_box(page, '12')
    expect_command(commands.next(), {'action': 'track', 'target_id': '12', 'mode': 'slant',
                                     'slant_range_m': 60, 'depression_deg': 70})
    wait_status(page, 'tracking', '12')
    page.wait_for_timeout(700)
    page.screenshot(path=str(out / 'desktop_tracking_slant.png'))
    for depression in ('59', '91', ''):
        page.fill('#depression', depression)
        click_box(page, '7')
        quiet, record = commands.none_within(1.5)
        check(quiet, f'depression {depression!r} publishes nothing (got {record})')
    page.fill('#depression', '70')
    page.fill('#slant-range', '200')
    click_box(page, '7')
    quiet, record = commands.none_within(1.5)
    check(quiet and 'Slant range must be' in page.inner_text('#mode-error'),
          'a slant range above 120 m / sin(70 deg) publishes nothing')

    # Apply sends the new values to the current target.
    page.fill('#slant-range', '50')
    page.fill('#depression', '80')
    page.click('#apply')
    expect_command(commands.next(), {'action': 'track', 'target_id': '12', 'mode': 'slant',
                                     'slant_range_m': 50, 'depression_deg': 80})

    # A click away from every box publishes nothing.
    view = drawn(page)
    corner = {'cx': 3, 'cy': view['imageHeight'] - 3}
    page.mouse.click(*client_point(view, corner))
    quiet, record = commands.none_within(1.5)
    check(quiet, f'a click on no box publishes nothing (got {record})')

    # Stop.
    page.click('#stop')
    expect_command(commands.next(), {'action': 'stop'})
    wait_status(page, 'idle')
    check(page.inner_text('#state-chip').strip().lower() == 'idle', 'Stop: the tracker is idle')

    # The map goal still works.
    map_box = page.locator('#map').bounding_box()
    page.mouse.click(map_box['x'] + map_box['width'] * 0.7, map_box['y'] + map_box['height'] * 0.4)
    page.fill('#altitude', '12')
    page.click('#send')
    goal = commands.next(topic='/aerostack/map_goal')
    check(goal is not None and goal['data'].count(';') == 2 and
          goal['data'].endswith(';12'), f'the map goal is published: {goal and goal["data"]}')
    page.screenshot(path=str(out / 'desktop_after_stop.png'))
    # Map tiles need the internet; everything else must load.
    unexpected = [error for error in errors if 'tile.openstreetmap.org' not in error[1]]
    check(not unexpected, f'no console errors: {unexpected}')
    context.close()


def phone(browser, url, commands, out, width=390, height=844, name='phone'):
    context = browser.new_context(viewport={'width': width, 'height': height},
                                  device_scale_factor=3, is_mobile=True, has_touch=True)
    page = context.new_page()
    page.goto(url)
    wait_for_boxes(page, 3)
    layout = page.evaluate("""() => {
      const stop = document.getElementById('stop').getBoundingClientRect();
      const canvas = document.getElementById('camera').getBoundingClientRect();
      return {scrollWidth: document.documentElement.scrollWidth, innerWidth: innerWidth,
              innerHeight: innerHeight, stopBottom: stop.bottom, stopTop: stop.top,
              stopHeight: stop.height, canvasWidth: canvas.width};
    }""")
    check(layout['scrollWidth'] <= layout['innerWidth'],
          f'{name}: no horizontal scrolling ({layout["scrollWidth"]} <= {layout["innerWidth"]})')
    check(0 <= layout['stopTop'] and layout['stopBottom'] <= layout['innerHeight'] and
          layout['stopHeight'] >= 44, f'{name}: Stop is on screen and finger-sized')
    if width < 900:  # one column below 900 px
        check(layout['canvasWidth'] >= layout['innerWidth'] - 40,
              f'{name}: the camera spans the width ({layout["canvasWidth"]:.0f} px)')
    page.screenshot(path=str(out / f'{name}_boxes.png'))
    # Tap the small person #7 (about 10 px) a little outside its box.
    view, box = box_by_id(page, '7')
    x, y = client_point(view, box, dx=box['width'] / 2 + 6 * view['imageWidth'] /
                        view['rect']['width'])
    page.touchscreen.tap(x, y)
    expect_command(commands.next(), {'action': 'track', 'target_id': '7', 'mode': 'centered'})
    wait_status(page, 'tracking', '7')
    page.wait_for_timeout(700)
    page.screenshot(path=str(out / f'{name}_tracking.png'))
    page.screenshot(path=str(out / f'{name}_full_page.png'), full_page=True)
    # Stop in the bar at the bottom of the screen.
    page.locator('#stop').tap()
    expect_command(commands.next(), {'action': 'stop'})
    wait_status(page, 'idle')
    context.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--url', required=True)
    parser.add_argument('--commands', required=True, help="the mock's --command-log")
    parser.add_argument('--out', required=True, help='screenshot directory')
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    commands = Commands(args.commands)
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch()
        try:
            desktop(browser, args.url, commands, out)
            phone(browser, args.url, commands, out)
            phone(browser, args.url, commands, out, width=1024, height=768,
                  name='tablet_landscape')
        finally:
            browser.close()
    print('all browser checks passed', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
