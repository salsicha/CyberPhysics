#!/usr/bin/env python3
"""Headless-browser check of the aerodrone page served by the companion
computer during a hardware-in-the-loop tracking run
(tools/aerodrone_hil_test.md, "Tracking and launcher").

Run by tools/aerodrone_hil_test.sh in the Playwright image on the
workstation, against the Pi's aerostack_gui (port 8080) and rosbridge (9092),
while the test driver follows the car: the page loads from the Pi, connects
to its rosbridge, and draws the tracking camera's frames (the workstation's
detector -> Wi-Fi -> the Pi's rosbridge -> Wi-Fi -> this browser) with the
detections' boxes and the tracker's status. It only watches: it clicks
nothing and publishes nothing (the driver commands the tracker).

For --seconds it polls the page every 20 ms and records each new frame's
age, this host's wall clock minus the frame's camera stamp (the OAK-1
stand-in on this same host stamps frames in wall time, so no clock offset
enters), the boxes drawn and the tracker state shown. Screenshots and
page_check.json go to --out. Exits 1 when a check fails.
"""
import argparse
import itertools
import json
import sys
import time
from pathlib import Path

from playwright.sync_api import sync_playwright

POLL = """() => {
  const ui = window.__trackingUi;
  if (!ui) return null;
  const s = ui.state;
  const f = s.frame;
  return {now: Date.now(), connected: !!s.connected,
          stamp: f && f.stamp ? f.stamp.sec + f.stamp.nanosec * 1e-9 : null,
          boxes: s.drawn ? s.drawn.boxes.length : 0,
          classes: s.drawn ? s.drawn.boxes.map(b => b.cls || b.label || '') : [],
          status: s.status ? s.status.state : null,
          note: document.getElementById('camera-note').textContent,
          frameInfo: document.getElementById('frame-info').textContent};
}"""

# An in-page sampler: every 20 ms, the wall time at which a new frame stamp
# first shows in the page's state (the frame is drawn in the same callback).
SAMPLER = """() => {
  window.__hilFrames = [];
  let last = null;
  window.__hilSampler = setInterval(() => {
    const f = window.__trackingUi && window.__trackingUi.state.frame;
    if (!f || !f.stamp) return;
    const stamp = f.stamp.sec + f.stamp.nanosec * 1e-9;
    if (stamp !== last) {
      last = stamp;
      const d = window.__trackingUi.state.drawn;
      window.__hilFrames.push([Date.now() / 1000, stamp, d ? d.boxes.length : 0]);
    }
  }, 20);
}"""


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    position = (len(values) - 1) * fraction
    low = int(position)
    high = min(low + 1, len(values) - 1)
    return values[low] + (values[high] - values[low]) * (position - low)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--url', required=True)
    parser.add_argument('--seconds', type=float, default=60.0)
    parser.add_argument('--out', required=True)
    parser.add_argument('--min-fps', type=float, default=3.0)
    parser.add_argument('--max-age-p95-s', type=float, default=1.0)
    args = parser.parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    report = {'url': args.url, 'checks': {}}
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch()
        page = browser.new_page(viewport={'width': 1366, 'height': 900})
        console = []
        page.on('console', lambda msg: console.append(f'{msg.type}: {msg.text}'))
        started = time.time()
        page.goto(args.url, wait_until='load', timeout=60000)
        report['load_s'] = round(time.time() - started, 2)
        page.wait_for_function('() => window.__trackingUi && window.__trackingUi.state.connected',
                               timeout=30000)
        report['connected_after_s'] = round(time.time() - started, 2)
        page.wait_for_function('() => window.__trackingUi.state.frame', timeout=30000)
        report['first_frame_after_s'] = round(time.time() - started, 2)
        page.screenshot(path=str(out / 'page_start.png'))
        page.evaluate(SAMPLER)
        states, notes, polls = {}, set(), 0
        end = time.time() + args.seconds
        while time.time() < end:
            sample = page.evaluate(POLL)
            if sample:
                polls += 1
                states[sample['status']] = states.get(sample['status'], 0) + 1
                notes.add(sample['note'])
            time.sleep(0.5)
        frames = page.evaluate('() => window.__hilFrames')
        page.screenshot(path=str(out / 'page_end.png'))
        phone = browser.new_page(viewport={'width': 390, 'height': 844}, is_mobile=True,
                                 has_touch=True)
        phone.goto(args.url, wait_until='load', timeout=60000)
        phone.wait_for_function('() => window.__trackingUi && window.__trackingUi.state.frame',
                                timeout=30000)
        phone.wait_for_timeout(2000)
        phone.screenshot(path=str(out / 'page_phone.png'), full_page=True)
        browser.close()
    ages = [shown - stamp for shown, stamp, _ in frames]
    span = frames[-1][0] - frames[0][0] if len(frames) > 1 else 0.0
    fps = (len(frames) - 1) / span if span > 0 else 0.0
    with_boxes = sum(1 for *_, boxes in frames if boxes > 0)
    gaps = [b[0] - a[0] for a, b in itertools.pairwise(frames)]
    report.update({
        'seconds': args.seconds, 'frames': len(frames), 'fps': round(fps, 2),
        'frames_with_boxes': with_boxes,
        'max_frame_gap_s': round(max(gaps), 3) if gaps else None,
        'frame_age_s': ({'median': round(percentile(ages, 0.5), 3),
                         'p95': round(percentile(ages, 0.95), 3),
                         'max': round(max(ages), 3), 'min': round(min(ages), 3)}
                        if ages else None),
        'status_states': states, 'camera_notes': sorted(notes), 'polls': polls,
        'console_errors': [line for line in console if line.startswith('error')][:20],
    })
    checks = report['checks']
    checks['connected_and_drawing'] = len(frames) > 0
    checks['fps'] = fps >= args.min_fps
    checks['boxes_drawn'] = bool(frames) and with_boxes >= 0.5 * len(frames)
    checks['tracker_status_shown'] = any(state for state in states if state)
    checks['frame_age_p95'] = bool(ages) and percentile(ages, 0.95) <= args.max_age_p95_s
    report['pass'] = all(checks.values())
    (out / 'page_check.json').write_text(json.dumps(report, indent=1))
    print(json.dumps(report, indent=1))
    return 0 if report['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
