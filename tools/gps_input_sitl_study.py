#!/usr/bin/env python3
"""GPS_INPUT ingredient study on standalone ArduCopter SITL
(tools/gps_denied_sitl_test.md, "Late and bursty corrections").

Starts ArduCopter 4.6.3 SITL (the navigator image's build, built-in quad
physics, the Aerodrone parameter files) in a container on its own port,
flies GUIDED legs, optionally denies GPS 1 (SIM_GPS_DISABLE) and switches to
EKF source set 2 with GPS2_TYPE=14 as the navigation bridge does, streams
GPS_INPUT built from simulator truth (SIM_STATE) with a chosen lateness,
jitter, bursts, time stamps and fields, then descends in GUIDED at a land
speed (Aerostack2's land) and records ArduPilot's estimate
(LOCAL_POSITION_NED) against the truth. Writes <name>.json (errors per
phase: horizontal, height, vertical and horizontal velocity; the landing's
climb; counted status texts), <name>.samples.jsonl and the dataflash log.

  pip install pymavlink        # on the host, once
  python3 tools/gps_input_sitl_study.py --name late1 --out /tmp/study --port 15760 \
      --deny-s 400 --deny-after-s 40 --cruise-s 200 --gps-input --delay-s 1

All scheduling is on the vehicle's clock (time_boot_ms), so --speedup does not
change the delays. Ingredients: --delay-s/--jitter-s (late, FIFO),
--burst-s (held and released together), --stamp capture|send|none (GPS
week/ms of the truth sample, of the send time, or absent), --fields
(vel_zero, vel_true, velh_true, alt_zero, alt_true, vacc: what is claimed
besides horizontal position), --set2-delay-s/--gps2-delay-s (the order of
source set 2 and GPS2_TYPE=14 after the loss).
"""
import argparse
import collections
import json
import math
import os
import random
import subprocess
import sys
import time

from pymavlink import mavutil

M = mavutil.mavlink
HOME = (37.9234, -122.5967, 781.0)
IMAGE = os.environ.get('SITL_IMAGE', 'salsicha/cyberphysics:navigator-latest')
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

IGNORE_ALT, IGNORE_HDOP, IGNORE_VDOP = 1, 2, 4
IGNORE_VEL_HORIZ, IGNORE_VEL_VERT, IGNORE_SPEED_ACC = 8, 16, 32
IGNORE_HACC, IGNORE_VACC = 64, 128
BRIDGE_FLAGS = (IGNORE_ALT | IGNORE_VDOP | IGNORE_VEL_HORIZ | IGNORE_VEL_VERT
                | IGNORE_SPEED_ACC | IGNORE_VACC)


def ne_from(lat, lon, lat0, lon0):
    r = 6378137.0
    return (math.radians(lat - lat0) * r,
            math.radians(lon - lon0) * r * math.cos(math.radians(lat0)))


def latlon_from(n, e, lat0, lon0):
    r = 6378137.0
    return (lat0 + math.degrees(n / r),
            lon0 + math.degrees(e / (r * math.cos(math.radians(lat0)))))


class Run:
    def __init__(self, a):
        self.a = a
        self.rng = random.Random(a.seed)
        self.t_ms = 0           # vehicle clock
        self.truth = collections.deque(maxlen=4000)  # (t_ms, lat, lon, alt, vn, ve, vd)
        self.lpn = None
        self.texts = []
        self.mode = None
        self.armed = False
        self.queue = []         # (release_t_ms, fields)
        self.next_capture = None
        self.burst_until = None
        self.samples = []
        self.events = []
        self.ekf_flags = 0
        self.gps1_fix = 0
        self.gps2 = None
        self.sent = 0
        self.origin = None      # truth lat/lon at the EKF origin
        self.ned_offset = (0.0, 0.0)

    # --- link -----------------------------------------------------------
    def connect(self):
        deadline = time.time() + 60
        while True:
            try:
                self.m = mavutil.mavlink_connection(f'tcp:127.0.0.1:{self.a.port}',
                                                    source_system=255, source_component=190)
                break
            except OSError:
                if time.time() > deadline:
                    raise
                time.sleep(0.5)
        self.m.wait_heartbeat(timeout=60)
        for msg_id, hz in ((108, 25), (32, 20), (193, 5), (24, 5), (124, 5), (74, 5),
                           (30, 10), (33, 5), (2, 2), (1, 1)):
            self.cmd(M.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)

    def cmd(self, command, *params):
        p = list(params) + [0] * (7 - len(params))
        self.m.mav.command_long_send(1, 1, command, 0, *p)

    def param(self, name, value):
        self.m.mav.param_set_send(1, 1, name.encode(), float(value),
                                  M.MAV_PARAM_TYPE_REAL32)
        self.events.append((self.t_ms, f'param {name}={value}'))

    def pump(self, seconds=0.0):
        """Read messages for `seconds` of vehicle time (at least one poll)."""
        end = self.t_ms + seconds * 1000
        while True:
            msg = self.m.recv_match(blocking=True, timeout=0.05)
            if msg is not None:
                self.handle(msg)
            self.service()
            if self.t_ms >= end:
                return

    def handle(self, msg):
        t = msg.get_type()
        if t == 'LOCAL_POSITION_NED':
            self.t_ms = max(self.t_ms, msg.time_boot_ms)
            self.lpn = msg
            self.sample()
        elif t == 'ATTITUDE':
            self.t_ms = max(self.t_ms, msg.time_boot_ms)
        elif t == 'SIM_STATE':
            lat = msg.lat_int * 1e-7 if getattr(msg, 'lat_int', 0) else msg.lat
            lon = msg.lon_int * 1e-7 if getattr(msg, 'lon_int', 0) else msg.lon
            self.truth.append((self.t_ms, lat, lon, msg.alt, msg.vn, msg.ve, msg.vd))
        elif t == 'STATUSTEXT':
            text = msg.text
            self.texts.append((self.t_ms, text))
            if self.a.verbose:
                print(f'  [{self.t_ms / 1000:7.1f}] FCU: {text}', flush=True)
        elif t == 'HEARTBEAT' and msg.get_srcComponent() == 1:
            self.mode = msg.custom_mode
            self.armed = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
        elif t == 'EKF_STATUS_REPORT':
            self.ekf_flags = msg.flags
            self.ekf_var = (msg.velocity_variance, msg.pos_horiz_variance,
                            msg.pos_vert_variance)
        elif t == 'GPS_RAW_INT':
            self.gps1_fix = msg.fix_type
        elif t == 'GPS2_RAW':
            self.gps2 = (msg.fix_type, msg.lat, msg.lon)

    def truth_at(self, t_ms):
        best = None
        for item in reversed(self.truth):
            if item[0] <= t_ms:
                best = item
                break
        return best or (self.truth[0] if self.truth else None)

    def sample(self):
        if not self.truth or self.lpn is None or self.origin is None:
            return
        tr = self.truth[-1]
        n, e = ne_from(tr[1], tr[2], *self.origin)
        lp = self.lpn
        self.samples.append({
            't': round(self.t_ms / 1000, 3),
            'ekf': [round(lp.x, 2), round(lp.y, 2), round(lp.z, 2),
                    round(lp.vx, 3), round(lp.vy, 3), round(lp.vz, 3)],
            'truth': [round(n - self.ned_offset[0], 2), round(e - self.ned_offset[1], 2),
                      round(-(tr[3] - HOME[2]), 2), round(tr[4], 3), round(tr[5], 3),
                      round(tr[6], 3)],
            'mode': self.mode, 'armed': self.armed, 'phase': self.phase,
            'flags': self.ekf_flags})

    # --- GPS_INPUT generator --------------------------------------------
    def gps_input_active(self):
        return self.a.gps_input and self.phase in self.a.gps_input_phases

    def service(self):
        a = self.a
        if not self.gps_input_active() or not self.truth:
            self.next_capture = None
            return
        if self.next_capture is None:
            self.next_capture = self.t_ms
        while self.t_ms >= self.next_capture:
            tr = self.truth_at(self.next_capture)
            delay = a.delay_s + self.rng.uniform(0, a.jitter_s)
            release = self.next_capture + delay * 1000
            if a.burst_s > 0:
                # hold everything until the next burst boundary
                release = (math.floor(release / (a.burst_s * 1000)) + 1) * a.burst_s * 1000
            self.queue.append((release, self.next_capture, tr))
            self.next_capture += 1000.0 / a.rate_hz
        due = [q for q in self.queue if q[0] <= self.t_ms]
        self.queue = [q for q in self.queue if q[0] > self.t_ms]
        due.sort(key=lambda q: q[1])
        for release, capture, tr in due:
            if a.max_age_s > 0 and (self.t_ms - capture) / 1000 > a.max_age_s:
                continue
            self.send_gps_input(capture, tr)

    def send_gps_input(self, capture_ms, tr):
        a = self.a
        _, lat, lon, alt, vn, ve, vd = tr
        n_err = self.rng.gauss(0, a.noise_m)
        e_err = self.rng.gauss(0, a.noise_m)
        n, e = ne_from(lat, lon, *HOME[:2])
        lat_s, lon_s = latlon_from(n + n_err, e + e_err, *HOME[:2])
        flags = BRIDGE_FLAGS
        out_alt, out_vn, out_ve, out_vd = 0.0, 0.0, 0.0, 0.0
        if 'vel_zero' in a.fields:
            flags &= ~(IGNORE_VEL_HORIZ | IGNORE_VEL_VERT)
        if 'vel_true' in a.fields:
            flags &= ~(IGNORE_VEL_HORIZ | IGNORE_VEL_VERT)
            out_vn, out_ve, out_vd = vn, ve, vd
        if 'velh_true' in a.fields:
            flags &= ~IGNORE_VEL_HORIZ
            out_vn, out_ve = vn, ve
        if 'alt_zero' in a.fields:
            flags &= ~IGNORE_ALT
        if 'alt_true' in a.fields:
            flags &= ~IGNORE_ALT
            out_alt = alt
        if 'vacc' in a.fields:
            flags &= ~IGNORE_VACC
        stamp_ms = {'capture': capture_ms, 'send': self.t_ms}.get(a.stamp, capture_ms)
        week, week_ms = 2400, int(100_000 + stamp_ms) % 604_800_000
        if a.stamp == 'none':
            week, week_ms = 0, 0
        self.m.mav.gps_input_send(
            int(stamp_ms * 1000), 1, flags, week_ms, week, 3,
            int(round(lat_s * 1e7)), int(round(lon_s * 1e7)), out_alt,
            max(0.5, a.acc_m / 5.0), 0.0, out_vn, out_ve, out_vd,
            0.5, a.acc_m, 2.0 if 'vacc' in a.fields else 0.0, 6, 0)
        self.sent += 1

    # --- flight ---------------------------------------------------------
    def goto(self, n, e, d):
        self.m.mav.set_position_target_local_ned_send(
            0, 1, 1, M.MAV_FRAME_LOCAL_NED, 0b110111111000, n, e, d,
            0, 0, 0, 0, 0, 0, 0, 0)

    def vel(self, vn, ve, vd):
        self.m.mav.set_position_target_local_ned_send(
            0, 1, 1, M.MAV_FRAME_LOCAL_NED, 0b110111000111, 0, 0, 0,
            vn, ve, vd, 0, 0, 0, 0, 0)

    def fly(self):
        a = self.a
        self.phase = 'init'
        self.connect()
        t_wall = time.time()
        # Ready: GPS 3D and EKF absolute position
        while not (self.gps1_fix >= 3 and self.ekf_flags & 0x10 and self.truth):
            self.pump(0.5)
            if time.time() - t_wall > 180:
                raise RuntimeError('EKF never ready')
        self.pump(5)
        tr = self.truth[-1]
        self.origin = (tr[1], tr[2])
        self.ned_offset = (self.lpn.x * 0, 0.0)
        # EKF origin vs truth: align NE using the EKF's own position at rest
        n, e = ne_from(tr[1], tr[2], *self.origin)
        self.ned_offset = (n - self.lpn.x, e - self.lpn.y)
        for name, value in a.set_param:
            self.param(name, value)
        self.pump(1)
        self.m.mav.set_mode_send(1, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
        self.pump(1)
        self.cmd(M.MAV_CMD_COMPONENT_ARM_DISARM, 1, 21196)
        self.pump(2)
        self.cmd(M.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, a.alt)
        self.phase = 'takeoff'
        self.events.append((self.t_ms, 'takeoff'))
        while self.lpn.z > -(a.alt - 1.5):
            self.pump(0.5)
        self.phase = 'cruise'
        self.events.append((self.t_ms, 'cruise'))
        legs = [(a.leg, 0), (a.leg, a.leg), (0, a.leg), (0, 0)]
        leg_i = 0
        t_cruise = self.t_ms
        t_deny = t_cruise + a.deny_after_s * 1000 if a.deny_s > 0 else None
        t_restore = t_deny + a.deny_s * 1000 if t_deny else None
        t_end = t_cruise + a.cruise_s * 1000
        set2_at = None
        gps2_at = None
        set1_at = None
        while self.t_ms < t_end:
            target = legs[leg_i % 4]
            self.goto(target[0], target[1], -a.alt)
            self.pump(0.5)
            tr = self.truth[-1]
            n, e = ne_from(tr[1], tr[2], *self.origin)
            if math.hypot(n - self.ned_offset[0] - target[0], e - self.ned_offset[1] - target[1]) < 5:
                leg_i += 1
            if t_deny and self.t_ms >= t_deny and self.phase == 'cruise':
                self.param('SIM_GPS_DISABLE', 1)
                self.phase = 'denied'
                set2_at = self.t_ms + 1000 + a.set2_delay_s * 1000
                gps2_at = self.t_ms + 1000 + a.gps2_delay_s * 1000
                self.events.append((self.t_ms, 'gps1 denied'))
            if gps2_at and self.t_ms >= gps2_at:
                if a.gps2_type_on_deny:
                    self.param('GPS2_TYPE', 14)
                gps2_at = None
            if set2_at and self.t_ms >= set2_at:
                self.cmd(42007, 2)
                self.events.append((self.t_ms, 'source set 2'))
                set2_at = None
            if t_restore and a.restore and self.t_ms >= t_restore and self.phase == 'denied':
                self.param('SIM_GPS_DISABLE', 0)
                self.phase = 'restored'
                set1_at = self.t_ms + 10000
                self.events.append((self.t_ms, 'gps1 restored'))
            if set1_at and self.t_ms >= set1_at:
                self.cmd(42007, 1)
                self.events.append((self.t_ms, 'source set 1'))
                set1_at = None
        # Land: back over the start, then a GUIDED descent at land_speed
        land_phase = 'land_denied' if self.phase == 'denied' else 'land'
        self.phase = 'return'
        t0 = self.t_ms
        while self.t_ms - t0 < 60000:
            self.goto(0, 0, -a.alt)
            self.pump(0.5)
            tr = self.truth[-1]
            n, e = ne_from(tr[1], tr[2], *self.origin)
            if math.hypot(n - self.ned_offset[0], e - self.ned_offset[1]) < 5:
                break
        self.phase = land_phase
        self.events.append((self.t_ms, 'land'))
        t0 = self.t_ms
        while self.t_ms - t0 < a.land_s * 1000:
            self.vel(0, 0, a.land_speed)
            self.pump(0.2)
            if -(self.truth[-1][3] - HOME[2]) > -0.3 and self.t_ms - t0 > 3000:
                self.events.append((self.t_ms, 'touchdown'))
                break
        self.cmd(M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196)
        self.pump(1)


def summarize(run):
    out = {'sent': run.sent, 'events': run.events}
    by = collections.defaultdict(list)
    for s in run.samples:
        by[s['phase']].append(s)

    def stats(values):
        if not values:
            return None
        v = sorted(values)
        return [round(v[len(v) // 2], 3), round(v[min(len(v) - 1, int(0.95 * len(v)))], 3),
                round(v[-1], 3)]
    for phase, ss in by.items():
        h = [math.hypot(s['ekf'][0] - s['truth'][0], s['ekf'][1] - s['truth'][1]) for s in ss]
        vz = [abs(s['ekf'][5] - s['truth'][5]) for s in ss]
        dz = [abs(s['ekf'][2] - s['truth'][2]) for s in ss]
        vh = [math.hypot(s['ekf'][3] - s['truth'][3], s['ekf'][4] - s['truth'][4]) for s in ss]
        out[phase] = {'n': len(ss), 'horiz_err_m': stats(h), 'vz_err_mps': stats(vz),
                      'height_err_m': stats(dz), 'vel_h_err_mps': stats(vh)}
    for phase in ('land', 'land_denied'):
        ss = by.get(phase)
        if ss:
            up0 = -ss[0]['truth'][2]
            ups = [-s['truth'][2] for s in ss]
            out[phase]['start_agl_m'] = round(up0, 2)
            out[phase]['max_agl_m'] = round(max(ups), 2)
            out[phase]['end_agl_m'] = round(ups[-1], 2)
            out[phase]['climbed_m'] = round(max(ups) - up0, 2)
            out[phase]['seconds'] = round(ss[-1]['t'] - ss[0]['t'], 1)
            out[phase]['touchdown'] = any(e[1] == 'touchdown' for e in run.events)
    keys = {'vibration': 'Vibration compensation ON', 'glitch': 'Glitch',
            'yaw_reset': 'yaw reset', 'ekf_failsafe': 'EKF variance',
            'gps2': 'GPS 2', 'lane': 'lane switch', 'aiding': 'aiding',
            'failsafe': 'ailsafe'}
    out['texts'] = {k: sum(1 for _, t in run.texts if v.lower() in t.lower())
                    for k, v in keys.items()}
    out['texts_all'] = sorted(set(t for _, t in run.texts))
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--name', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--port', type=int, required=True)
    p.add_argument('--speedup', type=float, default=4)
    p.add_argument('--seed', type=int, default=1)
    p.add_argument('--alt', type=float, default=40)
    p.add_argument('--leg', type=float, default=200)
    p.add_argument('--cruise-s', type=float, default=200)
    p.add_argument('--deny-after-s', type=float, default=40)
    p.add_argument('--deny-s', type=float, default=0)
    p.add_argument('--restore', action='store_true')
    p.add_argument('--land-s', type=float, default=150)
    p.add_argument('--land-speed', type=float, default=0.5)
    p.add_argument('--gps-input', action='store_true')
    p.add_argument('--gps-input-phases', default='takeoff,cruise,denied,restored,return,land,land_denied')
    p.add_argument('--gps2-type-on-deny', type=int, default=1)
    p.add_argument('--set2-delay-s', type=float, default=0.0)
    p.add_argument('--gps2-delay-s', type=float, default=0.0)
    p.add_argument('--rate-hz', type=float, default=2)
    p.add_argument('--noise-m', type=float, default=3)
    p.add_argument('--acc-m', type=float, default=30)
    p.add_argument('--delay-s', type=float, default=0.05)
    p.add_argument('--jitter-s', type=float, default=0.0)
    p.add_argument('--burst-s', type=float, default=0.0)
    p.add_argument('--max-age-s', type=float, default=0.0)
    p.add_argument('--stamp', default='capture', choices=['capture', 'send', 'none'])
    p.add_argument('--fields', default='')
    p.add_argument('--param', action='append', default=[], help='NAME=VALUE at boot (defaults file)')
    p.add_argument('--set', action='append', default=[], help='NAME=VALUE set before take-off')
    p.add_argument('--verbose', action='store_true')
    a = p.parse_args()
    a.gps_input_phases = a.gps_input_phases.split(',')
    a.fields = [f for f in a.fields.split(',') if f]
    a.set_param = [(kv.split('=')[0], float(kv.split('=')[1])) for kv in a.set]
    os.makedirs(a.out, exist_ok=True)
    extra = os.path.join(a.out, f'{a.name}.parm')
    with open(extra, 'w') as f:
        for kv in a.param:
            f.write(kv.replace('=', ',') + '\n')
    container = f'gi_sitl_{a.name}'
    subprocess.run(['docker', 'rm', '-f', container], capture_output=True)
    cfg = f'{REPO}/systems/aerodrone/config'
    cmd = ('cd /ardupilot && exec build/sitl/bin/arducopter -S --model quad '
           f'--speedup {a.speedup} -I0 --home {HOME[0]},{HOME[1]},{HOME[2]},0 '
           '--defaults Tools/autotest/default_params/copter.parm,/cfg/arducopter.params,'
           '/cfg/arducopter_sitl.params,/cfg/arducopter_gps_input.params,'
           f'/out/{a.name}.parm')
    subprocess.run(['docker', 'run', '-d', '--name', container, '-p', f'127.0.0.1:{a.port}:5760',
                    '-v', f'{cfg}:/cfg:ro', '-v', f'{os.path.abspath(a.out)}:/out',
                    '--entrypoint', 'bash', IMAGE, '-c', cmd], check=True, capture_output=True)
    run = Run(a)
    run.phase = 'init'
    status = 'ok'
    t0 = time.time()
    try:
        run.fly()
    except Exception as exc:  # noqa: BLE001
        status = f'error: {exc!r}'
    finally:
        # keep the dataflash log
        subprocess.run(['docker', 'exec', container, 'bash', '-c',
                        f'mkdir -p /out/{a.name}_logs && cp /ardupilot/logs/*.BIN /out/{a.name}_logs/ 2>/dev/null; '
                        f'chmod -R a+rw /out/{a.name}_logs'], capture_output=True)
        subprocess.run(['docker', 'rm', '-f', container], capture_output=True)
    summary = summarize(run)
    summary.update({'name': a.name, 'status': status, 'wall_s': round(time.time() - t0, 1),
                    'args': {k: v for k, v in vars(a).items() if k not in ('set_param',)}})
    with open(os.path.join(a.out, f'{a.name}.json'), 'w') as f:
        json.dump(summary, f, indent=1)
    with open(os.path.join(a.out, f'{a.name}.samples.jsonl'), 'w') as f:
        for s in run.samples:
            f.write(json.dumps(s) + '\n')
    print(json.dumps({k: summary[k] for k in summary if k not in ('args', 'events', 'texts_all')}))
    return 0 if status == 'ok' else 1


if __name__ == '__main__':
    sys.exit(main())
