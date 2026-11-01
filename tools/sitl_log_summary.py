#!/usr/bin/env python3
"""Summarise an ArduCopter SITL dataflash log from a simulator run (the HIL
runner keeps them in <run>/sitl/logs, tools/aerodrone_hil_test.md):

- ArduPilot's EKF (XKF1, core 0) against the simulator's state as SITL
  received it (SIM2): horizontal, height and vertical-velocity error;
- the attitude loop: roll and pitch about their mean, roll rate, motor 1's
  output (RCOU C1) and the IMU's mean vertical specific force (about
  -9.81 m/s^2 in steady flight; well beyond it when the vehicle shakes);
- ArduPilot's warnings (vibration compensation, GPS glitch, yaw resets, EKF
  failsafe, lane switches).

A HIL mission whose attitude loop oscillated (2026-11-01, run gi1) shows
roll and pitch standard deviations over 20 deg, motor outputs swinging by
300 PWM, the IMU at -11.7 m/s^2 and the EKF's vertical velocity 2 m/s off;
all-local runs show 2-5 deg, 30-40 PWM, -9.8 m/s^2 and 0.01-0.05 m/s.

  pip install pymavlink
  python3 tools/sitl_log_summary.py generated/aerodrone_hil/mission/sitl/logs/00000001.BIN.gz
"""
import argparse
import bisect
import gzip
import json
import math
import os
import shutil
import statistics
import sys
import tempfile

from pymavlink import mavutil

WARNINGS = ('Vibration compensation ON', 'GPS Glitch', 'yaw reset', 'EKF variance',
            'EKF Failsafe', 'lane switch')


def quantiles(values):
    if not values:
        return None
    v = sorted(values)
    pick = lambda q: round(v[min(len(v) - 1, int(q * (len(v) - 1) + 0.5))], 3)  # noqa: E731
    return {'median': pick(0.5), 'p95': pick(0.95), 'max': pick(1.0)}


def summarise(path):
    log = mavutil.mavlink_connection(path)
    ekf, truth, roll, pitch, rate, c1, accz, texts = [], [], [], [], [], [], [], []
    while True:
        msg = log.recv_match(type=['XKF1', 'SIM2', 'ATT', 'RATE', 'RCOU', 'IMU', 'MSG'])
        if msg is None:
            break
        kind = msg.get_type()
        if kind == 'XKF1' and msg.C == 0:
            ekf.append((msg.TimeUS, msg.PN, msg.PE, msg.PD, msg.VD))
        elif kind == 'SIM2':
            truth.append((msg.TimeUS, msg.PN, msg.PE, msg.PD, msg.VD))
        elif kind == 'ATT':
            roll.append(msg.Roll)
            pitch.append(msg.Pitch)
        elif kind == 'RATE':
            rate.append(msg.R)
        elif kind == 'RCOU':
            c1.append(msg.C1)
        elif kind == 'IMU' and msg.I == 0:
            accz.append(msg.AccZ)
        elif kind == 'MSG':
            texts.append(msg.Message)
    out = {'log': path}
    if ekf and truth:
        times = [row[0] for row in truth]

        def at(t_us):
            i = min(max(bisect.bisect_left(times, t_us), 0), len(truth) - 1)
            return truth[i]
        first = at(ekf[0][0])
        offset = [ekf[0][k] - first[k] for k in (1, 2, 3)]
        horizontal, height, vertical_velocity = [], [], []
        for t_us, pn, pe, pd, vd in ekf:
            ref = at(t_us)
            horizontal.append(math.hypot(pn - offset[0] - ref[1], pe - offset[1] - ref[2]))
            height.append(abs(pd - offset[2] - ref[3]))
            vertical_velocity.append(abs(vd - ref[4]))
        out['ekf_error'] = {'horizontal_m': quantiles(horizontal), 'height_m': quantiles(height),
                            'vertical_velocity_mps': quantiles(vertical_velocity)}
    spread = lambda v: round(statistics.pstdev(v), 2) if len(v) > 1 else None  # noqa: E731
    out['attitude_loop'] = {
        'roll_sd_deg': spread(roll), 'pitch_sd_deg': spread(pitch),
        'roll_rate_sd_dps': spread(rate), 'motor1_pwm_sd': spread(c1),
        'imu_vertical_specific_force_mps2': round(statistics.mean(accz), 2) if accz else None}
    out['warnings'] = {w: sum(1 for t in texts if w.lower() in t.lower()) for w in WARNINGS}
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('logs', nargs='+', help='dataflash logs (.BIN or .BIN.gz)')
    args = parser.parse_args()
    for path in args.logs:
        if path.endswith('.gz'):
            with gzip.open(path, 'rb') as src, tempfile.NamedTemporaryFile(
                    suffix='.BIN', delete=False) as dst:
                shutil.copyfileobj(src, dst)
            try:
                result = summarise(dst.name)
            finally:
                os.unlink(dst.name)
            result['log'] = path
        else:
            result = summarise(path)
        print(json.dumps(result, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
