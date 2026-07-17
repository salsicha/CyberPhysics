#!/usr/bin/env python3
"""Validate ArduPilot .params files against the pinned firmware source.

ArduPilot silently drops unknown parameter names from --add-param-file
defaults files, so a renamed parameter (e.g. TRIM_ARSPD_CM became
AIRSPEED_CRUISE in Plane 4.4) fails without any error at boot — the tuning
just never applies. This checks that every name in the given .params files
appears in the parameter tables of the pinned ArduPilot source.

Source of truth, in order of preference:
- a local checkout (default: cache/ardupilot) — the whole tree is scanned,
  so any parameter group resolves;
- otherwise the files in FETCH_FILES downloaded from raw.githubusercontent
  at the ref taken from --ref or ARDUPILOT_REF in --env-file. If a checked
  parameter belongs to a library not in FETCH_FILES, add that file.

Matching: a parameter NAME passes if "NAME" appears quoted in the sources,
or (for grouped parameters, whose tables store only the suffix) if a
suffix of NAME obtained by dropping leading underscore-separated tokens
appears quoted — e.g. TECS_SYNAIRSPEED matches "SYNAIRSPEED", ARSPD_USE
matches "USE". This can under-report a bad name whose suffix collides with
an unrelated parameter, but never rejects a valid one.

Usage:
    tools/check_ardupilot_params.py [params files...]
Defaults to systems/*/config/*.params. Exits nonzero if any name misses.
"""
import argparse
import glob
import os
import re
import sys
import urllib.request

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_ENV_FILE = os.path.join(REPO, 'systems/airplane/config/sitl.env')
DEFAULT_CHECKOUT = os.path.join(REPO, 'cache/ardupilot')
QUOTED_NAME = re.compile(r'"([A-Z][A-Z0-9_]{1,15})"')
# Sources fetched when no local checkout exists. ArduPlane/Parameters.cpp
# holds the top-level and group registrations; the libraries hold the
# grouped parameter tables used by systems/*/config/*.params today.
FETCH_FILES = [
    'ArduPlane/Parameters.cpp',
    'libraries/AP_Arming/AP_Arming.cpp',
    'libraries/AP_AHRS/AP_AHRS.cpp',
    'libraries/AP_NavEKF3/AP_NavEKF3.cpp',
    'libraries/AP_TECS/AP_TECS.cpp',
    'libraries/AP_Airspeed/AP_Airspeed.cpp',
]
RAW_URL = 'https://raw.githubusercontent.com/ArduPilot/ardupilot/{ref}/{path}'


def read_ref(env_file):
    with open(env_file) as fh:
        for line in fh:
            if line.startswith('ARDUPILOT_REF='):
                return line.split('=', 1)[1].strip()
    raise SystemExit(f'ARDUPILOT_REF not found in {env_file}')


def collect_tokens_from_text(text, tokens):
    tokens.update(QUOTED_NAME.findall(text))


def collect_tokens(checkout, ref):
    tokens = set()
    if os.path.isdir(checkout):
        print(f'Scanning local checkout {checkout}')
        for root in ('ArduPlane', 'libraries'):
            for path in glob.glob(
                    os.path.join(checkout, root, '**', '*.cpp'),
                    recursive=True):
                try:
                    with open(path, errors='ignore') as fh:
                        collect_tokens_from_text(fh.read(), tokens)
                except OSError:
                    pass
        return tokens
    print(f'No local checkout; fetching {len(FETCH_FILES)} files at {ref}')
    for path in FETCH_FILES:
        url = RAW_URL.format(ref=ref, path=path)
        try:
            with urllib.request.urlopen(url, timeout=30) as response:
                collect_tokens_from_text(
                    response.read().decode(errors='ignore'), tokens)
        except OSError as exc:
            print(f'  warning: could not fetch {path}: {exc}')
    if not tokens:
        raise SystemExit('No parameter tokens collected — network down?')
    return tokens


def param_names(path):
    names = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            names.append(line.split()[0])
    return names


def name_matches(name, tokens):
    if name in tokens:
        return True
    parts = name.split('_')
    for i in range(1, len(parts)):
        suffix = '_'.join(parts[i:])
        if len(suffix) >= 3 and suffix in tokens:
            return True
    return False


def main():
    parser = argparse.ArgumentParser(
        description='Check ArduPilot param names against the pinned source.')
    parser.add_argument('params', nargs='*', help='.params files to check')
    parser.add_argument('--ardupilot-dir', default=DEFAULT_CHECKOUT)
    parser.add_argument('--env-file', default=DEFAULT_ENV_FILE)
    parser.add_argument('--ref', default=None)
    args = parser.parse_args()

    files = args.params or sorted(
        glob.glob(os.path.join(REPO, 'systems/*/config/*.params')))
    if not files:
        raise SystemExit('No .params files found')

    ref = args.ref or read_ref(args.env_file)
    tokens = collect_tokens(args.ardupilot_dir, ref)

    failures = []
    for path in files:
        for name in param_names(path):
            if not name_matches(name, tokens):
                failures.append(f'{os.path.relpath(path, REPO)}: {name}')

    if failures:
        print(f'\nUnknown parameters for ArduPilot {ref} '
              '(renamed or misspelled — ArduPilot will silently drop them):')
        for failure in failures:
            print(f'  {failure}')
        return 1
    print(f'All parameter names in {len(files)} file(s) '
          f'resolve against ArduPilot {ref}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
