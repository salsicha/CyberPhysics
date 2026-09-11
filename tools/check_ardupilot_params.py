#!/usr/bin/env python3
"""Validate parameter names using ArduPilot's pinned, per-vehicle metadata.

Requires git and lxml. Use --ardupilot-dir for an existing checkout at --ref
(default: the same pin as the SITL image). Otherwise a temporary sparse checkout
is downloaded. The upstream metadata generator resolves full group prefixes and
vehicle-specific parameters; arbitrary suffix matches are not accepted.
"""
import argparse
from collections import defaultdict
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

REPO = Path(__file__).resolve().parents[1]
DEFAULT_ENV_FILE = REPO / 'systems/airplane/config/sitl.env'
DEFAULT_CHECKOUT = REPO / 'cache/ardupilot'
VEHICLES = {
    'ardupilot_plane.params': 'ArduPlane',
    'arducopter.params': 'ArduCopter',
    'ardurover.params': 'Rover',
    'ardusub.params': 'ArduSub',
}
METADATA_DIR = 'Tools/autotest/param_metadata'


def read_ref(env_file):
    with open(env_file) as stream:
        for line in stream:
            if line.startswith('ARDUPILOT_REF='):
                return line.split('=', 1)[1].strip()
    raise ValueError(f'ARDUPILOT_REF not found in {env_file}')


def param_names(path):
    names = []
    with open(path) as stream:
        for number, line in enumerate(stream, 1):
            line = line.split('#', 1)[0].strip()
            if not line:
                continue
            fields = re.split(r'[,\s]+', line)
            if len(fields) != 2 or not re.fullmatch(r'[A-Z][A-Z0-9_]*', fields[0]) or not fields[1]:
                raise ValueError(f'{path}:{number}: expected NAME,VALUE or NAME VALUE')
            names.append(fields[0])
    return names


def vehicle_for(path, override=None):
    vehicle = override or VEHICLES.get(Path(path).name)
    if vehicle is None:
        raise ValueError(f'Unknown vehicle for {path}; specify --vehicle')
    return vehicle


def verify_checkout(checkout, ref):
    def revision(value):
        return subprocess.check_output(
            ['git', '-C', str(checkout), 'rev-parse', '--verify', value], text=True).strip()
    if revision('HEAD') != revision(f'{ref}^{{commit}}'):
        raise ValueError(f'{checkout} is not at pinned ref {ref}')


def generate_names(checkout, vehicle):
    script = Path(checkout).resolve() / METADATA_DIR / 'param_parse.py'
    with tempfile.TemporaryDirectory(prefix='cyberphysics-params-') as output:
        result = subprocess.run(
            [sys.executable, str(script), '--vehicle', vehicle, '--format', 'json'],
            cwd=output, env={**os.environ, 'PYTHONDONTWRITEBYTECODE': '1'},
            text=True, capture_output=True,
        )
        if result.returncode:
            raise ValueError(f'{vehicle} metadata generation failed:\n{result.stdout}{result.stderr}')
        with open(Path(output) / 'apm.pdef.json') as stream:
            metadata = json.load(stream)
        names = {name for group, values in metadata.items() if group != 'json' for name in values}
        if not names:
            raise ValueError(f'No parameters generated for {vehicle}')
        return names


def validate(checkout, ref, files, vehicle=None):
    verify_checkout(checkout, ref)
    grouped = defaultdict(list)
    for path in files:
        grouped[vehicle_for(path, vehicle)].append(path)
    failures = []
    for firmware, paths in sorted(grouped.items()):
        names = generate_names(checkout, firmware)
        print(f'Checking {firmware} at {ref}: {len(names)} documented parameters')
        for path in paths:
            for name in param_names(path):
                if name not in names:
                    failures.append(f'{path}: {name} is unknown for {firmware}')
    if failures:
        print('\n'.join(failures))
        return 1
    print(f'All parameter names in {len(files)} file(s) resolve against {ref}')
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('params', nargs='*')
    parser.add_argument('--ardupilot-dir', type=Path, default=DEFAULT_CHECKOUT)
    parser.add_argument('--env-file', type=Path, default=DEFAULT_ENV_FILE)
    parser.add_argument('--ref')
    parser.add_argument('--vehicle', choices=sorted(set(VEHICLES.values())))
    args = parser.parse_args()
    files = args.params or sorted(REPO.glob('systems/*/config/*.params'))
    if not files:
        parser.error('No .params files found')
    ref = args.ref or read_ref(args.env_file)
    try:
        if args.ardupilot_dir.exists():
            return validate(args.ardupilot_dir, ref, files, args.vehicle)
        with tempfile.TemporaryDirectory(prefix='cyberphysics-ardupilot-') as temporary:
            checkout = Path(temporary) / 'source'
            subprocess.run([
                'git', 'clone', '--depth', '1', '--filter=blob:none', '--sparse',
                '--config', 'http.lowSpeedLimit=1024', '--config', 'http.lowSpeedTime=60',
                '--branch', ref, 'https://github.com/ArduPilot/ardupilot.git', str(checkout),
            ], check=True)
            # Metadata only needs source annotations. Exclude large firmware,
            # model and test-data binaries stored alongside library sources.
            patterns = [f'/{firmware}/*.{extension}'
                        for firmware in sorted(set(VEHICLES.values()))
                        for extension in ('cpp', 'h')]
            patterns += [f'/libraries/**/*.{extension}' for extension in ('cpp', 'h', 'lua')]
            patterns.append(f'/{METADATA_DIR}/*.py')
            subprocess.run([
                'git', '-C', str(checkout), 'sparse-checkout', 'set', '--no-cone', *patterns,
            ], check=True)
            return validate(checkout, ref, files, args.vehicle)
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        print(f'Parameter validation failed: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
