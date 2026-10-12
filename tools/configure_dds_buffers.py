#!/usr/bin/env python3
"""Persist the host socket-buffer limits needed by the camera DDS streams.

Run with sudo to configure; --check is read-only and needs no privileges.
These are host settings: the robot containers use network_mode: host.
"""
import argparse
import os
from pathlib import Path
import subprocess
import shutil
import tempfile

MIN_BUFFER_BYTES = 4 * 1024 * 1024
KEYS = ('net.core.rmem_max', 'net.core.wmem_max')
CONFIG = Path('/etc/sysctl.d/99-cyberphysics-dds.conf')


def sysctl():
    # Raspberry Pi OS omits sbin from the non-login SSH user's PATH.
    return 'sysctl' if shutil.which('sysctl') else '/usr/sbin/sysctl'


def read_limits():
    values = {}
    for key in KEYS:
        result = subprocess.run([sysctl(), '-n', key], check=True,
                                capture_output=True, text=True)
        values[key] = int(result.stdout.strip())
    return values


def check_limits():
    values = read_limits()
    for key, value in values.items():
        if value < MIN_BUFFER_BYTES:
            raise RuntimeError(
                f'{key}={value}; camera DDS requires at least {MIN_BUFFER_BYTES} bytes. '
                'Run sudo python3 tools/configure_dds_buffers.py on the host.')
    return values


def configure(config=CONFIG):
    # A host may already allow larger buffers. Preserve those limits both
    # now and in our boot configuration instead of reducing another app's allowance.
    values = {key: max(value, MIN_BUFFER_BYTES) for key, value in read_limits().items()}
    content = '# CyberPhysics camera DDS: minimum 4 MiB send and receive buffers.\n'
    content += ''.join(f'{key} = {value}\n' for key, value in values.items())
    config.parent.mkdir(parents=True, exist_ok=True)
    # Replace our own file atomically; never leave a partial boot configuration.
    with tempfile.NamedTemporaryFile(mode='w', dir=config.parent, delete=False) as handle:
        temporary = Path(handle.name)
        try:
            handle.write(content)
            handle.flush()
            os.chmod(temporary, 0o644)
            os.replace(temporary, config)
        finally:
            temporary.unlink(missing_ok=True)
    subprocess.run([sysctl(), '--load', str(config)], check=True)
    return check_limits()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='check live limits without changing them')
    args = parser.parse_args()
    if not args.check and os.geteuid() != 0:
        parser.exit(1, 'Configuration requires sudo; --check does not.\n')
    try:
        values = check_limits() if args.check else configure()
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as exc:
        parser.exit(1, f'DDS host setup failed: {exc}\n')
    print('DDS socket limits ready: ' + ', '.join(f'{key}={value}' for key, value in values.items()))


if __name__ == '__main__':
    main()
