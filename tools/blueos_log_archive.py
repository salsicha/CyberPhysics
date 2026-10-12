#!/usr/bin/env python3
"""Losslessly compress closed BlueOS flight/telemetry logs older than a day.

Never truncate a live recording or delete an archive. Run hourly with the
supplied systemd timer. Gzip archives can be restored with gunzip before
loading them in a flight-log viewer. Active recordings remain untouched.
"""
import argparse
import fcntl
import gzip
import hashlib
import os
from pathlib import Path
import shutil
import tempfile
import time


def open_inodes():
    opened = set()
    for process in Path('/proc').glob('[0-9]*'):
        try:
            for descriptor in (process / 'fd').iterdir():
                try:
                    stat = descriptor.stat()
                    opened.add((stat.st_dev, stat.st_ino))
                except FileNotFoundError:
                    pass
        except FileNotFoundError:
            pass
        # Permission errors deliberately propagate: cannot prove a log closed.
    return opened


def digest(stream):
    result = hashlib.sha256()
    while block := stream.read(1024 * 1024):
        result.update(block)
    return result.digest()


def archive(directory, minimum_age_s=86400, opened=open_inodes, now=time.time):
    total_saved = 0
    busy = opened()
    for source in sorted(directory.iterdir()):
        if source.is_symlink() or source.suffix not in {'.tlog', '.bin'} or not source.is_file():
            continue
        before = source.stat()
        identity = (before.st_dev, before.st_ino)
        destination = source.with_suffix(source.suffix + '.gz')
        if now() - before.st_mtime < minimum_age_s or identity in busy or destination.exists():
            continue
        handle, temporary = tempfile.mkstemp(prefix='.archive-', dir=directory)
        os.close(handle)
        temporary = Path(temporary)
        try:
            with source.open('rb') as incoming, gzip.open(temporary, 'wb', compresslevel=6) as outgoing:
                shutil.copyfileobj(incoming, outgoing, 1024 * 1024)
            with source.open('rb') as incoming, gzip.open(temporary, 'rb') as compressed:
                if digest(incoming) != digest(compressed):
                    raise RuntimeError(f'archive verification failed: {source}')
            after = source.stat()
            if (before.st_ino, before.st_size, before.st_mtime_ns) != (
                    after.st_ino, after.st_size, after.st_mtime_ns) or identity in opened():
                continue
            if temporary.stat().st_size >= before.st_size:
                continue
            os.chmod(temporary, before.st_mode & 0o777)
            os.utime(temporary, ns=(before.st_atime_ns, before.st_mtime_ns))
            with temporary.open('rb') as complete:
                os.fsync(complete.fileno())
            # link fails rather than overwrites if another archive appeared.
            os.link(temporary, destination)
            # Persist the verified archive's directory entry before removing
            # the original, including across a power loss on the Pi.
            descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
            source.unlink()
            saved = before.st_size - destination.stat().st_size
            total_saved += saved
            print(f'{source.name}: archived; saved {saved} bytes', flush=True)
        finally:
            temporary.unlink(missing_ok=True)
    return total_saved


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path,
                        default=Path('/root/.config/blueos/ardupilot-manager/logs'))
    parser.add_argument('--min-age-hours', type=float, default=24)
    args = parser.parse_args()
    if args.min_age_hours < 1:
        parser.error('--min-age-hours must be at least 1')
    with (args.directory / '.archive.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        saved = archive(args.directory, args.min_age_hours * 3600)
        print(f'Saved {saved} bytes; live logs and all archived data preserved.')


if __name__ == '__main__':
    main()
