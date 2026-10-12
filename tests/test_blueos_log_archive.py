import gzip
import importlib.util
import os
from pathlib import Path

spec = importlib.util.spec_from_file_location('blueos_archive', Path(__file__).parents[1] / 'tools/blueos_log_archive.py')
archive = importlib.util.module_from_spec(spec)
spec.loader.exec_module(archive)


def old_log(path):
    path.write_bytes(b'recorded telemetry\n' * 10000)
    os.utime(path, (1, 1))
    return path


def test_closed_log_compression_preserves_data_and_is_idempotent(tmp_path):
    path = old_log(tmp_path / 'old.tlog')
    original = path.read_bytes()
    assert archive.archive(tmp_path, opened=lambda: set(), now=lambda: 100000) > 0
    assert not path.exists()
    with gzip.open(str(path) + '.gz', 'rb') as stream:
        assert stream.read() == original
    assert archive.archive(tmp_path, opened=lambda: set(), now=lambda: 100000) == 0


def test_open_recent_and_symlink_logs_are_untouched(tmp_path):
    busy = old_log(tmp_path / 'busy.tlog')
    recent = tmp_path / 'recent.bin'
    recent.write_bytes(b'new recording')
    (tmp_path / 'alias.tlog').symlink_to(busy)
    stat = busy.stat()
    assert archive.archive(tmp_path, opened=lambda: {(stat.st_dev, stat.st_ino)}) == 0
    assert busy.exists() and recent.exists()
    assert not list(tmp_path.glob('*.gz'))


def test_log_opened_during_compression_is_not_removed(tmp_path):
    path = old_log(tmp_path / 'old.tlog')
    stat = path.stat()
    calls = iter([set(), {(stat.st_dev, stat.st_ino)}])
    assert archive.archive(tmp_path, opened=lambda: next(calls), now=lambda: 100000) == 0
    assert path.exists() and not list(tmp_path.glob('*.gz'))
