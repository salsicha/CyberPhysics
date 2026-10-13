"""The Make targets must resolve Dockerfiles after _build_it.sh changes cwd."""
import os
import subprocess

import pytest
from conftest import REPO


@pytest.mark.parametrize('target,base', [
    ('smolvla_workcell', 'smolvla'), ('embedded_zenoh_esp32', 'embedded_zenoh')])
def test_variant_make_target_resolves_real_dockerfile(tmp_path, target, base):
    docker = tmp_path / 'docker'
    docker.write_text('''#!/usr/bin/env python3
import pathlib, sys
args = sys.argv[1:]
assert args[0] == 'build'
assert pathlib.Path(args[args.index('-f') + 1]).is_file(), args
assert args[args.index('-t') + 1].endswith('-latest')
assert pathlib.Path('Dockerfile').is_file()
''')
    docker.chmod(0o755)
    subprocess.run(['make', '-C', str(REPO/'applications'), '-o', f'build_{base}',
                    f'build_{target}'], check=True, capture_output=True, text=True,
                   env={**os.environ, 'PATH': f'{tmp_path}:{os.environ["PATH"]}'})
