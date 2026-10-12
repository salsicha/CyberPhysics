"""Deployment regressions: tags must reach FROM, and CPU fallback is a failure.

Real ARM64 package installation is covered by images.yml's native builds;
GPU kernel execution is tools/jetson_gpu_smoke.sh on the deployment Jetson.
"""
import json
import os
import shutil
import subprocess
import sys
from types import SimpleNamespace

import pytest

from conftest import REPO, load_script


@pytest.mark.parametrize('tag', ['latest', 'arm64'])
@pytest.mark.parametrize('composition', [
    'racecarneo_hardware', 'racecar_turret_hardware', 'turret_hardware',
    'pantiltros', 'turret_sim',
])
def test_compose_builds_use_the_selected_base_architecture(tag, composition):
    if not shutil.which('docker'):
        pytest.skip('Docker Compose is not installed')
    version = subprocess.run(['docker', 'compose', 'version'], capture_output=True)
    if version.returncode:
        pytest.skip('Docker Compose is not installed')
    result = subprocess.run([
        'docker', 'compose', '-f', str(REPO / 'compositions' / f'{composition}.yaml'),
        '--profile', '*', 'config', '--format', 'json',
    ], env={**os.environ, 'CYBERPHYSICS_TAG': tag}, text=True, capture_output=True, check=True)
    for name, service in json.loads(result.stdout)['services'].items():
        if service['image'].startswith('salsicha/cyberphysics:'):
            assert service['image'].endswith('-' + tag), name
        if 'build' not in service:
            continue
        build = service['build']
        dockerfile = (REPO / build['context'] / build.get('dockerfile', 'Dockerfile')).read_text()
        if '${TAG}' in dockerfile:
            assert build.get('args', {}).get('TAG') == tag, name


@pytest.mark.parametrize('platform,tag', [('amd64', 'latest'), ('arm64', 'arm64')])
def test_make_build_path_passes_platform_and_base_tag(tmp_path, platform, tag):
    # Exercise the real wrapper while replacing Docker with an argument recorder.
    docker = tmp_path / 'docker'
    docker.write_text('#!/bin/sh\nprintf "%s\\n" "$@" > "$BUILD_ARGS_FILE"\n')
    docker.chmod(0o755)
    output = tmp_path / 'args'
    subprocess.run(['bash', str(REPO / 'applications/_build_it.sh')],
                   cwd=REPO / 'applications', check=True, capture_output=True,
                   env={**os.environ, 'PATH': f'{tmp_path}:{os.environ["PATH"]}',
                        'PLATFORM': platform, 'APP': 'nvblox', 'REPOSITORY': 'test/images',
                        'TARGET': '', 'DOCKERFILE': '', 'BUILD_STAGE': '', 'NOCACHE': '0',
                        'BUILD_ARGS_FILE': str(output)})
    args = output.read_text().splitlines()
    assert args[args.index('--platform') + 1] == f'linux/{platform}'
    assert args[args.index('--build-arg') + 1] == f'TAG={tag}'
    assert args[args.index('-t') + 1] == f'test/images:nvblox-{tag}'


@pytest.mark.parametrize('cuda_version,available,error', [
    (None, False, 'CPU-only'), ('13.0', False, 'CUDA is unavailable'),
])
def test_smoke_check_rejects_cpu_fallback(monkeypatch, cuda_version, available, error):
    module = load_script(REPO / 'tools/jetson_gpu_smoke.py', 'jetson_gpu_smoke_test')
    monkeypatch.setitem(sys.modules, 'torch', SimpleNamespace(
        version=SimpleNamespace(cuda=cuda_version),
        cuda=SimpleNamespace(is_available=lambda: available),
    ))
    monkeypatch.setitem(sys.modules, 'torchvision', SimpleNamespace())
    with pytest.raises(AssertionError, match=error):
        module.check_torch('pantiltros')
