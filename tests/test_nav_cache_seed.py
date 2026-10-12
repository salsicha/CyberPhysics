"""DemNav's and WildNav's build-time caches reach the vehicle.

The BlueOS stacks mount a host directory over /data/demnav_cache and
/data/wildnav_cache (compositions/blueos_nav_core.yaml). The images seeded
those very paths, so on the Raspberry Pi 5 the running containers saw an
empty directory while the images held the DEM and 98 WildNav files. The
seeds now live beside the caches and the entrypoints copy them in.
"""
import os
import re
import shutil
import subprocess

import pytest
import yaml

from conftest import REPO

APPS = {'demnav': 'DEMNAV_SEED_DIR', 'wildnav': 'WILDNAV_SEED_DIR'}


def seed_function(app):
    text = (REPO / f'applications/{app}/entrypoint.sh').read_text()
    match = re.search(r'^seed_cache\(\) \{\n.*?^\}\n', text, re.S | re.M)
    assert match, f'{app}: no seed_cache function in the entrypoint'
    return match.group(0)


@pytest.mark.parametrize('app', APPS)
def test_the_image_seeds_beside_the_cache_the_stacks_mount(app):
    var = APPS[app]
    dockerfile = (REPO / f'applications/{app}/Dockerfile').read_text()
    seed_dir = re.search(rf'^ENV {var}=(\S+)$', dockerfile, re.M).group(1)
    assert f'--cache-dir "${var}"' in dockerfile
    assert f'--cache-dir /data/{app}_cache' not in dockerfile
    entrypoint = (REPO / f'applications/{app}/entrypoint.sh').read_text()
    assert f'seed_cache "${{{var}:-{seed_dir}}}" /data/{app}_cache\nexec "$@"' in entrypoint
    service = yaml.safe_load((REPO / 'compositions/blueos_nav_core.yaml').read_text())[
        'services'][app]
    assert any(volume.endswith(f':/data/{app}_cache') for volume in service['volumes'])


@pytest.mark.skipif(not shutil.which('bash'), reason='needs bash')
@pytest.mark.parametrize('app', APPS)
def test_the_entrypoint_copies_only_missing_seed_files(app, tmp_path):
    seed, cache = tmp_path / 'seed', tmp_path / 'cache'
    seed.mkdir()
    (seed / 'a.npy').write_text('seed a')
    (seed / 'b.jpg').write_text('seed b')
    cache.mkdir()
    (cache / 'a.npy').write_text('flown a')
    script = seed_function(app) + f'seed_cache "{seed}" "{cache}"\n'
    run = subprocess.run(['bash', '-ec', script], capture_output=True, text=True, check=True)
    assert (cache / 'a.npy').read_text() == 'flown a'  # never overwritten
    assert (cache / 'b.jpg').read_text() == 'seed b'
    assert f'Seeded {cache} with 1 file(s)' in run.stderr
    # Nothing to copy the next time: quiet.
    again = subprocess.run(['bash', '-ec', script], capture_output=True, text=True, check=True)
    assert again.stderr == ''
    # A cache that does not exist yet is created and seeded.
    fresh = tmp_path / 'fresh'
    subprocess.run(['bash', '-ec', seed_function(app) + f'seed_cache "{seed}" "{fresh}"\n'],
                   check=True, capture_output=True)
    assert sorted(p.name for p in fresh.iterdir()) == ['a.npy', 'b.jpg']
    # No seed (SEED_NAV_ASSETS=0): nothing happens and the entrypoint goes on.
    none = subprocess.run(
        ['bash', '-ec', seed_function(app) + f'seed_cache "{tmp_path}/none" "{cache}"; echo ok\n'],
        capture_output=True, text=True, check=True)
    assert none.stdout == 'ok\n' and none.stderr == ''


@pytest.mark.skipif(not shutil.which('bash'), reason='needs bash')
@pytest.mark.skipif(os.geteuid() == 0, reason='root may write a read-only directory')
@pytest.mark.parametrize('app', APPS)
def test_a_read_only_cache_is_left_alone(app, tmp_path):
    seed, cache = tmp_path / 'seed', tmp_path / 'cache'
    seed.mkdir()
    (seed / 'a.npy').write_text('seed a')
    cache.mkdir()
    cache.chmod(0o555)
    try:
        run = subprocess.run(
            ['bash', '-ec', seed_function(app) + f'seed_cache "{seed}" "{cache}"; echo ok\n'],
            capture_output=True, text=True, check=True)
    finally:
        cache.chmod(0o755)
    assert run.stdout == 'ok\n' and run.stderr == ''
    assert not (cache / 'a.npy').exists()
