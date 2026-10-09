"""Unreal Engine server launcher (run_server.sh) against fake packaged builds:
missing builds, project detection, UE arguments, environment and stop
forwarding. The fake <Project>.sh prints what it was started with."""
import os
import signal
import subprocess

import pytest

from conftest import REPO

LAUNCHER = REPO / 'applications/unrealengine/server/run_server.sh'
DEFAULT_ARGS = ['-RenderOffScreen', '-nosound', '-ResX=1280', '-ResY=720',
                '-unattended', '-stdout', '-FullStdOutLogOutput']

# As root the launcher switches to the image's unrealuser, which a test host lacks
pytestmark = pytest.mark.skipif(os.geteuid() == 0, reason='launcher drops root to unrealuser')

STUB = """#!/bin/sh
for arg in "$@"; do echo "ARG:$arg"; done
echo "CWD:$(pwd)"
echo "LD_LIBRARY_PATH:${LD_LIBRARY_PATH-<unset>}"
echo "AMENT_PREFIX_PATH:${AMENT_PREFIX_PATH-<unset>}"
echo "ROS_DOMAIN_ID:${ROS_DOMAIN_ID-<unset>}"
trap 'echo "GOT:TERM"; exit 0' TERM
sleep "${STUB_SLEEP:-0}" &
echo READY
wait
exit "${STUB_EXIT:-0}"
"""


def make_project(root, name='MyProject', executable=True):
    """A packaged-build layout: <name>.sh next to <name>/Binaries/Linux/."""
    root.mkdir(parents=True, exist_ok=True)
    (root / name / 'Binaries' / 'Linux').mkdir(parents=True)
    script = root / f'{name}.sh'
    script.write_text(STUB)
    script.chmod(0o755 if executable else 0o644)
    return script


def launcher_env(build, **extra):
    env = {key: value for key, value in os.environ.items()
           if not key.startswith('UE_') and key not in ('LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH')}
    env['UE_BUILD_PATH'] = str(build)
    env.update(extra)
    return env


def run(build, *args, **extra):
    return subprocess.run(['bash', str(LAUNCHER), *args], env=launcher_env(build, **extra),
                          capture_output=True, text=True, timeout=30)


def stub_args(result):
    return [line[4:] for line in result.stdout.splitlines() if line.startswith('ARG:')]


def stub_value(result, key):
    [value] = [line[len(key) + 1:] for line in result.stdout.splitlines()
               if line.startswith(key + ':')]
    return value


def test_missing_build_folder_fails_with_instructions(tmp_path):
    result = run(tmp_path / 'nothing')
    assert result.returncode == 1
    assert 'no Unreal Engine build mounted' in result.stderr
    assert 'does not exist' in result.stderr
    assert 'applications/unrealengine/server/build/' in result.stderr
    assert 'UE_BUILD_DIR' in result.stderr


def test_empty_build_folder_with_placeholder_fails_with_instructions(tmp_path):
    (tmp_path / '.gitkeep').touch()
    result = run(tmp_path)
    assert result.returncode == 1
    assert f'{tmp_path} is empty' in result.stderr
    assert 'Package your Unreal Engine project for Linux' in result.stderr


def test_folder_without_a_packaged_project_lists_its_contents(tmp_path):
    (tmp_path / 'notes.sh').write_text('#!/bin/sh\n')  # no notes/Binaries/Linux
    result = run(tmp_path)
    assert result.returncode == 1
    assert 'no packaged UE project found' in result.stderr
    assert 'notes.sh' in result.stderr and 'UE_PROJECT_SCRIPT' in result.stderr
    assert 'ARG:' not in result.stdout


@pytest.mark.parametrize('subdir', ['', 'Linux', 'MyProject_v2/LinuxNoEditor'])
def test_finds_the_project_and_passes_the_default_arguments(tmp_path, subdir):
    script = make_project(tmp_path / subdir)
    (tmp_path / 'Engine').mkdir(exist_ok=True)
    result = run(tmp_path)
    assert result.returncode == 0, result.stderr
    assert stub_args(result) == DEFAULT_ARGS
    assert stub_value(result, 'CWD') == str(script.parent.resolve())
    assert f'starting: {script.resolve()} ' in result.stdout


def test_several_projects_need_ue_project_script(tmp_path):
    make_project(tmp_path / 'A', 'Alpha')
    make_project(tmp_path / 'B', 'Beta')
    result = run(tmp_path)
    assert result.returncode == 1
    assert 'several packaged UE projects' in result.stderr
    assert 'A/Alpha.sh' in result.stderr and 'B/Beta.sh' in result.stderr

    result = run(tmp_path, UE_PROJECT_SCRIPT='B/Beta.sh')
    assert result.returncode == 0, result.stderr
    assert f"starting: {(tmp_path / 'B/Beta.sh').resolve()} " in result.stdout


def test_missing_ue_project_script_fails(tmp_path):
    make_project(tmp_path)
    result = run(tmp_path, UE_PROJECT_SCRIPT='Other.sh')
    assert result.returncode == 1
    assert 'UE_PROJECT_SCRIPT=Other.sh' in result.stderr


def test_environment_and_arguments_build_the_ue_command_line(tmp_path):
    make_project(tmp_path)
    result = run(tmp_path, '-benchmark', '-ExecCmds=stat fps',
                 UE_MAP='/Game/Maps/Warehouse', UE_RES_X='1920', UE_RES_Y='1080',
                 UE_PORT='7777', UE_GRAPHICS_ADAPTER='1', UE_EXTRA_ARGS='-ForceRes  -log')
    assert result.returncode == 0, result.stderr
    assert stub_args(result) == [
        '/Game/Maps/Warehouse', '-RenderOffScreen', '-nosound', '-ResX=1920', '-ResY=1080',
        '-port=7777', '-graphicsadapter=1', '-unattended', '-stdout', '-FullStdOutLogOutput',
        '-ForceRes', '-log', '-benchmark', '-ExecCmds=stat fps']


def test_onscreen_and_sound_switches(tmp_path):
    make_project(tmp_path)
    result = run(tmp_path, UE_RENDER_OFFSCREEN='0', UE_NOSOUND='0')
    args = stub_args(result)
    assert '-windowed' in args
    assert '-RenderOffScreen' not in args and '-nosound' not in args


def test_ros_paths_are_removed_from_the_engine_environment(tmp_path):
    make_project(tmp_path)
    result = run(tmp_path, LD_LIBRARY_PATH='/opt/ros/jazzy/opt/x/lib:/opt/ros/jazzy/lib:/usr/local/cuda/lib64',
                 AMENT_PREFIX_PATH='/opt/ros/jazzy', ROS_DOMAIN_ID='42')
    assert result.returncode == 0, result.stderr
    assert stub_value(result, 'LD_LIBRARY_PATH') == '/usr/local/cuda/lib64'
    assert stub_value(result, 'AMENT_PREFIX_PATH') == '<unset>'
    assert stub_value(result, 'ROS_DOMAIN_ID') == '42'

    result = run(tmp_path, LD_LIBRARY_PATH='/opt/ros/jazzy/lib')
    assert stub_value(result, 'LD_LIBRARY_PATH') == '<unset>'


def test_script_without_executable_bit_still_runs(tmp_path):
    make_project(tmp_path, executable=False)
    result = run(tmp_path)
    assert result.returncode == 0, result.stderr
    assert stub_args(result) == DEFAULT_ARGS


def test_engine_exit_status_is_the_launcher_exit_status(tmp_path):
    make_project(tmp_path)
    result = run(tmp_path, STUB_EXIT='3')
    assert result.returncode == 3
    assert 'Unreal Engine exited with status 3' in result.stdout


@pytest.mark.parametrize('sig', [signal.SIGTERM, signal.SIGINT])
def test_stop_signals_reach_the_engine_as_sigterm(tmp_path, sig):
    make_project(tmp_path)
    proc = subprocess.Popen(['bash', str(LAUNCHER)], env=launcher_env(tmp_path, STUB_SLEEP='30'),
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        for line in proc.stdout:
            if line.strip() == 'READY':
                break
        proc.send_signal(sig)
        output, _ = proc.communicate(timeout=10)
    finally:
        if proc.poll() is None:
            proc.kill()
    assert 'GOT:TERM' in output
    assert 'Unreal Engine exited with status 0' in output
    assert proc.returncode == 0
