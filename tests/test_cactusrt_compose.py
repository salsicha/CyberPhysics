"""cactus-rt demo: the stack grants what its real-time examples need."""
import re

import yaml

from conftest import REPO

STACK = yaml.safe_load((REPO / 'compositions/cactusrt.yaml').read_text())['services']
DOCKERFILE = (REPO / 'applications/cactusrt/Dockerfile').read_text()


def test_every_service_gets_real_time_privileges():
    assert set(STACK) == {'cactusrt', 'cactusrt_tracing'}
    for name, service in STACK.items():
        # mlockall(MCL_CURRENT | MCL_FUTURE) and SCHED_FIFO priority 80
        assert 'SYS_NICE' in service['cap_add'], name
        assert service['ulimits']['memlock'] == -1, name
        assert service['ulimits']['rtprio'] >= 80, name
        assert service['network_mode'] == 'none', name
        # The examples stop their threads and join on SIGINT/SIGTERM
        assert service['stop_signal'] == 'SIGINT', name


def test_timed_run_stops_the_loop_cleanly():
    command = STACK['cactusrt']['command'].split()
    # INT lets rt_simple_example join its thread; --preserve-status reports
    # its exit code (0) instead of timeout's 124
    assert command[:3] == ['timeout', '--preserve-status', '--signal=INT']
    assert command[-1] == 'rt_simple_example'
    assert STACK['cactusrt_tracing']['profiles'] == ['tracing']
    # tracing_example writes build/data{1,2}.perfetto relative to /workspace
    assert any(volume.endswith(':/workspace/build')
               for volume in STACK['cactusrt_tracing']['volumes'])


def test_image_builds_both_examples_from_a_pinned_commit():
    assert re.search(r'^ARG CACTUS_RT_COMMIT=[0-9a-f]{40}$', DOCKERFILE, re.M)
    assert '--target rt_simple_example rt_tracing_example' in DOCKERFILE
    assert 'CMD ["rt_simple_example"]' in DOCKERFILE
