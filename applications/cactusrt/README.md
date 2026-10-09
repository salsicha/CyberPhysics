
# Cactus-RT

This app serves as a demonstration for real time control on Linux with
[cactus-rt](https://github.com/cactusdynamics/cactus-rt), a C++ framework in
which a control loop is the `Loop()` method of a `CyclicThread`.

The image builds the library at a pinned commit (`CACTUS_RT_COMMIT` in the
Dockerfile; upstream has no release tags) and two of its bundled examples:

- `rt_simple_example` (the default command): one 1 kHz loop on a `SCHED_FIFO`
  thread at priority 80, pinned to CPU 2. Before starting it, `cactus_rt::App`
  locks all of the process's memory (`mlockall`) so the loop never page-faults.
  It logs through Quill, an asynchronous logger that is safe to call from the
  real-time thread. It runs until SIGINT/SIGTERM, then stops and joins the
  thread.
- `rt_tracing_example`: two real-time threads running pretend sense/plan/act
  work, traced for 15 s into Perfetto files.

## Run

Build the image (`cd applications && make build_cactusrt`), then from the
repository root:

```bash
./start.sh cactusrt                          # 30 s run, then a clean stop
CACTUSRT_DURATION=0 ./start.sh cactusrt      # run until Ctrl-C
```

or without compose:

```bash
docker run --rm --cap-add SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 \
  salsicha/cyberphysics:cactusrt-latest
```

The loop needs these privileges. `SYS_NICE` and `rtprio` allow `SCHED_FIFO`
priorities, and an unlimited `memlock` allows `mlockall`. Without them the
entrypoint prints a warning. The example then aborts with
`mlockall failed: Cannot allocate memory` or
`failed to sched_setattr: Operation not permitted`.

## Reading the output

```text
App started
[...][I][ExampleRTThread][main.cc:39] Looping for 3697801ns
[...][I][ExampleRTThread][main.cc:39] Looping for 1003701563ns
...
Caught signal, requesting stop...
Done
```

- The loop runs 1000 times a second. To keep the log readable it logs only once
  a second (`LOG_INFO_LIMIT`). The value is the time since `App::Start`.
- `[W] ... At least 1 loop overrun detected` means that wakeup latency plus
  loop time was longer than the 1 ms period, so a deadline was missed. Both
  values are given in ns. The wakeup latency is how late the kernel woke the
  thread.
- `Caught signal, requesting stop...` followed by `Done` means the thread was
  stopped and joined. The container then exits with status 0.

To check the scheduling while it runs, list the threads.
`ExampleRTThread` should show `policy=1` (`SCHED_FIFO`) and `rtprio=80`:

```bash
docker exec cactusrt bash -c 'p=$(pgrep -x rt_simple_examp); for t in /proc/$p/task/*; do
  echo "$(cat $t/comm) policy=$(cut -d" " -f41 $t/stat) rtprio=$(cut -d" " -f40 $t/stat)"; done'
```

## Latency traces

```bash
./start.sh cactusrt --profile tracing cactusrt_tracing
```

This runs `rt_tracing_example` for 15 s. Every 1.5 s it adds a deliberate
1.2 ms "RogueSegment", so the overrun warnings in its log are expected. It
writes `generated/cactusrt/data1.perfetto` and `data2.perfetto` (owned by root).
Open them in https://ui.perfetto.dev, which runs entirely in the browser.
Each `Loop` span is one iteration, and `LoopOverrun` marks a missed deadline.

## Lower latency

The latency you see depends on the host. A `PREEMPT_RT` kernel is best.
Stock kernels give higher latency (Ubuntu's `PREEMPT_DYNAMIC` kernel can be
booted with `preempt=full`). Load on other cores and power management add
latency too.

To reserve the loop's core (as in the top-level README's real-time notes),
shield CPU 2 on the host. `simple_example` pins its thread to CPU 2, so the
host needs at least 3 CPUs and any `cpuset` must include CPU 2.

```bash
sudo cset shield --cpu 2 --kthread on   # move other tasks and kernel threads off CPU 2
# then uncomment `cpuset: "2"` in compositions/cactusrt.yaml, or:
docker run --rm --cpuset-cpus=2 --cap-add SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 \
  salsicha/cyberphysics:cactusrt-latest
sudo cset shield --reset                # restore the host afterwards
```

The source and build tree are in `/cactus-rt` inside the image. You can change
an example there and rebuild it with
`cmake --build /cactus-rt/build/release --target rt_simple_example`.
