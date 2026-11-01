#!/usr/bin/env python3
"""Resource sampler for the companion computer in hardware-in-the-loop runs
(tools/aerodrone_hil_test.md): runs on the Pi's host (Python standard
library only, reads /proc, /sys and `docker ps`; changes nothing) and writes
one JSON line per sample:

  {"t": wall s, "cpu_pct": host busy % of all cores, "cores": n,
   "load1": ..., "mem_avail_mib": ..., "mem_used_mib": ..., "temp_c": ...,
   "freq_mhz": [per core, cpufreq], "throttled": "0x0", "arm_mhz": firmware clock,
   "containers": {name: {"cpu_pct": % of one core, "mem_mib": ...}},
   "processes": {name: {"cpu_pct": % of one core}},
   "net": {interface: {"rx_Bps": ..., "tx_Bps": ...}},
   "links": {"host:port": {"rx_Bps": ..., "tx_Bps": ...}}}

Containers are every running container (BlueOS's included: its ArduPilot
runs inside blueos-core); processes are those whose /proc comm starts with
one of --process (default ArduPilot's "ardupilot_navig"); net, the bytes of
every interface but loopback; links, the payload bytes of the TCP sessions
to each --tcp-peer (`ss -ti`: bytes_received, and bytes_acked sent).

  python3 tools/aerodrone_hil_resources.py --output resources.jsonl --period-s 5
  python3 tools/aerodrone_hil_resources.py summary resources.jsonl
"""
import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time

CGROUP = '/sys/fs/cgroup/system.slice/docker-{}.scope'
TICKS = os.sysconf('SC_CLK_TCK') if hasattr(os, 'sysconf') else 100


def read(path, default=None):
    try:
        with open(path, encoding='utf-8') as stream:
            return stream.read()
    except OSError:
        return default


def host_cpu():
    """(busy, total) jiffies of all cores."""
    fields = [int(v) for v in read('/proc/stat').split('\n', 1)[0].split()[1:]]
    idle = fields[3] + (fields[4] if len(fields) > 4 else 0)
    return sum(fields) - idle, sum(fields)


def cpu_cores():
    return sum(1 for line in read('/proc/stat').splitlines()
               if line.startswith('cpu') and line[3:4].isdigit())


def meminfo():
    values = {}
    for line in read('/proc/meminfo', '').splitlines():
        name, _, rest = line.partition(':')
        values[name] = int(rest.split()[0]) if rest.split() else 0
    return values


def containers():
    """{name: full id} of the running containers."""
    try:
        out = subprocess.run(['docker', 'ps', '--no-trunc', '--format', '{{.ID}} {{.Names}}'],
                             capture_output=True, text=True, timeout=20, check=True).stdout
    except (OSError, subprocess.SubprocessError):
        return {}
    pairs = (line.split(' ', 1) for line in out.splitlines() if ' ' in line)
    return {name: cid for cid, name in pairs}


def container_usage(cid):
    """(cpu µs, memory bytes) of a container's cgroup, or None."""
    base = CGROUP.format(cid)
    stat = read(os.path.join(base, 'cpu.stat'))
    if stat is None:
        return None
    usage = next((int(line.split()[1]) for line in stat.splitlines()
                  if line.startswith('usage_usec')), 0)
    memory = int((read(os.path.join(base, 'memory.current'), '0') or '0').strip() or 0)
    return usage, memory


def processes(prefixes):
    """{comm: cpu jiffies} summed over the processes whose comm starts with
    one of prefixes."""
    found = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        comm = (read(f'/proc/{pid}/comm', '') or '').strip()
        if not any(comm.startswith(p) for p in prefixes):
            continue
        stat = read(f'/proc/{pid}/stat')
        if stat is None:
            continue
        fields = stat.rsplit(')', 1)[1].split()
        found[comm] = found.get(comm, 0) + int(fields[11]) + int(fields[12])
    return found


def interfaces():
    """{interface: (rx bytes, tx bytes)}, loopback and docker bridges left out."""
    found = {}
    for name in sorted(os.listdir('/sys/class/net')):
        if name == 'lo' or name.startswith(('docker', 'veth', 'br-')):
            continue
        rx = read(f'/sys/class/net/{name}/statistics/rx_bytes')
        tx = read(f'/sys/class/net/{name}/statistics/tx_bytes')
        if rx is not None and tx is not None:
            found[name] = (int(rx), int(tx))
    return found


def tcp_bytes(peer):
    """(received, sent) payload bytes over the TCP sessions to peer
    (host:port), or None without ss."""
    try:
        out = subprocess.run(['ss', '-tinH', 'dst', peer], capture_output=True, text=True,
                             timeout=5, check=True).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    received = sum(int(v) for v in re.findall(r'bytes_received:(\d+)', out))
    sent = sum(int(v) for v in re.findall(r'bytes_acked:(\d+)', out))
    return received, sent


def throttled():
    try:
        out = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True,
                             timeout=5, check=True).stdout.strip()
        return out.split('=', 1)[1]
    except (OSError, subprocess.SubprocessError, IndexError):
        return None


def arm_clock_mhz():
    """The ARM clock the firmware runs (it throttles below what cpufreq
    reports), or None without vcgencmd."""
    try:
        out = subprocess.run(['vcgencmd', 'measure_clock', 'arm'], capture_output=True,
                             text=True, timeout=5, check=True).stdout.strip()
        return int(out.split('=', 1)[1]) // 1_000_000
    except (OSError, subprocess.SubprocessError, IndexError, ValueError):
        return None


def temperature_c():
    value = read('/sys/class/thermal/thermal_zone0/temp')
    return None if value is None else round(int(value) / 1000.0, 1)


def frequencies_mhz(cores):
    return [int((read(f'/sys/devices/system/cpu/cpu{i}/cpufreq/scaling_cur_freq', '0')
                 or '0').strip() or 0) // 1000 for i in range(cores)]


def sample_loop(args):
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(True))
    signal.signal(signal.SIGINT, lambda *_: stop.append(True))
    cores = cpu_cores()
    previous = None
    with open(args.output, 'a', encoding='utf-8') as out:
        while not stop:
            now = time.time()
            names = containers()
            usage = {name: container_usage(cid) for name, cid in names.items()}
            current = {'t': now, 'host': host_cpu(), 'usage': usage,
                       'procs': processes(args.process), 'net': interfaces(),
                       'links': {peer: tcp_bytes(peer) for peer in args.tcp_peer}}
            if previous is not None:
                dt = now - previous['t']
                busy = current['host'][0] - previous['host'][0]
                total = current['host'][1] - previous['host'][1]
                mem = meminfo()
                row = {'t': round(now, 3), 'cores': cores,
                       'cpu_pct': round(100.0 * busy / total, 1) if total else None,
                       'load1': float(read('/proc/loadavg', '0').split()[0]),
                       'mem_avail_mib': mem.get('MemAvailable', 0) // 1024,
                       'mem_used_mib': (mem.get('MemTotal', 0) - mem.get('MemAvailable', 0)) // 1024,
                       'temp_c': temperature_c(), 'freq_mhz': frequencies_mhz(cores),
                       'throttled': throttled(), 'arm_mhz': arm_clock_mhz(),
                       'containers': {}, 'processes': {},
                       'net': {}, 'links': {}}
                for key, values in (('net', current['net']), ('links', current['links'])):
                    for name, now_bytes in values.items():
                        before = previous[key].get(name)
                        if now_bytes is None or before is None:
                            continue
                        rx, tx = now_bytes[0] - before[0], now_bytes[1] - before[1]
                        if rx >= 0 and tx >= 0:  # not a new session
                            row[key][name] = {'rx_Bps': round(rx / dt), 'tx_Bps': round(tx / dt)}
                for name, now_usage in usage.items():
                    before = previous['usage'].get(name)
                    if now_usage is None or before is None:
                        continue
                    row['containers'][name] = {
                        'cpu_pct': round((now_usage[0] - before[0]) / 1e4 / dt, 1),
                        'mem_mib': round(now_usage[1] / 2 ** 20, 1)}
                for comm, jiffies in current['procs'].items():
                    if comm in previous['procs']:
                        row['processes'][comm] = {'cpu_pct': round(
                            100.0 * (jiffies - previous['procs'][comm]) / TICKS / dt, 1)}
                out.write(json.dumps(row, separators=(',', ':')) + '\n')
                out.flush()
            previous = current
            deadline = time.time() + args.period_s
            while not stop and time.time() < deadline:
                time.sleep(0.2)


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    position = (len(values) - 1) * fraction
    low = int(position)
    high = min(low + 1, len(values) - 1)
    return values[low] + (values[high] - values[low]) * (position - low)


def summarize(rows, start=None, end=None):
    """Median / p95 / max of the host and per container over [start, end)."""
    rows = [r for r in rows if (start is None or r['t'] >= start) and (end is None or r['t'] < end)]
    if not rows:
        return {}

    def stats(values):
        values = [v for v in values if v is not None]
        if not values:
            return None
        return {'median': round(percentile(values, 0.5), 1), 'p95': round(percentile(values, 0.95), 1),
                'max': round(max(values), 1)}

    flags, now_throttled = 0, 0
    for row in rows:
        try:
            value = int(row.get('throttled') or '0', 16)
        except ValueError:
            continue
        flags |= value
        # Bits 0-3: under-voltage, frequency capped, throttled, soft
        # temperature limit, now (16-19: since boot).
        now_throttled += bool(value & 0xE)
    names = sorted({name for row in rows for name in row['containers']})
    procs = sorted({name for row in rows for name in row['processes']})
    result = {
        'samples': len(rows), 'seconds': round(rows[-1]['t'] - rows[0]['t'], 1),
        'cores': rows[0]['cores'],
        'host_cpu_pct': stats([r['cpu_pct'] for r in rows]),
        'load1': stats([r['load1'] for r in rows]),
        'mem_used_mib': stats([r['mem_used_mib'] for r in rows]),
        'mem_avail_mib_min': min(r['mem_avail_mib'] for r in rows),
        'temp_c': stats([r['temp_c'] for r in rows]),
        'freq_mhz_min': min(min(r['freq_mhz']) for r in rows if r['freq_mhz']),
        'throttled_or': hex(flags),
        'throttled_now_share': round(now_throttled / len(rows), 3),
        'freq_mhz_median': percentile([min(r['freq_mhz']) for r in rows if r['freq_mhz']], 0.5),
        'arm_mhz': stats([r.get('arm_mhz') for r in rows]),
        'arm_mhz_min': min((r['arm_mhz'] for r in rows if r.get('arm_mhz')), default=None),
        'containers': {name: {'cpu_pct': stats([r['containers'].get(name, {}).get('cpu_pct')
                                                 for r in rows]),
                              'mem_mib': stats([r['containers'].get(name, {}).get('mem_mib')
                                                for r in rows])}
                       for name in names},
        'processes': {name: {'cpu_pct': stats([r['processes'].get(name, {}).get('cpu_pct')
                                                for r in rows])} for name in procs},
    }
    for key in ('net', 'links'):
        names = sorted({name for row in rows for name in row.get(key, {})})
        result[key] = {name: {direction: stats([row.get(key, {}).get(name, {}).get(direction)
                                                for row in rows])
                              for direction in ('rx_Bps', 'tx_Bps')} for name in names}
    return result


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv and argv[0] == 'summary':
        parser = argparse.ArgumentParser(prog='aerodrone_hil_resources.py summary')
        parser.add_argument('path')
        parser.add_argument('--start', type=float)
        parser.add_argument('--end', type=float)
        args = parser.parse_args(argv[1:])
        with open(args.path, encoding='utf-8') as stream:
            rows = [json.loads(line) for line in stream if line.strip()]
        print(json.dumps(summarize(rows, args.start, args.end), indent=1))
        return 0
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--output', required=True)
    parser.add_argument('--period-s', type=float, default=5.0)
    parser.add_argument('--process', nargs='*', default=['ardupilot_navig'])
    parser.add_argument('--tcp-peer', nargs='*', default=[],
                        help='host:port of TCP sessions to count (the HIL links)')
    sample_loop(parser.parse_args(argv))
    return 0


if __name__ == '__main__':
    sys.exit(main())
