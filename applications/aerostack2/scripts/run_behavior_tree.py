#!/usr/bin/env python3
"""Run the Aerostack2 mission tree; start it again only if it never started.

as2_behavior_tree builds every node of the tree before the first tick, and
each action node (TakeOff, GoTo, Land) gives its action server a hard-coded
1 s to appear (BtActionNode::createActionClient in as2_behavior_tree 1.1.3;
server_timeout does not apply). A freshly started process first has to
discover the servers' endpoints over DDS, which in a loaded simulator can
take longer than that even when the servers have been up for minutes. The
node then aborts before its first tick (the exception it throws is built
with a printf-style std::string constructor and surfaces as
std::length_error), so nothing has been commanded yet, and ros2 launch
still exits 0.

This wrapper runs the command after `--` (ros2 launch of the tree), echoes
its output, and starts it again, a bounded number of times, only when that
output shows the construction-time failure and no tree node ever left IDLE.
A tree that ticked is never started again. When the tree's process died
(the tree that never started included), or the tree ended in FAILURE
(as2_behavior_tree itself still exits 0 then), the wrapper exits non-zero.
SIGINT and SIGTERM are passed on to the command and end the run.
"""

from __future__ import annotations

import argparse
import re
import signal
import subprocess
import sys
import time

ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*m")
# BT::StdCoutLogger transition of any node out of IDLE: the tree ticked.
TICKED = re.compile(r"\bIDLE\s*->\s*(RUNNING|SUCCESS|FAILURE)\b")
# BtActionNode / service node waits that only run while the tree is built.
CONSTRUCTION_FAILURE = re.compile(r"(action|service) server not available after waiting")
# A node finishing. Once the root has finished the tree only halts nodes
# back to IDLE, so the last of these is the tree's result.
FINISHED = re.compile(r"\b(?:IDLE|RUNNING)\s*->\s*(SUCCESS|FAILURE)\b")
# ros2 launch, for a process that exited non-zero or on a signal.
PROCESS_DIED = re.compile(r"process has died \[pid \d+, exit code")


class Attempt:
    """What one run of the tree command showed."""

    def __init__(self):
        self.ticked = False
        self.construction_failed = False
        self.process_died = False
        self.result = None  # the tree's result, once it finished
        self.returncode = None

    def observe(self, line):
        text = ANSI_ESCAPE.sub("", line)
        self.ticked = self.ticked or bool(TICKED.search(text))
        self.construction_failed = (
            self.construction_failed or bool(CONSTRUCTION_FAILURE.search(text)))
        self.process_died = self.process_died or bool(PROCESS_DIED.search(text))
        finished = FINISHED.search(text)
        if finished:
            self.result = finished.group(1)

    @property
    def never_started(self):
        return self.construction_failed and not self.ticked


def exit_status(returncode):
    """A shell-style exit status for a Popen return code (-N: signal N)."""
    return 128 - returncode if returncode < 0 else returncode


class TreeRunner:
    def __init__(self, command, attempts, retry_delay,
                 popen=subprocess.Popen, clock=time.monotonic, sleep=time.sleep,
                 out=sys.stdout):
        self.command = command
        self.attempts = attempts
        self.retry_delay = retry_delay
        self.popen = popen
        self.clock = clock
        self.sleep = sleep
        self.out = out
        self.child = None
        self.stopping = False

    def log(self, message):
        self.out.write(f"[run_behavior_tree] {message}\n")
        self.out.flush()

    def stop(self, signum, _frame=None):
        self.stopping = True
        if self.child is not None and self.child.poll() is None:
            self.child.send_signal(signum)

    def run_once(self):
        attempt = Attempt()
        # Its own session: a terminal's Ctrl-C reaches only this wrapper,
        # which passes it on once.
        self.child = self.popen(
            self.command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, errors="replace", bufsize=1, start_new_session=True)
        if self.stopping:  # a stop signal that came while it was starting
            self.child.send_signal(signal.SIGINT)
        for line in self.child.stdout:
            self.out.write(line)
            self.out.flush()
            attempt.observe(line)
        attempt.returncode = self.child.wait()
        return attempt

    def wait_before_retry(self):
        until = self.clock() + self.retry_delay
        while not self.stopping and self.clock() < until:
            self.sleep(min(0.2, max(0.0, until - self.clock())))

    def run(self):
        for number in range(1, self.attempts + 1):
            attempt = self.run_once()
            if self.stopping:
                return exit_status(attempt.returncode)
            if not attempt.never_started:
                if attempt.process_died:
                    self.log("the behaviour tree process died")
                    return exit_status(attempt.returncode) or 1
                if attempt.result == "FAILURE":
                    self.log("the mission tree ended in FAILURE")
                    return exit_status(attempt.returncode) or 1
                return exit_status(attempt.returncode)
            if number < self.attempts:
                self.log(
                    f"the tree aborted while it was being built, before its first "
                    f"tick (attempt {number}/{self.attempts}): starting it again "
                    f"in {self.retry_delay:g} s")
                self.wait_before_retry()
                if self.stopping:
                    return exit_status(attempt.returncode)
        self.log(f"the tree could not be built in {self.attempts} attempts")
        return 1


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--attempts", type=int, default=5)
    parser.add_argument("--retry-delay", type=float, default=5.0)
    parser.add_argument("command", nargs=argparse.REMAINDER,
                        help="-- followed by the command that runs the tree")
    args = parser.parse_args(argv)
    if args.command[:1] == ["--"]:
        args.command = args.command[1:]
    if not args.command or args.attempts < 1:
        parser.error("needs --attempts >= 1 and a command after --")
    return args


def main(argv=None):
    args = parse_args(argv)
    runner = TreeRunner(args.command, args.attempts, args.retry_delay)
    signal.signal(signal.SIGINT, runner.stop)
    signal.signal(signal.SIGTERM, runner.stop)
    return runner.run()


if __name__ == "__main__":
    sys.exit(main())
