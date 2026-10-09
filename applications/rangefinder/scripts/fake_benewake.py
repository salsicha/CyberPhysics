#!/usr/bin/env python3
"""Simulated Benewake TFmini-S / TF-Luna on a pseudo-terminal.

Streams standard 9-byte frames on a pty so rangefinder_node.py can be
bench-tested without hardware:

    python3 /scripts/fake_benewake.py --link /tmp/tfmini &
    python3 /scripts/rangefinder_node.py --ros-args -p port:=/tmp/tfmini

Distances cycle through --distances (metres); --weak-every and
--garbage-every interleave a no-return frame or line noise.
"""
import argparse
import os
import pty
import random
import time
import tty

from benewake import encode_frame


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--link', default='/tmp/tfmini', help='Symlink to the pty device')
    parser.add_argument('--rate', type=float, default=100.0, help='Frames per second')
    parser.add_argument('--distances', default='1.5,2.0,2.5',
                        help='Comma-separated distances to cycle through [m]')
    parser.add_argument('--strength', type=int, default=1200)
    parser.add_argument('--temperature', type=float, default=35.0, help='Chip temperature [degC]')
    parser.add_argument('--weak-every', type=int, default=0,
                        help='Send a no-return frame (strength 50, dist 65535) every N frames')
    parser.add_argument('--garbage-every', type=int, default=0,
                        help='Send 5 random bytes before every Nth frame')
    parser.add_argument('--count', type=int, default=0, help='Stop after N frames (0: forever)')
    args = parser.parse_args()

    distances_cm = [int(round(float(d) * 100.0)) for d in args.distances.split(',')]
    master, slave = pty.openpty()
    tty.setraw(slave)  # no line discipline translation of the binary frames
    if os.path.lexists(args.link):
        os.remove(args.link)
    os.symlink(os.ttyname(slave), args.link)
    print(f'Benewake simulator on {os.ttyname(slave)} ({args.link})', flush=True)

    rng = random.Random(0)
    period = 1.0 / args.rate
    sent = 0
    try:
        while not args.count or sent < args.count:
            n = sent + 1
            if args.garbage_every and n % args.garbage_every == 0:
                os.write(master, bytes(rng.randrange(256) for _ in range(5)))
            if args.weak_every and n % args.weak_every == 0:
                frame = encode_frame(0xFFFF, 50, args.temperature)
            else:
                frame = encode_frame(distances_cm[sent % len(distances_cm)],
                                     args.strength, args.temperature)
            os.write(master, frame)
            sent += 1
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        os.remove(args.link)


if __name__ == '__main__':
    main()
