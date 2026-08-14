#!/usr/bin/env python3
"""Health probe for the local SmolVLA ZeroMQ policy service."""

import os
import sys

import msgpack
import zmq


def main() -> int:
    host = os.environ.get("SMOLVLA_HOST", "127.0.0.1")
    port = int(os.environ.get("SMOLVLA_PORT", "5556"))
    timeout_ms = int(os.environ.get("SMOLVLA_HEALTH_TIMEOUT_MS", "5000"))

    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
    socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
    try:
        socket.connect(f"tcp://{host}:{port}")
        socket.send(msgpack.packb({"endpoint": "health"}, use_bin_type=True))
        response = msgpack.unpackb(socket.recv(), raw=False)
        if not isinstance(response, dict) or response.get("status") != "ok":
            print(f"Unhealthy SmolVLA response: {response}", file=sys.stderr)
            return 1
        return 0
    except Exception as exc:
        print(f"SmolVLA health check failed: {exc}", file=sys.stderr)
        return 1
    finally:
        socket.close()
        context.term()


if __name__ == "__main__":
    raise SystemExit(main())
