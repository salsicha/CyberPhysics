#!/usr/bin/env python3
"""Return success only after the GR00T policy server accepts requests."""

import os

from gr00t.policy.server_client import PolicyClient


def main():
    client = PolicyClient(
        host=os.environ.get("SO101_GROOT_HEALTH_HOST", "127.0.0.1"),
        port=int(os.environ.get("SO101_GROOT_PORT", "5555")),
        timeout_ms=5000,
    )
    try:
        if not client.ping():
            raise SystemExit(1)
    finally:
        client.close()


if __name__ == "__main__":
    main()
