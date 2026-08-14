#!/usr/bin/env python3
"""Print the geodetic ArduPilot home recorded in a terrain manifest."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest")
    args = parser.parse_args()
    manifest = json.loads(Path(args.manifest).read_text(encoding="utf-8"))
    home = manifest["home_geodetic"]
    print(f"{home['latitude']},{home['longitude']},{home['altitude_m']}")


if __name__ == "__main__":
    main()
