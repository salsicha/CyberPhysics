#!/usr/bin/env python3
"""Stage the same pinned, patched transport and demo C sources for PlatformIO."""

import argparse
import json
from pathlib import Path
import shutil


def stage(source, destination, picoros):
    if destination.exists():
        raise FileExistsError(f"Refusing to replace existing project: {destination}")
    shutil.copytree(source / "esp32", destination)
    libs = destination / "lib"
    transport = libs / "zenoh-pico"
    shutil.copytree(
        picoros / "thirdparty/zenoh-pico",
        transport,
        ignore=shutil.ignore_patterns(".git", "build", "docs"),
    )
    manifest = json.loads((transport / "library.json").read_text())
    # These are supplied by Arduino-ESP32 itself. Registry packages with the
    # same names are for different targets and break this build.
    manifest.pop("dependencies", None)
    (transport / "library.json").write_text(json.dumps(manifest, indent=2) + "\n")
    native = libs / "cp_native"
    native.mkdir()
    for name in ("picoros.c", "picoros.h"):
        shutil.copy2(picoros / "src" / name, native / name)
    for name in ("demo.c", "demo.h"):
        shutil.copy2(source / "src" / name, native / name)
    (native / "library.json").write_text(
        json.dumps(
            {
                "name": "cp_native",
                "version": "1.0.0",
                "build": {"srcDir": "."},
                "dependencies": {"zenoh-pico": "*"},
            }
        )
        + "\n"
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, default=Path(__file__).resolve().parent)
    p.add_argument("--destination", type=Path, required=True)
    p.add_argument("--picoros", type=Path, default=Path("/opt/picoros"))
    a = p.parse_args()
    stage(a.source, a.destination, a.picoros)
