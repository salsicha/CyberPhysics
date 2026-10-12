#!/usr/bin/env python3
"""Assets of the RACECAR Turret Isaac simulation (racecar_turret_isaac.py).

Runs once before the simulator (the racecar_turret_assets service of
compositions/racecar_turret_isaac.yaml, in the racecarneo image: xacro,
numpy and OpenCV) and writes into --output-dir:

* racecar_turret_sim.urdf: systems/racecar_turret/urdf/racecar_turret.urdf.xacro
  expanded with sim_vehicle:=true (the wheels and knuckles physics needs)
  and launcher:=false;
* models/<key>/: the people of the scenario, the pinned CC0 Gazebo Fuel
  models of the file its people.models_from names
  (applications/aerostack2/missions/tracking_targets.json), downloaded once
  into --cache-dir and baked into simulator-neutral meshes by
  applications/demnav/scripts/target_models.py;
* tracking_targets.json: the people's routes resolved on the flat floor by
  applications/demnav/scripts/tracking_targets.py, which the simulator
  replays with simulation time;
* textures/floor.png: a procedural concrete texture (the floor needs
  texture for monocular depth and visual odometry cues);
* manifest.json: the files above and each model's size.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(SCRIPTS.parents[1] / "demnav" / "scripts"))
import racecar_turret_sim as rts  # noqa: E402
from tracking_targets import resolve_tracks  # noqa: E402

URDF_NAME = "racecar_turret_sim.urdf"


def flat_floor(east, north):
    """Height of the arena floor (z = 0) at any point."""
    return np.zeros_like(np.asarray(east, dtype=np.float64))


def expand_urdf(xacro_file, output):
    # The turret's foam dart launcher (interfaces.md) is not simulated: the
    # tracker does not need it (actuator_fitted false).
    urdf = subprocess.run(["xacro", str(xacro_file), "sim_vehicle:=true", "launcher:=false"],
                          check=True, capture_output=True, text=True).stdout
    output.write_text(urdf, encoding="utf-8")
    return urdf


def floor_texture(size=1024, seed=3):
    """Grey concrete: smooth blotches, fine grain and faint 1 m slab joints
    (the texture repeats every 2 m on the floor)."""
    import cv2

    rng = np.random.default_rng(seed)
    image = np.full((size, size), 118.0)
    for scale, amplitude in ((64, 18.0), (16, 10.0), (4, 6.0)):
        noise = rng.normal(0.0, 1.0, (size // scale, size // scale))
        image += amplitude * cv2.resize(noise, (size, size), interpolation=cv2.INTER_CUBIC)
    image += rng.normal(0.0, 5.0, (size, size))
    for joint in (0, size // 2):
        image[max(0, joint - 2):joint + 2, :] -= 35.0
        image[:, max(0, joint - 2):joint + 2] -= 35.0
    image = np.clip(image, 0, 255).astype(np.uint8)
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--repo", default="/workspace",
                        help="root the scenario's relative paths resolve against")
    parser.add_argument("--scenario", default="systems/racecar_turret/scenarios/isaac_arena.json")
    parser.add_argument("--xacro", default="systems/racecar_turret/urdf/racecar_turret.urdf.xacro")
    parser.add_argument("--cache-dir", required=True, help="downloaded Fuel models")
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args(argv)

    from target_models import bake, fetch_model
    import cv2

    repo = Path(args.repo)
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    scenario = rts.load_scenario(repo / args.scenario)
    expand_urdf(repo / args.xacro, output / URDF_NAME)
    print(f"{URDF_NAME}: {args.xacro} sim_vehicle:=true launcher:=false", flush=True)

    people = scenario["people"]
    models = json.loads((repo / people["models_from"]).read_text(encoding="utf-8"))["models"]
    config = rts.people_config(scenario, models)
    sizes = {}
    for key, spec in config["models"].items():
        source = fetch_model(key, spec, args.cache_dir)
        summary = bake(spec, source, output / "models" / key, key)
        sizes[key] = {"size_m": summary["size_m"], "triangles": summary["triangles"],
                      "fuel": spec["fuel"], "version": spec["version"], "license": spec["license"]}
        print(json.dumps(summary), flush=True)

    tracks = resolve_tracks(config, flat_floor, [0.0, 0.0, 0.0], models_dir="models")
    tracks["frame"] = scenario.get("frame", "world")
    (output / "tracking_targets.json").write_text(json.dumps(tracks) + "\n", encoding="utf-8")
    (output / "textures").mkdir(exist_ok=True)
    if not cv2.imwrite(str(output / "textures" / "floor.png"), floor_texture()):
        raise RuntimeError("failed to write the floor texture")
    manifest = {
        "scenario": scenario["name"],
        "urdf": URDF_NAME,
        "tracking_targets": "tracking_targets.json",
        "floor_texture": "textures/floor.png",
        "models": sizes,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"RACECAR Turret assets in {output}: {', '.join(t['name'] for t in tracks['targets'])}",
          flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
