#!/usr/bin/env python3
"""Run upstream open-loop evaluation with one output plot per trajectory."""

from __future__ import annotations

import argparse
from pathlib import Path

from gr00t.eval import open_loop_eval


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-path", required=True)
    parser.add_argument("--model-path", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--traj-ids", type=int, nargs="+", default=[0])
    parser.add_argument("--execution-horizon", type=int, default=16)
    parser.add_argument("--steps", type=int, default=400)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    upstream_plot = open_loop_eval.plot_trajectory_results

    def plot_per_trajectory(*plot_args, **plot_kwargs):
        trajectory = plot_kwargs.get("traj_id")
        if trajectory is None:
            trajectory = plot_args[3]
        plot_kwargs["save_plot_path"] = str(output_dir / f"traj_{trajectory}.jpeg")
        return upstream_plot(*plot_args, **plot_kwargs)

    open_loop_eval.plot_trajectory_results = plot_per_trajectory
    config = open_loop_eval.ArgsConfig(
        steps=args.steps,
        traj_ids=args.traj_ids,
        execution_horizon=args.execution_horizon,
        dataset_path=args.dataset_path,
        embodiment_tag="NEW_EMBODIMENT",
        model_path=args.model_path,
        save_plot_path=str(output_dir),
    )
    open_loop_eval.main(config)


if __name__ == "__main__":
    main()
