#!/usr/bin/env python3
"""Evaluate a pinned learned SmolVLA policy with actual MuJoCo jaw contacts.

This checkpoint's embodiment is SO-ARM100, not the repository's SO101 Isaac
URDF. State and actions are six MuJoCo joint angles in radians (including jaw).
No expert, object pose, or success signal enters the policy observation.
"""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

import numpy as np

MODEL = "ahmedsohail2003/smolvla-so101-pickplace-v2"
REVISION = "f43f9f3a1c81b050f9747919c53019fcae2caec0"
ENV_REVISION = "5822ac1dd508c92183fd8ecc99f91d6ec386549b"
TASK = "Pick up the red block and place it in the blue tray."
PARAPHRASES = [
    TASK,
    "Put the red cube into the blue tray.",
    "Lift the red block and move it to the blue container.",
]
SCENARIOS = {
    "nominal",
    "visual",
    "physics",
    "full",
    "occlusion",
    "perturbation",
    "paraphrase",
}


def checked_action(action, ranges):
    action = np.asarray(action, dtype=np.float64)
    if action.shape != (6,) or not np.isfinite(action).all():
        raise ValueError("Policy must produce six finite joint angles")
    bounded = np.clip(action, ranges[:, 0], ranges[:, 1])
    return bounded, bool(np.any(bounded != action))


def stable_placement(
    position, rotation, half_size, tray_xy, tray_half, speed, tcp_z, jaw
):
    """Require the entire block inside, supported at rest, and released."""
    extent = np.abs(np.asarray(rotation).reshape(3, 3)) @ np.asarray(half_size)
    return bool(
        np.all(np.abs(np.asarray(position)[:2] - tray_xy) + extent[:2] < tray_half)
        and 0.004 <= position[2] - extent[2] <= 0.010
        and speed < 0.04
        and tcp_z > 0.07
        and jaw > 0.8
    )


def summarize_episodes(rows):
    """Keep task completion distinct from collision-free policy acceptance."""
    scenarios = {}
    for name in sorted({row["scenario"] for row in rows}):
        group = [row for row in rows if row["scenario"] == name]
        count = sum(row["success"] for row in group)
        scenarios[name] = {
            "successes": count,
            "episodes": len(group),
            "success_rate": count / len(group),
            "collision_free_successes": sum(
                row["collision_free_success"] for row in group
            ),
        }
    nominal_pass = scenarios.get("nominal", {}).get("success_rate", 0) >= 0.8
    robustness_pass = bool(scenarios) and all(
        s["success_rate"] >= 0.8 for s in scenarios.values()
    )
    contact_pass = bool(rows) and all(row["collision_free_success"] for row in rows)
    return {
        "scenarios": scenarios,
        "nominal_task_acceptance_passed": nominal_pass,
        "evaluated_robustness_acceptance_passed": robustness_pass,
        "collision_free_acceptance_passed": contact_pass,
        "passed": nominal_pass and robustness_pass and contact_pass,
    }


class PhysicsAudit:
    def __init__(self, env):
        self.env = env
        m = env.model
        self.block = m.body("block").id
        self.base = m.body("Base").id
        self.jaws = {m.body("Fixed_Jaw").id, m.body("Moving_Jaw").id}
        self.robot = {
            m.body(name).id
            for name in (
                "Base",
                "Rotation_Pitch",
                "Upper_Arm",
                "Lower_Arm",
                "Wrist_Pitch_Roll",
                "Fixed_Jaw",
                "Moving_Jaw",
            )
        }
        self.block_geom = m.geom("block_geom").id
        self.velocity_address = m.jnt_dofadr[m.joint("block_free").id]
        self.contact_steps = Counter()
        self.contact_events = Counter()
        self.active_pairs = set()
        self.max_contact_force_n = 0.0
        self.bilateral_contact_steps = 0
        self.lift_with_contact_m = 0.0
        self.max_lift_m = 0.0
        self.joint_limit_steps = 0
        self.initial_z = float(env.block_pos[2])

    def sample(self):
        import mujoco

        m, d = self.env.model, self.env.data
        now_pairs, jaw_contacts = set(), set()
        for contact in d.contact[: d.ncon]:
            a, b = int(contact.geom1), int(contact.geom2)
            ba, bb = int(m.geom_bodyid[a]), int(m.geom_bodyid[b])
            # Intended contacts: block/jaws, block/tray/floor, fixed base/floor.
            if self.block in (ba, bb) and (ba in self.jaws or bb in self.jaws):
                jaw_contacts.add(bb if ba == self.block else ba)
                category = "jaw_block"
            elif ba in self.robot or bb in self.robot:
                if self.base in (ba, bb) and 0 in (ba, bb):
                    category = "base_support"
                elif ba in self.robot and bb in self.robot:
                    category = "robot_self"
                else:
                    category = "robot_environment"
            else:
                category = "object_support"
            pair = (category, min(a, b), max(a, b))
            now_pairs.add(pair)
            if pair not in self.active_pairs:
                self.contact_events[category] += 1
            self.contact_steps[category] += 1
        self.active_pairs = now_pairs
        height = float(self.env.block_pos[2]) - self.initial_z
        self.max_lift_m = max(self.max_lift_m, height)
        if jaw_contacts == self.jaws:
            self.bilateral_contact_steps += 1
            self.lift_with_contact_m = max(self.lift_with_contact_m, height)
        if np.any(d.qpos[:6] < m.jnt_range[:6, 0] - 0.03) or np.any(
            d.qpos[:6] > m.jnt_range[:6, 1] + 0.03
        ):
            self.joint_limit_steps += 1
        # Contact forces are measured from the solver, not inferred from proximity.
        for idx in range(d.ncon):
            wrench = np.zeros(6)
            mujoco.mj_contactForce(m, d, idx, wrench)
            self.max_contact_force_n = max(
                self.max_contact_force_n, float(np.linalg.norm(wrench[:3]))
            )

    def settled(self):
        from sim2cell.env import TRAY_HALF, TRAY_POS

        m, d = self.env.model, self.env.data
        speed = np.linalg.norm(
            d.qvel[self.velocity_address : self.velocity_address + 3]
        )
        return stable_placement(
            self.env.block_pos,
            d.xmat[self.block],
            m.geom_size[self.block_geom],
            TRAY_POS,
            TRAY_HALF,
            speed,
            self.env.tcp_pos[2],
            d.qpos[5],
        )


def run(args):
    import mujoco
    import torch
    from huggingface_hub import snapshot_download
    from lerobot.configs import PreTrainedConfig
    from lerobot.policies import make_pre_post_processors
    from lerobot.policies.smolvla import SmolVLAPolicy
    from sim2cell.env import PickPlaceEnv
    from so101_policy_server import require_checkpoint_statistics

    levels = args.scenarios.split(",")
    if not set(levels) <= SCENARIOS or len(set(levels)) != len(levels):
        raise ValueError(
            "Scenarios must be distinct names from: " + ",".join(sorted(SCENARIOS))
        )
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA required for this measured GPU evaluation")
    torch.set_num_threads(4)
    torch.manual_seed(args.seed)
    path = snapshot_download(MODEL, revision=REVISION, ignore_patterns=["*.gif"])
    cfg = PreTrainedConfig.from_pretrained(path)
    cfg.device = "cuda"
    cfg.n_action_steps = min(args.action_horizon, cfg.chunk_size)
    pre, post = make_pre_post_processors(
        cfg, path, preprocessor_overrides={"device_processor": {"device": "cuda"}}
    )
    require_checkpoint_statistics(cfg, pre, post)
    if tuple(cfg.robot_state_feature.shape) != (6,) or tuple(
        cfg.action_feature.shape
    ) != (6,):
        raise ValueError("Checkpoint embodiment differs from workcell")
    torch.cuda.reset_peak_memory_stats()
    policy = (
        SmolVLAPolicy.from_pretrained(path, config=cfg, strict=True).to("cuda").eval()
    )
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    rows, latency_ms, chunk_latency_ms = [], [], []
    started = time.monotonic()
    for scenario in levels:
        env = PickPlaceEnv(
            seed=args.seed,
            randomize=scenario if scenario in ("visual", "physics", "full") else "none",
        )
        try:
            for episode in range(args.episodes):
                # Pair spawn and policy randomness across conditions. Rendering
                # randomization consumes its own RNG, so do not draw the spawn
                # from that stream when comparing robustness conditions.
                rng = np.random.default_rng(args.seed + episode)
                radius, angle = (
                    rng.uniform(0.19, 0.27),
                    rng.uniform(np.deg2rad(-20), np.deg2rad(35)),
                )
                obs = env.reset(
                    block_pos=np.array(
                        [radius * np.sin(angle), -radius * np.cos(angle)]
                    )
                )
                torch.manual_seed(args.seed + episode)
                # No state changes after reset. Perturbation uses an external force.
                audit = PhysicsAudit(env)
                policy.reset()
                pre.reset()
                post.reset()
                settled_steps, clips = 0, 0
                frames, trajectory = [], []
                instruction = (
                    PARAPHRASES[episode % len(PARAPHRASES)]
                    if scenario == "paraphrase"
                    else TASK
                )
                for step in range(args.max_steps):
                    batch = {
                        "observation.state": torch.from_numpy(obs["state"]).unsqueeze(
                            0
                        ),
                        "task": instruction,
                    }
                    for source, key in (("front", "camera1"), ("wrist", "camera2")):
                        rgb = obs["images"][source].copy()
                        if (
                            scenario == "occlusion"
                            and source == "front"
                            and 15 <= step < 45
                        ):
                            rgb[70:150, 65:150] = 80
                        batch[f"observation.images.{key}"] = (
                            torch.from_numpy(rgb).permute(2, 0, 1).unsqueeze(0).float()
                            / 255
                        )
                    before = time.perf_counter()
                    new_chunk = not policy._queues["action"]
                    with torch.inference_mode():
                        action = (
                            post(policy.select_action(pre(batch)))
                            .squeeze(0)
                            .cpu()
                            .numpy()
                        )
                    torch.cuda.synchronize()
                    elapsed_ms = (time.perf_counter() - before) * 1000
                    latency_ms.append(elapsed_ms)
                    if new_chunk:
                        chunk_latency_ms.append(elapsed_ms)
                    bounded, clipped = checked_action(
                        action, env.model.actuator_ctrlrange
                    )
                    clips += clipped
                    env.data.ctrl[:] = bounded
                    for substep in range(env.n_substeps):
                        env.data.xfrc_applied[:] = 0
                        if scenario == "perturbation" and step == 35:
                            env.data.xfrc_applied[audit.block, 0] = 0.30
                        mujoco.mj_step(env.model, env.data)
                        audit.sample()
                    obs = env.get_obs()
                    trajectory.append(
                        {
                            "t": float(env.data.time),
                            "joints": obs["state"].tolist(),
                            "action": bounded.tolist(),
                            "block_xyz": env.block_pos.tolist(),
                        }
                    )
                    if episode == 0:
                        frames.append(
                            np.concatenate(
                                (obs["images"]["front"], obs["images"]["wrist"]), axis=1
                            )
                        )
                    settled_steps = settled_steps + 1 if audit.settled() else 0
                    if settled_steps * env.n_substeps * env.model.opt.timestep >= 0.5:
                        break
                placed = settled_steps * env.n_substeps * env.model.opt.timestep >= 0.5
                success = bool(
                    placed
                    and audit.bilateral_contact_steps > 0
                    and audit.lift_with_contact_m >= 0.06
                )
                row = {
                    "scenario": scenario,
                    "episode": episode,
                    "success": success,
                    "stable_placement": placed,
                    "instruction": instruction,
                    "steps": step + 1,
                    "simulation_seconds": float(env.data.time),
                    "final_xyz": env.block_pos.tolist(),
                    "max_lift_m": audit.max_lift_m,
                    "bilateral_lift_m": audit.lift_with_contact_m,
                    "bilateral_contact_steps": audit.bilateral_contact_steps,
                    "contact_events": dict(audit.contact_events),
                    "collision_free_success": bool(
                        success
                        and not audit.contact_events["robot_environment"]
                        and not audit.contact_events["robot_self"]
                    ),
                    "max_contact_force_n": audit.max_contact_force_n,
                    "joint_limit_steps": audit.joint_limit_steps,
                    "clipped_commands": clips,
                }
                rows.append(row)
                print(json.dumps(row), flush=True)
                (output / f"{scenario}_{episode:02d}.json").write_text(
                    json.dumps(trajectory)
                )
                if frames:
                    import imageio.v3 as iio

                    iio.imwrite(
                        output / f"{scenario}.mp4",
                        np.asarray(frames),
                        plugin="pyav",
                        codec="libx264",
                        fps=int(env.control_hz),
                    )
        finally:
            env._renderer.close()
    report = {
        "model": MODEL,
        "revision": REVISION,
        "environment_revision": ENV_REVISION,
        "robot": "MuJoCo Menagerie SO-ARM100; not the SO101 Isaac URDF",
        "model_sha256": file_sha256(Path(path) / "model.safetensors"),
        "mujoco": mujoco.__version__,
        "torch": torch.__version__,
        "seed": args.seed,
        "action_horizon": cfg.n_action_steps,
        "gpu": torch.cuda.get_device_name(),
        "wall_seconds": time.monotonic() - started,
        "ground_truth_to_policy": False,
        "scripted_fallback": False,
        "grasp_model": "free rigid body, friction and force-limited jaw contacts; no attachment",
        "policy_observations": [
            "front RGB",
            "wrist RGB",
            "joint angles",
            "instruction",
        ],
        "peak_allocated_mib": torch.cuda.max_memory_allocated() / 2**20,
        "peak_reserved_mib": torch.cuda.max_memory_reserved() / 2**20,
        "latency_p50_ms": float(np.median(latency_ms)),
        "latency_p95_ms": float(np.percentile(latency_ms, 95)),
        "fresh_chunk_latency_p50_ms": float(np.median(chunk_latency_ms)),
        "fresh_chunk_latency_p95_ms": float(np.percentile(chunk_latency_ms, 95)),
        "fresh_chunks": len(chunk_latency_ms),
        "language_scope": "One trained task; paraphrases do not establish instruction following for other tasks",
        **summarize_episodes(rows),
        "episodes": rows,
    }
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="/workspace/results/workcell")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--max-steps", type=int, default=240)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument("--action-horizon", type=int, default=50)
    parser.add_argument(
        "--scenarios",
        default="nominal,visual,physics,full,occlusion,perturbation,paraphrase",
    )
    args = parser.parse_args()
    if args.episodes < 1 or args.max_steps < 1 or args.action_horizon < 1:
        parser.error("Episode and step counts must be positive")
    report = run(args)
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
