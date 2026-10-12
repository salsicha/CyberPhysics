# Synthetic warehouse segmentation fixture

`warehouse_yoloe26s.pt` is a YOLOE-26s segmentation checkpoint fine-tuned on
480 randomized synthetic scenes, with 96 separately seeded validation scenes.
It is only the default for `compositions/turret_sim.yaml`. Hardware stacks
retain their own model selection. This model is not validated on real cameras.

Fetch the Git LFS artifact after cloning:

```bash
git lfs pull --include='applications/turret_sim/models/*.pt'
```

The original 18-second acceptance trajectory and its frames were excluded
from training. The generator randomizes poses, distances, scales, colors,
backgrounds and lighting using seeds 6101201/6101202; training uses 6101203.
All images use the same synthetic renderer and object assets, so this is
held-out scenario validation within that domain, not independent real-world
generalization evidence. The saved classifier labels must not be overwritten
with text prompts. `configure_model_classes` distinguishes the fused trained
head from a promptable YOLOE head without relying on the filename.

`training_provenance.json` records image hashes, accepted annotation counts,
polygon round-trip checks, training metrics, configuration and checkpoint
SHA-256 hashes. Tiny instances that cannot be encoded faithfully as polygons
are counted and excluded from labels. Reproduce using the PanTiltROS GPU image:

```bash
docker run --rm --gpus all \
  -v "$PWD/applications/turret_sim:/source:ro" \
  -v "$PWD/cache/turret:/workspace/cache" \
  -v "$PWD/generated/turret:/results" \
  -w /workspace/cache/models \
  -e YOLO_CONFIG_DIR=/workspace/cache/ultralytics \
  salsicha/cyberphysics:pantiltros-latest \
  python3 /source/scripts/train_sim_segmentation.py \
    --scenario /source/scenarios/warehouse_tracking.json \
    --output /results/new_training_run
```

The output directory must be new. Training starts from Ultralytics'
`yoloe-26s-seg.pt`; see its [upstream model and licensing documentation](https://docs.ultralytics.com/models/yoloe/).

The saved checkpoint identifies its license as **AGPL-3.0**, inherited from
Ultralytics. That model license is separate from the repository's MIT license.
The training script and provenance above accompany the derived weights.
