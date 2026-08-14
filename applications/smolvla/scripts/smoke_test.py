#!/usr/bin/env python3
"""Import-only image smoke test; it deliberately does not download model weights."""

import torch
import transformers

import lerobot
from lerobot.policies.smolvla import SmolVLAPolicy


def main() -> None:
    assert SmolVLAPolicy is not None
    print(
        "SmolVLA image ready: "
        f"lerobot={lerobot.__version__} "
        f"torch={torch.__version__} "
        f"transformers={transformers.__version__} "
        f"cuda_available={torch.cuda.is_available()}"
    )


if __name__ == "__main__":
    main()
