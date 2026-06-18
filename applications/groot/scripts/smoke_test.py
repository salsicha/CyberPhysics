#!/usr/bin/env python3
import importlib.util
import sys

import torch

if importlib.util.find_spec("gr00t") is None:
    raise SystemExit("gr00t package is not importable")

import gr00t

print("GR00T installed successfully")
print(f"python={sys.version.split()[0]}")
print(f"torch={torch.__version__}")
print(f"cuda_available={torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"cuda_device={torch.cuda.get_device_name(0)}")
