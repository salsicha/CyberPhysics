"""Local HTTP policy RPC. The request contains RGB/point cloud, not object poses."""
import argparse
import hashlib
import io
import json
import time
import zipfile
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import numpy as np
import torch

from model import load_policy, config_digest


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", default="/workspace/models/peract_reach.pt")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5562)
    p.add_argument("--device", default="cuda")
    args = p.parse_args()
    torch.set_num_threads(4)
    policy, data = load_policy(args.checkpoint, args.device)
    sha = hashlib.sha256(Path(args.checkpoint).read_bytes()).hexdigest()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, code, result):
            body = json.dumps(result, allow_nan=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            self.reply(200 if self.path == "/health" else 404,
                       {"ready": True, "checkpoint_sha256": sha, "model": "PerAct", "config_digest": config_digest(data["config"])})

        def do_POST(self):
            try:
                if self.path != "/predict":
                    raise ValueError("Unknown endpoint")
                size = int(self.headers.get("Content-Length", 0))
                if not 0 < size <= 4_000_000:
                    raise ValueError("Invalid request size")
                with np.load(io.BytesIO(self.rfile.read(size)), allow_pickle=False) as obs:
                    instruction = str(obs["instruction"])
                    if instruction not in data["language"]:
                        raise ValueError("Instruction is not part of this task's trained vocabulary")
                    w, h = data["config"]["camera_resolution"]
                    rgb, points, proprio = obs["rgb"], obs["points"], obs["proprio"]
                    if rgb.shape != (h, w, 3) or points.shape != rgb.shape or proprio.shape != (4,):
                        raise ValueError("Observation shape mismatch")
                    if (rgb.dtype != np.uint8 or points.dtype not in (np.float32, np.float64)
                            or not np.isfinite(points).all(axis=-1).any() or not np.isfinite(proprio).all()):
                        raise ValueError("Invalid observation")
                    if str(obs["config_digest"]) != config_digest(data["config"]):
                        raise ValueError("Simulator and checkpoint configurations differ")
                    start = time.monotonic()
                    action = policy.predict(rgb, points, proprio, data["language"][instruction])
                    if args.device.startswith("cuda"):
                        torch.cuda.synchronize()
                    action.update(inference_ms=(time.monotonic()-start)*1000,
                                  checkpoint_sha256=sha, policy_used_ground_truth=False,
                                  peak_allocated_mib=torch.cuda.max_memory_allocated()/2**20 if args.device.startswith("cuda") else 0)
                    self.reply(200, action)
            except (ValueError, KeyError, OSError, TypeError, zipfile.BadZipFile) as exc:
                self.reply(400, {"error": str(exc)})

    print(json.dumps({"ready": True, "checkpoint_sha256": sha}), flush=True)
    HTTPServer((args.host, args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
