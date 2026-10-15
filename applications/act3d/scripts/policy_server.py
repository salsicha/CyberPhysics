"""Loopback HTTP inference, matching the shared Isaac sparse-pose executor."""
import argparse
import hashlib
import io
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
import time
import zipfile

import numpy as np
import torch

from act3d_policy import load_policy
from contracts import config_digest
from observations import checked_observation


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", default="/workspace/models/act3d_reach.pt")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5563)
    p.add_argument("--device", default="cuda")
    p.add_argument("--seed", type=int, default=321)
    args = p.parse_args()
    torch.set_num_threads(4)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
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
                       {"ready": True, "model": "Act3D", "checkpoint_sha256": sha,
                        "config_digest": config_digest(data["config"])})

        def do_POST(self):
            try:
                if self.path != "/predict":
                    raise ValueError("Unknown endpoint")
                size = int(self.headers.get("Content-Length", 0))
                if not 0 < size <= 4_000_000:
                    raise ValueError("Invalid request size")
                with np.load(io.BytesIO(self.rfile.read(size)), allow_pickle=False) as obs:
                    rgb, points, gripper, instruction = checked_observation(obs, data["config"])
                start = time.monotonic()
                action = policy.predict(rgb, points, gripper, data["language"][instruction])
                if args.device.startswith("cuda"):
                    torch.cuda.synchronize()
                action.update(inference_ms=(time.monotonic()-start)*1000,
                              checkpoint_sha256=sha, policy_used_ground_truth=False,
                              peak_allocated_mib=torch.cuda.max_memory_allocated()/2**20
                              if args.device.startswith("cuda") else 0)
                self.reply(200, action)
            except (ValueError, KeyError, OSError, TypeError, zipfile.BadZipFile) as exc:
                self.reply(400, {"error": str(exc)})

    print(json.dumps({"ready": True, "model": "Act3D", "checkpoint_sha256": sha}), flush=True)
    HTTPServer((args.host, args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
