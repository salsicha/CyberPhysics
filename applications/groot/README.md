# NVIDIA Isaac GR00T

This application builds a container for NVIDIA Isaac GR00T N1.7 from the upstream `NVIDIA/Isaac-GR00T` repository. The upstream repository and Python dependencies are downloaded at image build time; model checkpoints and datasets are not stored in this repo.

## Build

```bash
make -C applications build_groot
```

Optional build args can pin a specific upstream ref:

```bash
docker build \
  --build-arg GR00T_REF=main \
  -t cyberphysics/groot:latest \
  applications/groot
```

## Run A Smoke Test

```bash
docker compose -f compositions/groot.yaml up groot
```

The default command imports `gr00t` and `torch`, then reports whether CUDA is visible. Full inference and fine-tuning require an NVIDIA GPU, NVIDIA Container Toolkit, and enough VRAM for the selected checkpoint.

## Interactive Shell

```bash
docker compose -f compositions/groot.yaml run --rm groot bash
```

Inside the container, the upstream checkout is at `/opt/Isaac-GR00T` and its virtual environment is already active.

Upstream project: https://github.com/NVIDIA/Isaac-GR00T
