# Jetson Orin GPU images

The car's GPU applications have native `linux/arm64` build paths for
**JetPack 7.2 / Jetson Linux 39.2.x**, Ubuntu 24.04, Python 3.12 and ROS 2
Jazzy. This includes the Orin Nano Super in the hardware BOM. JetPack 6
and the original Jetson Nano are not supported by these images.

| Image tag | ARM64 GPU stack |
| --- | --- |
| `nvblox-arm64` | Isaac ROS 4.6, `noble-jetpack` packages, CUDA 13.2, VPI 4 |
| `pantiltros-arm64` | PyTorch 2.10.0 + CUDA 13.0, torchvision 0.25.0 + CUDA 13.0, Ultralytics 8.4.52 |
| `depthanything-arm64` | The same CUDA PyTorch/torchvision pair, metric Depth Anything V2 |

The desktop `*-latest` builds retain their existing package versions.
Build locally before deployment; publishing these tags to Docker Hub
is a separate step. Native ARM64 build CI
checks package installation and CUDA-enabled binaries without a GPU;
the deployment Jetson must also pass the runtime check below. Throughput
and combined memory use on the physical Orin Nano remain to be measured.

NVIDIA documents Orin with JetPack 7.2 in the
[Isaac ROS 4.6 requirements](https://nvidia-isaac-ros.github.io/v/release-4.6/getting_started/index.html).
The Isaac repository suite is `noble-jetpack`, not the desktop `noble`
suite or the Thor-only 4.4 release. The
[Ultralytics Jetson guide](https://docs.ultralytics.com/guides/nvidia-jetson/)
uses upstream `cu130` PyTorch wheels for JetPack 7.2. Both Python images
explicitly select those CUDA wheels, and reject CPU-only builds during
image construction.

## Build on the Jetson

Check `cat /etc/nv_tegra_release` for `R39` and `REVISION: 2.x`. Install
JetPack's compute components and NVIDIA Container Toolkit. Configure the
Docker runtime if it is not already configured:

```bash
sudo nvidia-ctk runtime configure --runtime=docker --set-as-default
sudo systemctl restart docker
```

The repository's `./install.sh` also configures persistent DDS socket
limits. If Docker and the NVIDIA runtime are already installed, apply
just the networking setup:

```bash
sudo python3 tools/configure_dds_buffers.py
python3 tools/configure_dds_buffers.py --check
```

This writes `/etc/sysctl.d/99-cyberphysics-dds.conf`, applies it immediately,
and verifies both `net.core.rmem_max` and `net.core.wmem_max` are at least
**4194304 bytes (4 MiB)**. Higher existing values are preserved. Ubuntu's
`systemd-sysctl` reapplies the file at boot. The Jetson GPU smoke test checks
these limits; `./start.sh` checks them for the affected vehicle and turret
stacks when a DDS fallback is selected. Native Zenoh uses its TCP transport
and does not require this DDS UDP buffer preflight. Direct `docker compose`
invocations bypass `start.sh`; run `--check` before using DDS that way.
These settings belong on the host, not in the host-network containers.

From this checkout, build the car and turret images natively. Allow ample
NVMe space for CUDA packages, intermediate layers and model checkpoints
(NVIDIA specifies a 128 GB or larger NVMe drive for Isaac ROS).

```bash
make -C applications PLATFORM=arm64 \
  build_racecarneo build_realsense build_nicegui \
  build_nvblox build_pantiltros

# Optional comparison model; the car maps RealSense depth by default.
make -C applications PLATFORM=arm64 build_depthanything
```

The Makefile builds `ros2-arm64` first. Compose's turret build forwards
`CYBERPHYSICS_TAG` to `TAG`, so it uses that same ARM64 base. An x86 host
needs an ARM64 builder or configured emulation to build these images;
emulation cannot validate Jetson GPU execution.

The ARM64 nvblox image omits the large recorded quickstart assets. To
include them, build its Dockerfile with `--build-arg
DOWNLOAD_EXAMPLE_ASSETS=1`; physical camera mapping does not use them.

## Verify GPU execution before starting the robot

```bash
tools/jetson_gpu_smoke.sh nvblox
tools/jetson_gpu_smoke.sh pantiltros
tools/jetson_gpu_smoke.sh depthanything  # if built
```

These containers have no network or camera/actuator devices. The checks
verify the host release and image architecture, then execute CUDA matrix
multiplication, convolution and torchvision NMS, run an Ultralytics model
or the bundled metric depth checkpoint, and load the nvblox ROS component
and Nav2 plugin. Merely importing Torch or checking that a launch process
stays alive is insufficient. A failed check exits nonzero; do not treat
CPU fallback as successful Jetson GPU deployment.

After the checks pass, use the existing bring-up procedure for the robot:

```bash
export CYBERPHYSICS_TAG=arm64
./start.sh racecar_turret_hardware -d
# Without the turret: ./start.sh racecarneo_hardware -d
```

Mapping and detection can then execute locally; an x86 GPU machine is
not an architectural dependency. Depth Anything remains an optional
comparison profile because its metric depth was inaccurate for the car's
low camera position. GPU support does not change that result or establish
that all three workloads can run simultaneously at their desktop rates
within the Orin Nano's shared 8 GB memory.
