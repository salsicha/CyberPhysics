#!/bin/bash
# Host setup for CyberPhysics (Ubuntu 24.04).
#
#   ./install.sh              # docker, NVIDIA container toolkit, git-lfs, udev rules, ...
#   ./install.sh --with-k3s   # additionally install k3s (only needed for k3s deployments)
set -e

install_k3s=0
for arg in "$@"; do
  case "$arg" in
    --with-k3s) install_k3s=1 ;;
    -h|--help)
      sed -n '2,5p' "$0" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *)
      echo "Unknown option: $arg (see ./install.sh --help)"
      exit 1
      ;;
  esac
done

if [ "$(id -u)" -eq 0 ]; then
  echo "You cannot run this script with sudo!"
  exit 1
fi

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Setting up git"
sudo apt-get update
sudo apt-get install -y git-lfs curl
# Repository-local preferences (do not override the user's global git config)
git -C "$repo_dir" config pull.rebase true
git -C "$repo_dir" config fetch.prune true
git lfs install

# Sometimes needed to talk to devices
sudo usermod -a -G dialout "$USER"

# Oak-1 camera
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Install Docker
if [ -x "$(command -v docker)" ]; then
    echo "Docker is already installed on this machine"
    docker --version
else
    echo "Installing docker"
    curl -fsSL https://get.docker.com -o get-docker.sh
    DRY_RUN=0 sudo sh ./get-docker.sh
    rm -f get-docker.sh
fi

# Enable docker command without sudo
if ! getent group docker > /dev/null; then
  sudo groupadd docker || true
fi
sudo usermod -aG docker "$USER"

# Install Foxglove
sudo snap install foxglove-studio
echo "To run Foxglove, open terminal: foxglove-studio"

# Create data directory for recordings
sudo mkdir -p /data

# Enable Nvidia GPU passthrough
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update && sudo apt-get install -y make nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl daemon-reload
sudo systemctl restart docker

# For Jetson emulation
echo "Installing qemu (for Jetson platform target)"
sudo apt install -y qemu-user-static binfmt-support

# Let containers running as root use the host X display. This only admits the
# local root user, unlike "xhost +local:" which admits every local process.
if ! grep -q 'xhost +SI:localuser:root' "$HOME/.bashrc"; then
    echo '[ -n "$DISPLAY" ] && xhost +SI:localuser:root > /dev/null' >> "$HOME/.bashrc"
fi
if grep -qE 'xhost \+local:' "$HOME/.bashrc"; then
    echo "NOTE: ~/.bashrc still runs 'xhost +local:...', which lets every local process"
    echo "      use your X display; remove those lines unless you need them."
fi
if ! grep -q host.docker.internal /etc/hosts; then
    echo '127.0.0.1    host.docker.internal' | sudo tee -a /etc/hosts
fi

# Install k3s (opt-in)
if [ "$install_k3s" -eq 1 ]; then
    curl -sfL https://get.k3s.io | sh -s - --docker
    echo "FYI, K3S will automatically prune when disk nears capacity!"
else
    echo "Skipping k3s (rerun with --with-k3s to install it)"
fi

echo "All done."