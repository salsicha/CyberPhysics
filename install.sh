#!/bin/bash
set -e

if [ `id -u` -eq 0 ]; then
  echo "You cannot run this script with sudo!"
  exit 1
fi

echo "Setting up git"
sudo apt install -y git-lfs curl
git config --global pull.rebase true
git config --global fetch.prune true
git lfs install
git config --global credential.helper 'cache --timeout=31536000'

# Sometimes needed to talk to devices
sudo usermod -a -G dialout $USER

# Install Docker
if [ -x "$(command -v docker)" ]; then
    echo "Docker is already installed on this machine"
    echo $(docker --version)
else
    echo "Installing docker"
    curl -fsSL https://get.docker.com -o get-docker.sh
    DRY_RUN=0 sudo sh ./get-docker.sh
fi

# Enable docker command without sudo
if ! [ $(getent group docker) ]; then
  sudo groupadd docker || true
fi
sudo usermod -aG docker $USER

# Install Foxglove
sudo snap install foxglove-studio
echo "To run Foxglove, open terminal: foxglove-studio"

# Create data directory for recordings
sudo mkdir /data

# Enable Nvidia GPU passthrough
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L -k https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L -k https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y make nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# For Jetson emulation
echo "Installing qemu (for Jetson platform target)"
sudo apt install -y qemu binfmt-support qemu-user-static

# Convenient shortcuts
if [ $(cat /home/$USER/.bashrc | grep -c 'xhost +local:root') -lt 1 ]; then
    echo 'xhost +local:root' >> /home/$USER/.bashrc
    echo 'xhost +local:docker' >> /home/$USER/.bashrc
fi
if [ $(cat /etc/hosts | grep -c host.docker.internal) -lt 1 ]; then
    echo '127.0.0.1    host.docker.internal' | sudo tee -a /etc/hosts
fi

# Install k3s
curl -sfL https://get.k3s.io | sh -s - --docker
echo "FYI, K3S will automatically prune when disk nears capacity!"

echo "All done."