#!/bin/bash
# Install lossless hourly archiving of inactive BlueOS telemetry recordings.
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ "$(id -u)" != 0 ]; then
  echo "Run with sudo: this installs a host systemd timer." >&2
  exit 1
fi
test -d /root/.config/blueos/ardupilot-manager/logs
install -Dm755 "$repo_dir/tools/blueos_log_archive.py" /usr/local/lib/cyberphysics/blueos_log_archive.py
cat > /etc/systemd/system/cyberphysics-blueos-log-archive.service <<'UNIT'
[Unit]
Description=Compress inactive BlueOS recordings without removing recorded data
ConditionPathIsDirectory=/root/.config/blueos/ardupilot-manager/logs
[Service]
Type=oneshot
ExecStart=/usr/bin/python3 /usr/local/lib/cyberphysics/blueos_log_archive.py
Nice=15
IOSchedulingClass=idle
UNIT
cat > /etc/systemd/system/cyberphysics-blueos-log-archive.timer <<'UNIT'
[Unit]
Description=Hourly BlueOS recording archive
[Timer]
OnCalendar=hourly
Persistent=true
RandomizedDelaySec=300
[Install]
WantedBy=timers.target
UNIT
systemctl daemon-reload
systemctl enable --now cyberphysics-blueos-log-archive.timer
systemctl start --no-block cyberphysics-blueos-log-archive.service
