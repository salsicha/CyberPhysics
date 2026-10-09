# whereami

Room-level localization from WiFi fingerprints. The vendored
[kootenpv/whereami](https://github.com/kootenpv/whereami) (in `whereami/`)
learns the access points and signal strengths seen in each named place and
predicts the current one with a random forest; a ROS 2 node publishes the
prediction.

**NOTICE:** `whereami/` is a vendored copy of kootenpv/whereami, licensed
under the GNU Affero General Public License v3.0 (AGPL-3.0, see
`whereami/LICENSE`; its `setup.py` still says MIT, the LICENSE file
governs). The image installs it unmodified and keeps its source and license
in `/opt/whereami`. It depends on `access_points` 0.4.73 from PyPI (same
author; its metadata lists both MIT and GPL-3.0).

## Topics

| Topic | Type | |
|---|---|---|
| `/whereami/location` | `std_msgs/String` | most likely location label |
| `/whereami/confidence` | `std_msgs/Float32` | its probability, 0-1 |
| `/whereami/probabilities` | `std_msgs/String` | JSON `{label: probability}` (as `whereami predict_proba`) |

Reliable, transient-local, depth 1: a late subscriber gets the current
location. Nothing is published until a model exists; until then the node
logs `No whereami model at /root/.whereami/model.pkl yet: learn each room
first, ...`. It reloads `model.pkl` whenever it changes, so rooms can be
trained while it runs.

## Train and run

```bash
make -C applications build_whereami

# In each room (several samples, 15 s apart; move around the room)
docker compose -f compositions/whereami.yaml run --rm whereami_train learn -l kitchen -n 10
docker compose -f compositions/whereami.yaml run --rm whereami_train learn -l lab -n 10
docker compose -f compositions/whereami.yaml run --rm whereami_train locations
docker compose -f compositions/whereami.yaml run --rm whereami_train crossval    # >= 10 samples
docker compose -f compositions/whereami.yaml run --rm whereami_train predict_proba

docker compose -f compositions/whereami.yaml up -d
ros2 topic echo /whereami/location
```

`whereami_train` runs whereami's own CLI (`learn`, `train`, `locations`,
`crossval`, `predict`, `predict_proba`, `rename`) through
`scripts/whereami_cli.py`, which only swaps in the container's WiFi scanner.
Samples (`<label>.txt`, one JSON scan per line) and `model.pkl` are written
to `WHEREAMI_MODEL_DIR` (default `cache/whereami/`, outside git), mounted at
`/root/.whereami` (read-only for the node). To delete samples, edit or
remove `<label>.txt` and run `train`. `model.pkl` is a pickled
scikit-learn pipeline: only mount a directory you trained yourself, and run
`train` again after a scikit-learn upgrade (or to reuse samples recorded
with a host install of whereami).

## Host requirements

The container scans the host's WiFi; it never runs NetworkManager itself.

- **nmcli (default, `WHEREAMI_SCANNER=nmcli`)**: NetworkManager manages the
  WiFi interface on the host. The host's system D-Bus socket
  `/run/dbus/system_bus_socket` is bind-mounted read-only (the compose file
  refuses to start without it); the container runs as root, which
  NetworkManager allows to list and rescan access points. No capabilities
  or privileged mode.
- **iwlist (`WHEREAMI_SCANNER=iwlist`)**: hosts without NetworkManager.
  Needs `network_mode: host` (already set) to see the interface and
  `cap_add: [NET_ADMIN]` to trigger a scan; without it only the kernel's
  cached results come back. The D-Bus mount can then be removed.
- Train and run with the same scanner: nmcli reports signal 0-100, iwlist
  the driver's quality scale.
- `auto` picks nmcli when the D-Bus socket is mounted, else iwlist. The
  upstream `access_points.get_scanner()` is not used as-is because it
  detects NetworkManager with `systemctl`, which fails in a container, and
  runs iwlist through `sudo`.

## Configuration

Environment (shared by the node and the training CLI):

| Variable | Default | |
|---|---|---|
| `WHEREAMI_SCANNER` | `nmcli` (compose), `auto` (image) | `auto`, `nmcli`, `iwlist`, `file` |
| `WHEREAMI_DEVICE` | any | WiFi interface, e.g. `wlan0` |
| `WHEREAMI_SCAN_FILE` | | JSON list of access points to replay (`file` scanner; the `access_points` command's output format) |
| `WHEREAMI_NMCLI_RESCAN` | `auto` (node), `yes` (CLI) | nmcli `--rescan`: `auto` reuses a scan younger than 30 s |
| `WHEREAMI_MODEL_DIR` | `cache/whereami` | host directory (compose only) |

Node parameters: `scanner`, `device`, `scan_file`, `nmcli_rescan` (defaults
from the variables above), `model_dir` (`~/.whereami` = `/root/.whereami`),
`scan_period_s` (10), `scan_timeout_s` (20), `location_topic`,
`confidence_topic`, `probabilities_topic`.

## Without WiFi

The `file` scanner replays a recorded scan, e.g. to try the pipeline in a
container with no WiFi:

```bash
echo '[{"ssid": "home", "bssid": "aa:bb:cc:dd:ee:01", "quality": 80, "security": "WPA2"}]' > /tmp/scan.json
docker run --rm --network none -v /tmp/scan.json:/scan.json:ro -v /tmp/wm:/root/.whereami \
  -e WHEREAMI_SCANNER=file -e WHEREAMI_SCAN_FILE=/scan.json \
  salsicha/cyberphysics:whereami-latest bash -c \
  'python3 /scripts/whereami_cli.py learn -l kitchen && python3 /scripts/whereami_node.py'
```

Unit tests: `tests/test_whereami_node.py`.
