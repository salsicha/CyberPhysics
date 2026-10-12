
# Rangefinder

ROS 2 driver for a Benewake **TFmini-S** or **TF-Luna** single-point LiDAR on
a UART, mounted **downward** on the aerodrone. It publishes
`sensor_msgs/msg/Range` on `/aerodrone/rangefinder`, the height-above-ground
input DemNav uses (`rangefinder_topic`, see `systems/aerodrone/config/`).

```bash
make -C applications build_rangefinder
RANGEFINDER_DEVICE=/dev/serial/by-id/usb-...-if00-port0 \
  docker compose -f compositions/rangefinder.yaml up
ros2 topic echo /aerodrone/rangefinder
```

## Hardware

| | TFmini-S | TF-Luna |
|---|---|---|
| Range (90% reflectivity) | 0.1-12 m (0.1-7 m at 10%) | 0.2-8 m (0.2-2.5 m at 10%) |
| Field of view | 2 deg | 2 deg |
| Supply | 5 V | 3.7-5.2 V |
| UART | 3.3 V LVTTL, 115200 8N1, 100 Hz | 3.3 V LVTTL, 115200 8N1, 100 Hz |

The default port is **`/dev/ttyUSB0` at 115200 baud**: a 3.3 V USB-UART
adapter (CP2102/FT232; sensor TX -> adapter RX, power the sensor from 5 V).
On the BlueOS/Navigator Raspberry Pi the GPIO and Navigator UARTs are
handed to ArduPilot as SERIALx ports, so a USB adapter keeps the sensor
independent of the autopilot's port map. To use a Pi UART instead, pick
one ArduPilot does not open (`SERIALx_PROTOCOL = -1`) and set
`RANGEFINDER_DEVICE=/dev/ttyAMA0` (or `/dev/serial0`). Do not configure
the same sensor as an ArduPilot rangefinder (`RNGFND1_TYPE`): only one
reader can own the port.

In the aircraft stack (`compositions/aerodrone_hardware.yaml`) the
container does not get the port through `devices:`, which Docker refuses
when the adapter is missing (and `docker compose up` then stops with most
of the stack not started). It sees the host's `/dev` read-only at
`/host/dev` and may open only USB serial devices (`ttyUSB*`, `ttyACM*`,
device cgroup rules), so the node waits for an unplugged adapter and
reopens it after a re-plug. A Pi UART needs its rule added there too
(`c 204:* rmw` for `ttyAMA*`); on BlueOS those UARTs are ArduPilot's
SERIALx ports, so only one ArduPilot does not open.

The sensor must stream its factory-default **9-byte/cm** frame
(`0x59 0x59 Dist_L Dist_H Strength_L Strength_H Temp_L Temp_H Checksum`,
checksum = low byte of the sum of the first eight bytes). TF-Luna can be
switched to other formats; restore it with Benewake's tool if needed.

## Readings (REP 117)

| Condition | `range` |
|---|---|
| valid, within `min_range`..`max_range` | distance [m] |
| strength < 100 (no return: too high, dark target), TFmini-S dist -1, or beyond `max_range` | `+inf` |
| closer than `min_range` (blind zone, e.g. on the ground) | `-inf` |
| strength 65535 (signal saturation), TFmini-S dist -2/-4, TF-Luna Amp > 32768 (sunlight) | `NaN` |

DemNav ignores readings that are non-finite, outside the message's
`min_range`/`max_range`, or older than `rangefinder_timeout_s` (0.5 s), and
falls back to altitude above the DEM. `publish_invalid:=false` drops
invalid readings instead of publishing them.

## Frames

`frame_id` defaults to `rangefinder_link`, the frame the simulator
(`applications/gazebo/scripts/aerodrone_sensor_sim.py`) uses. The beam is
the frame's +x axis, so the downward mount is `base_link` pitched +90 deg,
e.g. 5 cm below the centre:

```bash
ros2 run tf2_ros static_transform_publisher --z -0.05 --pitch 1.5708 \
  --frame-id base_link --child-frame-id rangefinder_link
```

## Parameters

| Parameter | Default | |
|---|---|---|
| `port` | `/dev/ttyUSB0` | serial device (`/dev/rangefinder` in `compositions/rangefinder.yaml`, `/host$RANGEFINDER_DEVICE` in `aerodrone_hardware.yaml`) |
| `baud_rate` | `115200` | |
| `model` | `tfmini_s` | `tfmini_s` or `tf_luna`: range limits and FOV |
| `topic` | `/aerodrone/rangefinder` | |
| `frame_id` | `rangefinder_link` | |
| `min_range`, `max_range`, `field_of_view` | `0` | override the model's datasheet values (m, rad) |
| `publish_invalid` | `true` | publish +inf/-inf/NaN readings |
| `stale_timeout_s` | `0.5` | warn when no valid frame arrives for this long |
| `reconnect_period_s` | `1.0` | retry period after the port fails or disappears |

Messages use the sensor-data QoS (best effort), like DemNav's subscription.

## Without hardware

`scripts/fake_benewake.py` streams frames on a pseudo-terminal:

```bash
docker compose -f compositions/rangefinder.yaml run --rm rangefinder_sim
ros2 topic echo /rangefinder_sim/range --field range   # 1.5, 2.0, 2.5, ... inf
```

Unit tests: `tests/test_rangefinder_benewake.py`.
