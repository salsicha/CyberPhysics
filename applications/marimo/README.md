# Marimo

Containerized [Marimo](https://marimo.io/) notebook server with
[ArrayDataEngine](https://github.com/salsicha/ArrayDataEngine) and
[Open3D](https://www.open3d.org/) installed for robotics data exploration.

The container starts Marimo in edit mode on port `2718` and uses `/notebooks` as its workspace.

## Build

From the repository root:

```bash
make -C applications build_marimo
```

The image includes:

- `arraydataengine==0.3.1`
- `open3d==0.19.0`
- Marimo with its recommended dependencies

Open3D adds a large binary wheel, so expect this image to be significantly
larger than the base Marimo image.

## Run

```bash
docker compose -f compositions/marimo.yaml up
```

Open:

```text
http://localhost:2718
```

By default the compose service mounts `applications/marimo/notebooks` into
`/notebooks`. Changes made in Marimo are therefore saved back into the
repository.

## LiDAR example

Open `lidar_arraydataengine_open3d.py` from the Marimo home page. The notebook:

1. loads deterministic sample LiDAR frames with ArrayDataEngine;
2. removes invalid and zero-padded points;
3. converts the selected frame into an Open3D point cloud; and
4. renders an interactive 3D Plotly view in Marimo.

The example runs without external data. To load your own LiDAR topic, save it
in ArrayDataEngine's portable topic format and enter its container path in the
notebook:

```python
from arraydataengine.ops import save_topic_npz

save_topic_npz("/notebooks/my_lidar_topic.npz", buffered_topic)
```

Files placed in `applications/marimo/notebooks` are available under
`/notebooks` in the container. The saved point-cloud data must have shape
`(frames, points, 3)` or more than three columns, with XYZ in the first three.
