import marimo

__generated_with = "0.23.16"
app = marimo.App(width="medium")


@app.cell
def _():
    import marimo as mo
    import numpy as np
    import open3d as o3d
    from pathlib import Path

    from arraydataengine.ops import load_topic_npz, to_open3d_point_cloud
    from arraydataengine.sources.synthetic_source import SyntheticSource
    from open3d.visualization.draw_plotly import get_plotly_fig

    return (
        Path,
        SyntheticSource,
        get_plotly_fig,
        load_topic_npz,
        mo,
        np,
        to_open3d_point_cloud,
    )


@app.cell
def _(mo):
    mo.md(r"""
    # LiDAR with ArrayDataEngine and Open3D

    This notebook loads point-cloud frames with ArrayDataEngine, converts a
    selected frame to an Open3D `PointCloud`, and displays it as an
    interactive Plotly scene. Rotate with the left mouse button, pan with
    the right mouse button, and scroll to zoom.

    Leave the path empty to use ArrayDataEngine's deterministic synthetic
    LiDAR source. To use your own data, enter the path to an `.npz` file
    created by `arraydataengine.ops.save_topic_npz`. Paths are inside the
    container, so a file placed beside this notebook is under `/notebooks`.
    """)
    return


@app.cell
def _(mo):
    lidar_path = mo.ui.text(
        value="",
        placeholder="/notebooks/my_lidar_topic.npz",
        label="Optional ArrayDataEngine .npz path",
        full_width=True,
    )
    lidar_path
    return (lidar_path,)


@app.cell
def _(Path, SyntheticSource, lidar_path, load_topic_npz, np):
    requested_path = lidar_path.value.strip()
    requested_file = Path(requested_path).expanduser() if requested_path else None

    if requested_file is not None and requested_file.is_file():
        lidar_topic = load_topic_npz(requested_file)
        lidar_frames = np.asarray(lidar_topic["data"])
        if lidar_frames.ndim == 2:
            lidar_frames = lidar_frames[np.newaxis, ...]
        if lidar_frames.ndim != 3 or lidar_frames.shape[-1] < 3:
            raise ValueError(
                "Expected LiDAR data with shape (frames, points, >=3); "
                f"got {lidar_frames.shape}."
            )
        timestamps = np.asarray(
            lidar_topic.get("ts", np.arange(len(lidar_frames))), dtype=float
        )
        source_note = f"Loaded `{requested_file}`."
    else:
        source = SyntheticSource(
            topics=[
                {
                    "name": "/lidar/points",
                    "kind": "pointcloud",
                    "rate": 5.0,
                    "points": 160,
                    "max_range": 30.0,
                }
            ],
            duration=4.0,
            radius=8.0,
            seed=7,
        )
        lidar_messages = list(source.get_message())
        lidar_frames = np.stack([message["data"] for message in lidar_messages])
        timestamps = np.asarray(
            [message["timestamp"] for message in lidar_messages], dtype=float
        )
        if requested_file is None:
            source_note = "Using ArrayDataEngine's synthetic LiDAR source."
        else:
            source_note = (
                f"`{requested_file}` was not found; using the synthetic LiDAR source."
            )
    return lidar_frames, source_note, timestamps


@app.cell
def _(lidar_frames, mo):
    frame_index = mo.ui.slider(
        start=0,
        stop=len(lidar_frames) - 1,
        step=1,
        value=0,
        label="LiDAR frame",
        show_value=True,
        full_width=True,
    )
    frame_index
    return (frame_index,)


@app.cell
def _(frame_index, lidar_frames, np, to_open3d_point_cloud):
    raw_points = np.asarray(lidar_frames[frame_index.value], dtype=float)
    xyz = raw_points[:, :3]

    # SyntheticSource pads fixed-size scans with zero rows. The same cleanup is
    # useful for many exported sensor buffers.
    valid = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 1e-9)
    points = xyz[valid]
    if len(points) == 0:
        raise ValueError("The selected frame contains no finite, non-zero points.")

    height = points[:, 2]
    height_span = float(np.ptp(height))
    normalized_height = (
        (height - height.min()) / height_span
        if height_span > 0.0
        else np.zeros_like(height)
    )
    colors = np.column_stack(
        (
            normalized_height,
            0.35 + 0.45 * (1.0 - normalized_height),
            1.0 - normalized_height,
        )
    )

    point_cloud = to_open3d_point_cloud(points, colors=colors)
    return point_cloud, points


@app.cell
def _(frame_index, get_plotly_fig, point_cloud, timestamps):
    lidar_figure = get_plotly_fig(
        [point_cloud],
        width=900,
        height=600,
        point_sample_factor=1,
        up=[0.0, 0.0, 1.0],
    )
    lidar_figure.update_layout(
        title=f"LiDAR frame {frame_index.value} at {timestamps[frame_index.value]:.2f} s",
        scene={
            "aspectmode": "data",
            "xaxis_title": "x (m)",
            "yaxis_title": "y (m)",
            "zaxis_title": "z (m)",
        },
        margin={"l": 0, "r": 0, "t": 45, "b": 0},
    )
    lidar_figure
    return


@app.cell
def _(frame_index, lidar_frames, mo, points, source_note, timestamps):
    mo.md(
        f"""
        {source_note}

        - Frames: **{len(lidar_frames)}**
        - Selected timestamp: **{timestamps[frame_index.value]:.2f} s**
        - Valid points in selected frame: **{len(points)}**
        """
    )
    return


if __name__ == "__main__":
    app.run()
