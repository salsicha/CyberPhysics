"""Headless PNG renders of meshes for part previews and assembly figures.

Uses matplotlib only (no OpenGL), so it runs in the cad image and in CI.
Each item is ``(trimesh.Trimesh, "#rrggbb")``; faces are flat-shaded from
one light, painter-sorted by matplotlib.
"""
import numpy as np

MATERIAL_COLORS = {
    "PETG": "#4c9be8",
    "ASA": "#e8833a",
    "TPU95A": "#3d3d3d",
    "PETG-CF": "#5a5a66",
    "PA-CF": "#4a4a52",
}
PURCHASED_COLOR = "#b9bec6"


def _shade(normals, rgb, light=(0.35, -0.55, 0.75)):
    light = np.asarray(light, float)
    light /= np.linalg.norm(light)
    lam = np.clip(normals @ light, 0.0, 1.0)
    k = 0.45 + 0.55 * lam
    return np.clip(np.outer(k, rgb), 0.0, 1.0)


def render(items, path, *, elev=24.0, azim=-58.0, width_px=800, height_px=600,
           title=None, labels=(), max_faces=60000, dpi=100, zoom=1.0):
    """Render coloured meshes to a PNG.

    ``labels`` is a sequence of (xyz, text) annotations in model units.
    Large meshes are decimated by face sampling to ``max_faces`` in total.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import to_rgb
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    total = sum(len(m.faces) for m, _ in items) or 1
    keep = min(1.0, max_faces / total)
    rng = np.random.default_rng(0)

    fig = plt.figure(figsize=(width_px / dpi, height_px / dpi), dpi=dpi)
    ax = fig.add_subplot(111, projection="3d", proj_type="ortho")
    ax.view_init(elev=elev, azim=azim)
    lo = np.array([np.inf] * 3)
    hi = -lo
    tris, cols = [], []
    for mesh, color in items:
        faces = np.arange(len(mesh.faces))
        if keep < 1.0 and len(faces) > 200:
            faces = faces[rng.random(len(faces)) < keep]
        tris.append(mesh.triangles[faces])
        cols.append(_shade(mesh.face_normals[faces], np.array(to_rgb(color))))
        lo = np.minimum(lo, mesh.bounds[0])
        hi = np.maximum(hi, mesh.bounds[1])
    # One collection, so matplotlib depth-sorts every face against every other.
    ax.add_collection3d(Poly3DCollection(np.concatenate(tris), facecolors=np.concatenate(cols),
                                         edgecolors="none", linewidths=0, antialiased=False))
    ext = np.maximum(hi - lo, 1e-6)
    ext = np.maximum(ext, ext.max() * 0.15)   # keep very flat parts visible
    center = (lo + hi) / 2
    ax.set_xlim(center[0] - ext[0] / 2, center[0] + ext[0] / 2)
    ax.set_ylim(center[1] - ext[1] / 2, center[1] + ext[1] / 2)
    ax.set_zlim(center[2] - ext[2] / 2, center[2] + ext[2] / 2)
    ax.set_box_aspect(tuple(ext), zoom=zoom)
    ax.set_axis_off()
    for xyz, text in labels:
        ax.text(*xyz, text, fontsize=8, color="#222222", ha="center",
                bbox={"boxstyle": "round,pad=0.2", "fc": "white", "ec": "#888888", "lw": 0.5})
    if title:
        fig.suptitle(title, fontsize=10, y=0.97)
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    fig.savefig(path, dpi=dpi, facecolor="white")
    plt.close(fig)
    _compress_png(path)


def _compress_png(path):
    """Quantise to a 64-colour palette: renders are flat-shaded, so this
    keeps them small without visible loss."""
    try:
        from PIL import Image
    except ImportError:
        return
    img = Image.open(path).convert("RGB")
    img = img.quantize(colors=64, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
    img.save(path, optimize=True)
