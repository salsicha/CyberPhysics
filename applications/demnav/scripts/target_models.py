#!/usr/bin/env python3
"""Tracking-target 3D models for the Aerodrone simulations.

Two steps, both driven by ``models`` in
``applications/aerostack2/missions/tracking_targets.json``:

``fetch`` (the ``target_assets`` service, needs the network) downloads each
pinned Gazebo Fuel model version, checks its SHA-256 and that Fuel still
publishes it under the licence the JSON names, and unpacks it into a cache
directory with a ``fuel.json`` record of where it came from. A model already
in the cache is not downloaded again, so later runs work offline.

``bake`` (called by build_georeferenced_terrain.py, offline) turns a cached
model into one simulator-neutral mesh: metres, z up, the model's front along
+x, its footprint centred on the origin and its lowest point at z = 0, and a
single texture atlas holding every material, so Gazebo, Genesis and Isaac
draw exactly the same thing from ``model.obj``/``model.mtl``/``<name>_atlas.png``
(and ``model_mesh.npz``, the same triangles as arrays, for Isaac). Wavefront
OBJ and static COLLADA (polylist/triangles/polygons, node transforms, skin
bind poses) are read; nothing else is needed for the Fuel models used.

Only numpy and OpenCV (texture resizing) besides the standard library.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import shutil
import sys
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ElementTree
import zipfile

import numpy as np

FUEL_API = "https://fuel.gazebosim.org/1.0"
ATLAS_SIZE = 2048
# Texels of padding around each atlas cell: mipmapped texture lookups near a
# cell's edge must not bleed into its neighbour.
ATLAS_GUTTER = 8


# --- fetch ------------------------------------------------------------------

def fuel_paths(fuel):
    """(owner, name) of a Fuel model reference 'Owner/models/Name'."""
    owner, kind, name = fuel.split("/", 2)
    if kind != "models":
        raise ValueError(f"not a Fuel model reference: {fuel}")
    return owner, name


def fuel_urls(fuel, version):
    owner, name = fuel_paths(fuel)
    base = f"{FUEL_API}/{urllib.parse.quote(owner)}/models/{urllib.parse.quote(name)}"
    return base, f"{base}/{int(version)}/{urllib.parse.quote(name)}.zip"


def _get(url, timeout_s=120.0):
    request = urllib.request.Request(url, headers={"User-Agent": "cyberphysics-target-assets"})
    with urllib.request.urlopen(request, timeout=timeout_s) as response:
        return response.read()


def fetch_model(key, spec, cache_dir, getter=_get):
    """Download one pinned Fuel model into cache_dir/key unless it is there.
    Returns its directory. Raises when the licence or checksum differ."""
    directory = Path(cache_dir) / key
    record_path = directory / "fuel.json"
    if record_path.is_file():
        record = json.loads(record_path.read_text(encoding="utf-8"))
        if (record.get("sha256") == spec["sha256"]
                and record.get("version") == spec["version"]):
            return directory
    metadata_url, zip_url = fuel_urls(spec["fuel"], spec["version"])
    metadata = json.loads(getter(metadata_url))
    licence = metadata.get("license_name")
    if licence != spec["license"]:
        raise RuntimeError(
            f"{spec['fuel']}: Fuel lists the licence {licence!r}, not {spec['license']!r}")
    payload = getter(zip_url)
    digest = hashlib.sha256(payload).hexdigest()
    if digest != spec["sha256"]:
        raise RuntimeError(
            f"{zip_url}: SHA-256 {digest} differs from the pinned {spec['sha256']}")
    temporary = directory.with_name(f".{key}.tmp")
    shutil.rmtree(temporary, ignore_errors=True)
    temporary.mkdir(parents=True)
    with zipfile.ZipFile(io.BytesIO(payload)) as archive:
        for member in archive.namelist():
            target = (temporary / member).resolve()
            if not str(target).startswith(str(temporary.resolve())):
                raise RuntimeError(f"{zip_url}: unsafe path {member}")
        archive.extractall(temporary)
    (temporary / "fuel.json").write_text(json.dumps({
        "fuel": spec["fuel"], "version": spec["version"], "url": zip_url,
        "sha256": digest, "license": licence,
        "license_url": metadata.get("license_url"),
        "description": metadata.get("description", ""),
    }, indent=2) + "\n", encoding="utf-8")
    shutil.rmtree(directory, ignore_errors=True)
    os.replace(temporary, directory)
    return directory


# --- mesh readers -----------------------------------------------------------

class Part:
    """Triangles of one material: per-corner positions, uvs and normals."""

    def __init__(self, material, positions, uvs, normals):
        self.material = material
        self.positions = np.asarray(positions, dtype=np.float64).reshape(-1, 3, 3)
        self.uvs = np.asarray(uvs, dtype=np.float64).reshape(-1, 3, 2)
        self.normals = (None if normals is None
                        else np.asarray(normals, dtype=np.float64).reshape(-1, 3, 3))


class Material:
    def __init__(self, name, color=(0.8, 0.8, 0.8), texture=None):
        self.name = name
        self.color = tuple(float(c) for c in color)
        self.texture = texture


def _fan(indices):
    """Triangle fan of one polygon's corner indices."""
    return [(indices[0], indices[i], indices[i + 1]) for i in range(1, len(indices) - 1)]


def read_obj(path):
    """(parts, materials) of a Wavefront OBJ and its MTL files."""
    path = Path(path)
    positions, uvs, normals = [], [], []
    faces = {}
    materials = {}
    current = "default"
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        words = line.split()
        if not words:
            continue
        if words[0] == "v":
            positions.append([float(v) for v in words[1:4]])
        elif words[0] == "vt":
            uvs.append([float(v) for v in words[1:3]])
        elif words[0] == "vn":
            normals.append([float(v) for v in words[1:4]])
        elif words[0] == "usemtl":
            current = " ".join(words[1:])
        elif words[0] == "mtllib":
            materials.update(read_mtl(path.parent / " ".join(words[1:])))
        elif words[0] == "f":
            corners = []
            for word in words[1:]:
                fields = (word.split("/") + ["", ""])[:3]
                corners.append(tuple(
                    (int(v) - 1 if int(v) > 0 else int(v)) if v else None for v in fields))
            faces.setdefault(current, []).extend(_fan(corners))
    positions = np.asarray(positions, dtype=np.float64)
    uvs = np.asarray(uvs, dtype=np.float64).reshape(-1, 2)
    normals = np.asarray(normals, dtype=np.float64).reshape(-1, 3)
    parts = []
    for name, triangles in faces.items():
        corners = [corner for triangle in triangles for corner in triangle]
        part_positions = positions[[c[0] for c in corners]]
        part_uvs = (uvs[[c[1] for c in corners]] if all(c[1] is not None for c in corners)
                    else np.zeros((len(corners), 2)))
        part_normals = (normals[[c[2] for c in corners]]
                        if all(c[2] is not None for c in corners) else None)
        parts.append(Part(name, part_positions, part_uvs, part_normals))
        materials.setdefault(name, Material(name))
    return parts, materials


def read_mtl(path):
    materials = {}
    current = None
    if not Path(path).is_file():
        return materials
    for line in Path(path).read_text(encoding="utf-8", errors="replace").splitlines():
        words = line.split()
        if not words:
            continue
        if words[0] == "newmtl":
            current = Material(" ".join(words[1:]))
            materials[current.name] = current
        elif current is not None and words[0] == "Kd":
            current.color = tuple(float(v) for v in words[1:4])
        elif current is not None and words[0] == "map_Kd":
            current.texture = words[-1]
    return materials


def _tag(element):
    return element.tag.rsplit("}", 1)[-1]


class Collada:
    """Static geometry of a COLLADA 1.4 document, in its own units and axes."""

    def __init__(self, path):
        self.path = Path(path)
        self.root = ElementTree.parse(path).getroot()
        self.ns = {"c": self.root.tag[1:].split("}")[0]} if self.root.tag.startswith("{") else {}
        self.ids = {element.get("id"): element for element in self.root.iter()
                    if element.get("id")}

    def find(self, element, path):
        return element.find(path, self.ns) if self.ns else element.find(path.replace("c:", ""))

    def findall(self, element, path):
        return (element.findall(path, self.ns) if self.ns
                else element.findall(path.replace("c:", "")))

    def ref(self, url):
        return self.ids.get(url.lstrip("#")) if url else None

    def unit_and_up(self):
        asset = self.find(self.root, "c:asset")
        meter, up = 1.0, "Y_UP"
        if asset is not None:
            unit = self.find(asset, "c:unit")
            if unit is not None and unit.get("meter"):
                meter = float(unit.get("meter"))
            axis = self.find(asset, "c:up_axis")
            if axis is not None and axis.text:
                up = axis.text.strip()
        return meter, up

    def source(self, url):
        source = self.ref(url)
        if _tag(source) == "vertices":
            for item in self.findall(source, "c:input"):
                if item.get("semantic") == "POSITION":
                    return self.source(item.get("source"))
        array = np.array(self.find(source, "c:float_array").text.split(), dtype=np.float64)
        accessor = self.find(source, "c:technique_common/c:accessor")
        stride = int(accessor.get("stride", "1"))
        return array.reshape(-1, stride)

    def primitives(self, geometry):
        """(material symbol, positions, uvs, normals) per primitive, per corner."""
        mesh = self.find(geometry, "c:mesh")
        for primitive in list(mesh):
            kind = _tag(primitive)
            if kind not in ("triangles", "polylist", "polygons"):
                continue
            inputs = self.findall(primitive, "c:input")
            stride = max(int(item.get("offset", "0")) for item in inputs) + 1
            if kind == "polygons":
                polygons = [np.array(p.text.split(), dtype=np.int64).reshape(-1, stride)
                            for p in self.findall(primitive, "c:p")]
            else:
                indices = np.array(self.find(primitive, "c:p").text.split(),
                                   dtype=np.int64).reshape(-1, stride)
                if kind == "triangles":
                    counts = np.full(len(indices) // 3, 3)
                else:
                    counts = np.array(self.find(primitive, "c:vcount").text.split(),
                                      dtype=np.int64)
                polygons = np.split(indices, np.cumsum(counts)[:-1])
            corners = np.array([corner for polygon in polygons
                                for triangle in _fan(list(polygon)) for corner in triangle])
            data = {}
            for item in inputs:
                semantic = item.get("semantic")
                if semantic == "TEXCOORD" and item.get("set", "0") not in ("0", None):
                    if "TEXCOORD" in data:
                        continue
                if semantic in ("VERTEX", "NORMAL", "TEXCOORD") and semantic not in data:
                    values = self.source(item.get("source"))
                    data[semantic] = values[corners[:, int(item.get("offset", "0"))]]
            yield (primitive.get("material"), data["VERTEX"][:, :3],
                   data["TEXCOORD"][:, :2] if "TEXCOORD" in data else np.zeros((len(corners), 2)),
                   data["NORMAL"][:, :3] if "NORMAL" in data else None)

    @staticmethod
    def node_matrix(node):
        matrix = np.eye(4)
        for element in list(node):
            kind = _tag(element)
            values = [float(v) for v in (element.text or "").split()]
            if kind == "matrix":
                matrix = matrix @ np.array(values).reshape(4, 4)
            elif kind == "translate":
                step = np.eye(4)
                step[:3, 3] = values
                matrix = matrix @ step
            elif kind == "scale":
                matrix = matrix @ np.diag(values + [1.0])
            elif kind == "rotate":
                axis = np.array(values[:3]) / np.linalg.norm(values[:3])
                angle = math.radians(values[3])
                k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]],
                              [-axis[1], axis[0], 0]])
                step = np.eye(4)
                step[:3, :3] = np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k
                matrix = matrix @ step
        return matrix

    def material(self, url):
        """Material of a COLLADA material id: diffuse colour or texture file."""
        material = self.ref(url)
        name = material.get("name") or material.get("id")
        effect = self.ref(self.find(material, "c:instance_effect").get("url"))
        result = Material(name)
        diffuse = None
        for element in effect.iter():
            if _tag(element) == "diffuse":
                diffuse = element
                break
        if diffuse is None:
            return result
        for element in list(diffuse):
            kind = _tag(element)
            if kind == "color":
                result.color = tuple(float(v) for v in element.text.split()[:3])
            elif kind == "texture":
                result.texture = self.image_file(effect, element.get("texture"))
        return result

    def image_file(self, effect, sampler_sid):
        params = {param.get("sid"): param for param in effect.iter()
                  if _tag(param) == "newparam"}
        image_id = sampler_sid
        param = params.get(sampler_sid)
        if param is not None:
            source = next((e for e in param.iter() if _tag(e) == "source"), None)
            surface = params.get(source.text.strip()) if source is not None else None
            if surface is not None:
                init = next((e for e in surface.iter() if _tag(e) == "init_from"), None)
                if init is not None:
                    image_id = init.text.strip()
        image = self.ids.get(image_id)
        if image is None:
            return None
        init = next((e for e in image.iter() if _tag(e) == "init_from"), None)
        return urllib.parse.unquote(init.text.strip()) if init is not None else None

    def parts(self):
        meter, up = self.unit_and_up()
        to_z_up = np.eye(4)
        if up == "Y_UP":
            to_z_up[:3, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
        elif up == "X_UP":
            to_z_up[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        base = to_z_up @ np.diag([meter, meter, meter, 1.0])
        scene_url = self.find(self.root, "c:scene/c:instance_visual_scene").get("url")
        parts, materials = [], {}

        def visit(node, matrix):
            matrix = matrix @ self.node_matrix(node)
            for instance in list(node):
                kind = _tag(instance)
                if kind not in ("instance_geometry", "instance_controller"):
                    continue
                transform = matrix
                geometry = self.ref(instance.get("url"))
                if kind == "instance_controller":
                    skin = self.find(geometry, "c:skin")
                    bind = self.find(skin, "c:bind_shape_matrix")
                    if bind is not None:
                        transform = matrix @ np.array(bind.text.split(), dtype=float).reshape(4, 4)
                    geometry = self.ref(skin.get("source"))
                bindings = {item.get("symbol"): item.get("target")
                            for item in instance.iter() if _tag(item) == "instance_material"}
                normal_matrix = np.linalg.inv(transform[:3, :3]).T
                for symbol, positions, uvs, normals in self.primitives(geometry):
                    target = bindings.get(symbol, symbol)
                    if target not in materials:
                        materials[target] = (self.material(target) if self.ref(target) is not None
                                             else Material(target or "default"))
                    world = positions @ transform[:3, :3].T + transform[:3, 3]
                    if normals is not None:
                        normals = normals @ normal_matrix.T
                    parts.append(Part(materials[target].name, world, uvs, normals))
            for child in list(node):
                if _tag(child) == "node":
                    visit(child, matrix)

        for node in list(self.ref(scene_url)):
            if _tag(node) == "node":
                visit(node, base)
        return parts, {material.name: material for material in materials.values()}


def read_model(path):
    """(parts, materials) of an OBJ or COLLADA file."""
    suffix = Path(path).suffix.lower()
    if suffix == ".obj":
        return read_obj(path)
    if suffix == ".dae":
        return Collada(path).parts()
    raise ValueError(f"unsupported mesh format: {path}")


# --- normalisation and atlas ------------------------------------------------

def rotation_matrix(rpy_deg):
    roll, pitch, yaw = (math.radians(v) for v in rpy_deg)
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def normalise(parts, scale=1.0, rpy_deg=(0.0, 0.0, 0.0)):
    """Scale and rotate the parts, then centre their footprint on the
    origin and put their lowest point at z = 0. Returns the bounding box
    size (x, y, z) in metres."""
    rotation = rotation_matrix(rpy_deg)
    for part in parts:
        part.positions = (part.positions * scale) @ rotation.T
        if part.normals is not None:
            part.normals = part.normals @ rotation.T
    every = np.concatenate([part.positions.reshape(-1, 3) for part in parts])
    low, high = every.min(axis=0), every.max(axis=0)
    shift = np.array([-(low[0] + high[0]) / 2.0, -(low[1] + high[1]) / 2.0, -low[2]])
    for part in parts:
        part.positions = part.positions + shift
    return high - low


def face_normals(positions):
    normal = np.cross(positions[:, 1] - positions[:, 0], positions[:, 2] - positions[:, 0])
    length = np.linalg.norm(normal, axis=1, keepdims=True)
    return np.repeat(normal / np.where(length > 0, length, 1.0), 3, axis=0).reshape(-1, 3, 3)


def load_texture(directory, reference, search_dirs):
    """BGR texture image of a material reference, or None. Fuel MTLs name
    textures by URL or by paths relative to the mesh; the file is looked up
    by name in the model's directories."""
    import cv2

    if not reference:
        return None
    name = Path(urllib.parse.urlparse(reference).path).name
    candidates = [Path(directory) / reference] + [Path(d) / name for d in search_dirs]
    for candidate in candidates:
        if candidate.is_file():
            image = cv2.imread(str(candidate), cv2.IMREAD_UNCHANGED)
            if image is None:
                continue
            if image.ndim == 2:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGRA)
            if image.shape[2] == 3:
                image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
            return image
    return None


def flatten_alpha(image):
    """Opaque BGR: texels the material leaves transparent (hair cards,
    lashes) take the mean colour of its opaque texels, as a cut-out seen
    from tens of metres would."""
    bgr = image[..., :3].astype(np.float64)
    alpha = image[..., 3]
    opaque = alpha >= 128
    if opaque.any() and not opaque.all():
        bgr[~opaque] = bgr[opaque].mean(axis=0)
    return bgr.astype(np.uint8)


def build_atlas(cells, size=ATLAS_SIZE, gutter=ATLAS_GUTTER):
    """Pack square material images into one atlas. cells: list of BGR
    images. Returns (atlas, [(x0, y0, cell) per image])."""
    import cv2

    count = max(1, len(cells))
    grid = int(math.ceil(math.sqrt(count)))
    cell = size // grid
    atlas = np.zeros((size, size, 3), dtype=np.uint8)
    placements = []
    for index, image in enumerate(cells):
        x0, y0 = (index % grid) * cell, (index // grid) * cell
        inner = cell - 2 * gutter
        resized = cv2.resize(image, (inner, inner), interpolation=cv2.INTER_AREA)
        # Replicate the border into the gutter.
        padded = cv2.copyMakeBorder(resized, gutter, gutter, gutter, gutter,
                                    cv2.BORDER_REPLICATE)
        atlas[y0:y0 + cell, x0:x0 + cell] = padded
        placements.append((x0 + gutter, y0 + gutter, inner))
    return atlas, placements


def atlas_uvs(uvs, placement, size=ATLAS_SIZE):
    """Material UVs (v up, wrapped into [0, 1]) to the atlas cell."""
    x0, y0, inner = placement
    wrapped = uvs - np.floor(uvs)
    # A coordinate of exactly 1.0 means the far edge, not a wrap to 0.
    wrapped = np.where((uvs >= 1.0) & (wrapped == 0.0) & (uvs == np.round(uvs)), 1.0, wrapped)
    half = 0.5
    u = (x0 + half + wrapped[..., 0] * (inner - 1)) / size
    v = 1.0 - (y0 + half + (1.0 - wrapped[..., 1]) * (inner - 1)) / size
    return np.stack([u, v], axis=-1)


def index_corners(positions, uvs, normals):
    """Share the vertices of triangle corners with the same position, UV
    and normal: (positions, uvs, normals, triangles (F, 3) int32)."""
    keys = np.round(np.column_stack([positions * 1e5, uvs * 1e6, normals * 1e4])).astype(np.int64)
    _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    # Keep the vertices in their first-use order.
    order = np.argsort(first)
    rank = np.empty_like(order)
    rank[order] = np.arange(len(order))
    keep = first[order]
    triangles = rank[np.asarray(inverse).reshape(-1)].reshape(-1, 3).astype(np.int32)
    return positions[keep], uvs[keep], normals[keep], triangles


def atlas_file(name):
    """File name of model `name`'s texture atlas, next to its model.obj."""
    return f"{name}_atlas.png"


def bake(spec, source_dir, output_dir, name):
    """Write the normalised, single-atlas model `name` into output_dir:
    model.obj, model.mtl, <name>_atlas.png, model_mesh.npz, model.sdf (a
    visual-only, gravity-free Gazebo model moved with set_pose),
    model.config and ATTRIBUTION.txt. Returns a summary dict."""
    import cv2

    source_dir = Path(source_dir)
    mesh_path = source_dir / spec["mesh"]
    parts, materials = read_model(mesh_path)
    excluded = [re.compile(pattern, re.IGNORECASE)
                for pattern in spec.get("exclude_materials", [])]
    parts = [part for part in parts
             if not any(pattern.search(part.material) for pattern in excluded)]
    if not parts:
        raise RuntimeError(f"{mesh_path}: no geometry left")
    size = normalise(parts, float(spec.get("scale", 1.0)),
                     spec.get("rotate_rpy_deg", (0.0, 0.0, 0.0)))
    overrides = spec.get("material_colors", {})
    search = [mesh_path.parent, source_dir / "materials" / "textures", source_dir]
    names = []
    for part in parts:
        if part.material not in names:
            names.append(part.material)
    cells = []
    for material_name in names:
        material = materials.get(material_name, Material(material_name))
        override = next((color for pattern, color in overrides.items()
                         if re.search(pattern, material_name, re.IGNORECASE)), None)
        texture = None if override else load_texture(
            mesh_path.parent, material.texture, search)
        if texture is not None:
            image = flatten_alpha(texture)
        else:
            color = override or material.color
            image = np.zeros((16, 16, 3), dtype=np.uint8)
            image[:] = [int(round(255 * min(max(c, 0.0), 1.0))) for c in color[2::-1]]
        cells.append(image)
    atlas, placements = build_atlas(cells)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    texture = atlas_file(name)
    if not cv2.imwrite(str(output_dir / texture), atlas):
        raise RuntimeError(f"failed to write {output_dir / texture}")

    positions = np.concatenate([part.positions for part in parts]).reshape(-1, 3)
    uvs = np.concatenate([atlas_uvs(part.uvs, placements[names.index(part.material)])
                          for part in parts]).reshape(-1, 2)
    normals = np.concatenate([
        part.normals if part.normals is not None else face_normals(part.positions)
        for part in parts]).reshape(-1, 3)
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = normals / np.where(lengths > 0, lengths, 1.0)
    positions, uvs, normals, triangles = index_corners(positions, uvs, normals)
    np.savez_compressed(output_dir / "model_mesh.npz", positions=positions.astype(np.float32),
                        uvs=uvs.astype(np.float32), normals=normals.astype(np.float32),
                        triangles=triangles)
    # Gazebo shares materials and textures between meshes by name: each
    # model's must be its own, or every model is drawn with the first one's.
    material_name = f"{name}_atlas"
    lines = ["# Baked by applications/demnav/scripts/target_models.py", "mtllib model.mtl",
             f"usemtl {material_name}"]
    lines += [f"v {x:.5f} {y:.5f} {z:.5f}" for x, y, z in positions]
    lines += [f"vt {u:.6f} {v:.6f}" for u, v in uvs]
    lines += [f"vn {x:.4f} {y:.4f} {z:.4f}" for x, y, z in normals]
    lines += [f"f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}"
              for a, b, c in (triangles + 1).tolist()]
    (output_dir / "model.obj").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (output_dir / "model.mtl").write_text(
        f"newmtl {material_name}\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nd 1\nillum 1\nmap_Kd {texture}\n",
        encoding="utf-8")
    (output_dir / "model.config").write_text(f"""<?xml version="1.0"?>
<model><name>{name}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>
<description>Tracking target baked from Gazebo Fuel {spec['fuel']}
version {spec['version']} ({spec['license']})</description></model>
""", encoding="utf-8")
    # Visual only: no collision, so nothing touches the drone, and no
    # gravity, so the model stays where set_pose puts it.
    (output_dir / "model.sdf").write_text(f"""<?xml version="1.0"?>
<sdf version="1.9"><model name="{name}"><static>false</static>
<link name="body"><gravity>false</gravity>
<inertial><mass>1.0</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
<visual name="visual"><cast_shadows>true</cast_shadows><geometry><mesh><uri>model://{name}/model.obj</uri></mesh></geometry></visual>
</link></model></sdf>
""", encoding="utf-8")
    record_path = source_dir / "fuel.json"
    record = json.loads(record_path.read_text(encoding="utf-8")) if record_path.is_file() else {}
    (output_dir / "ATTRIBUTION.txt").write_text(
        f"{spec['fuel']} version {spec['version']} from Gazebo Fuel ({record.get('url', '')}),\n"
        f"licence: {spec['license']} ({record.get('license_url', '')}).\n"
        "Converted to a single-texture mesh for the CyberPhysics Aerodrone simulations.\n",
        encoding="utf-8")
    return {"name": name, "triangles": int(len(triangles)),
            "size_m": [round(float(v), 3) for v in size], "materials": names}


# --- command line -----------------------------------------------------------

def load_targets(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    fetch = sub.add_parser("fetch", help="download the pinned Fuel models")
    fetch.add_argument("--targets", required=True)
    fetch.add_argument("--cache-dir", required=True)
    bake_parser = sub.add_parser("bake", help="bake every model for inspection")
    bake_parser.add_argument("--targets", required=True)
    bake_parser.add_argument("--cache-dir", required=True)
    bake_parser.add_argument("--output-dir", required=True)
    args = parser.parse_args(argv)
    config = load_targets(args.targets)
    for key, spec in config["models"].items():
        directory = fetch_model(key, spec, args.cache_dir)
        print(f"{key}: {spec['fuel']} v{spec['version']} ({spec['license']}) in {directory}",
              flush=True)
        if args.command == "bake":
            print(json.dumps(bake(spec, directory, Path(args.output_dir) / key, key)), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
