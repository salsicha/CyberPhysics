#!/usr/bin/env python3
"""Assemble printable STLs and purchased CAD envelopes for both simulators.

Run in the CAD image. Masses are the documented BOM/print estimates, not a
scale measurement. Inertia assumes each item's mass is uniform in its CAD
envelope; wiring allowances are point masses at their documented positions.
"""
import argparse
import hashlib
import importlib
import json
import math
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import cadquery as cq
import numpy as np
import trimesh

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / 'tools'))
from hardware.parts.aerodrone import assembly as A, _frame as F
from build_printable_parts import print_oriented


def write_sdf(directory, data):
    def child(parent, tag, text=None, **attributes):
        element = ET.SubElement(parent, tag, attributes)
        if text is not None:
            element.text = str(text)
        return element

    def vector(values):
        return ' '.join(f'{v:.12g}' for v in values)

    def inertial(link, mass, tensor, center=(0, 0, 0)):
        node = child(link, 'inertial')
        child(node, 'pose', vector([*center, 0, 0, 0]))
        child(node, 'mass', f'{mass:.12g}')
        matrix = child(node, 'inertia')
        for name, i, j in [('ixx', 0, 0), ('iyy', 1, 1), ('izz', 2, 2),
                           ('ixy', 0, 1), ('ixz', 0, 2), ('iyz', 1, 2)]:
            child(matrix, name, f'{tensor[i][j]:.12g}')

    sdf = ET.Element('sdf', version='1.6')
    model = child(sdf, 'model', name='aerodrone_airframe')
    base = child(model, 'link', name='base_link')
    # Tiny inertias make sensor and rotor frames valid SDF links; all real
    # hardware mass is already included in the assembly tensor.
    inertial(base, data['mass_kg'] - 7e-6, data['inertia_kg_m2'], data['center_of_mass_m'])
    visual = child(base, 'visual', name='assembled_prints_and_hardware')
    child(child(child(visual, 'geometry'), 'mesh'), 'uri', 'model://aerodrone/airframe.stl')
    material = child(visual, 'material')
    child(material, 'ambient', '.18 .22 .3 1')
    child(material, 'diffuse', '.18 .22 .3 1')
    for box in data['collision_boxes']:
        collision = child(base, 'collision', name=box['name'])
        child(collision, 'pose', vector([*box['center_m'], 0, 0, 0]))
        child(child(child(collision, 'geometry'), 'box'), 'size', vector(box['size_m']))
    for i, position in enumerate(data['rotor_positions_flu_m']):
        link = child(model, 'link', name=f'rotor_{i}')
        child(link, 'pose', vector([*position, 0, 0, 0]), relative_to='base_link')
        inertial(link, 1e-6, np.eye(3) * 1e-10)
        joint = child(model, 'joint', name=f'rotor_{i}_joint', type='revolute')
        child(joint, 'parent', 'base_link')
        child(joint, 'child', f'rotor_{i}')
        child(child(joint, 'axis'), 'xyz', '0 0 1')
    imu = child(model, 'model', name='imu')
    link = child(imu, 'link', name='internal')
    inertial(link, 1e-6, np.eye(3) * 1e-10)
    sensor = child(link, 'sensor', name='imu', type='imu')
    child(sensor, 'always_on', '1')
    child(sensor, 'update_rate', '250')
    joint = child(model, 'joint', name='imu_joint', type='fixed')
    child(joint, 'parent', 'base_link')
    child(joint, 'child', 'imu::internal')
    ET.indent(sdf)
    ET.ElementTree(sdf).write(directory / 'model.sdf', encoding='unicode', xml_declaration=True)
    (directory / 'model.config').write_text('<model><name>aerodrone_airframe</name><version>1</version><sdf version="1.6">model.sdf</sdf></model>\n')


def rigid_transform(source, placed):
    """Recover the assembly's rigid placement from matching CAD vertices."""
    a = np.array([v.Center().toTuple() for v in source.Vertices()])
    b = np.array([v.Center().toTuple() for v in placed.Vertices()])
    if a.shape != b.shape or len(a) < 3:
        raise ValueError('part topology changed between source and assembly')
    u, _, vt = np.linalg.svd((a - a.mean(0)).T @ (b - b.mean(0)))
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ correction @ u.T
    translation = b.mean(0) - rotation @ a.mean(0)
    if np.max(np.linalg.norm(a @ rotation.T + translation - b, axis=1)) > 1e-5:
        raise ValueError('CAD vertices do not describe one rigid placement')
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = rotation, translation
    return matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=REPO / 'applications/aerostack2/models/aerodrone')
    args = parser.parse_args()
    entries = {e['id']: e for e in json.loads((REPO / 'hardware/parts/aerodrone/stl/manifest.json').read_text())['parts']}
    base = (A.printed(), A._purchased(), A.launcher_printed(), A.launcher_purchased())
    config = A.launcher_configuration(entries, base=base)
    shapes = dict(config['bodies'])
    parts = {n: ('aerodrone', stem, s) for n, stem, s in base[0] + base[2] if n in shapes}
    parts.update({n: ('turret', 'launcher_' + stem, s) for n, stem, s in config['models'] if stem})
    meshes, sources, built = [], {}, {}
    for name, placed in config['bodies']:
        if name in parts:
            platform, stem, _ = parts[name]
            key = (platform, stem)
            if key not in built:
                module = importlib.import_module(f'hardware.parts.{platform}.{stem}')
                source = module.build()
                oriented = print_oriented(source, module.PART['print']['rotate'])
                path = REPO / f'hardware/parts/{platform}/stl/{stem}.stl'
                sources[str(path.relative_to(REPO))] = hashlib.sha256(path.read_bytes()).hexdigest()
                built[key] = (source, rigid_transform(oriented, source), trimesh.load(path, force='mesh'))
            source, undo_print, original = built[key]
            mesh = original.copy()
            try:
                mesh.apply_transform(rigid_transform(source, placed) @ undo_print)
            except ValueError as error:
                raise ValueError(f'{name}: {error}') from error
        else:
            vertices, faces = placed.tessellate(0.25, 0.2)
            mesh = trimesh.Trimesh([v.toTuple() for v in vertices], faces, process=False)
        mesh.apply_scale(.001)
        meshes.append(mesh)
    mesh = trimesh.util.concatenate(meshes)
    mass_g, cg_mm = A.centre_of_mass(config['items'])
    cg = np.array(cg_mm) * .001
    inertia = np.zeros((3, 3))
    items = []
    for name, mass, position in config['items']:
        if mass <= 0:
            raise ValueError(f'missing mass estimate: {name}')
        kilograms = mass * .001
        shape = shapes.get(name)
        intrinsic = (np.array(cq.Shape.matrixOfInertia(shape)) * kilograms / shape.Volume() * 1e-6
                     if shape is not None else np.zeros((3, 3)))
        displacement = np.array(position) * .001 - cg
        inertia += intrinsic + kilograms * (np.dot(displacement, displacement) * np.eye(3) - np.outer(displacement, displacement))
        items.append({'name': name, 'mass_kg': kilograms, 'center_m': (np.array(position) * .001).tolist()})
    # Conservative CAD-aligned contact envelopes, excluding tiny fasteners.
    collisions = []
    prefixes = ('hub_', 'arm_tube_', 'motor_', 'tall_leg_foot_', 'battery', 'launcher_barrel', 'launcher_breech_down')
    for name, shape in config['bodies']:
        if name.startswith(prefixes):
            b = shape.BoundingBox()
            collisions.append({'name': name, 'center_m': [(b.xmin + b.xmax) / 2000, (b.ymin + b.ymax) / 2000, (b.zmin + b.zmax) / 2000],
                               'size_m': [b.xlen / 1000, b.ylen / 1000, b.zlen / 1000]})
    max_rpm = A.THRUST_TABLE[-1][3]
    max_speed = max_rpm * 2 * math.pi / 60
    rotor_positions = [[F.MOTOR_R * math.cos(math.radians(angle)) / 1000,
                        F.MOTOR_R * math.sin(math.radians(angle)) / 1000, F.PROP_Z / 1000]
                       for angle, _ in F.MOTORS.values()]
    manifest = {'configuration': 'launcher, full clip plus chambered dart',
                'mass_basis': 'BOM and shell/infill estimates; not physically weighed',
                'inertia_basis': 'uniform item CAD envelopes plus point wiring allowances',
                'mass_kg': mass_g / 1000, 'center_of_mass_m': cg.tolist(),
                'inertia_kg_m2': inertia.tolist(), 'bounds_m': mesh.bounds.tolist(),
                'ground_clearance_m': -float(mesh.bounds[0, 2]),
                'camera_position_flu_m': (np.array(A.lens_point()) / 1000).tolist(),
                'rotor_positions_flu_m': rotor_positions, 'rotor_yaw_sign': [-1, -1, 1, 1],
                'max_rotor_speed_rad_s': max_speed,
                'motor_constant': A.THRUST_TABLE[-1][1] * .001 * 9.80665 / max_speed ** 2,
                'moment_constant': .016, 'moment_constant_basis': 'retained simulation estimate; not measured',
                'collision_boxes': collisions, 'items': items, 'stl_sha256': sources}
    inputs = [REPO / 'tools/build_aerodrone_sim_assets.py',
              REPO / 'tools/build_printable_parts.py']
    inputs.extend((REPO / 'hardware/parts/common').glob('*.py'))
    for platform in ('aerodrone', 'turret'):
        inputs.extend((REPO / 'hardware/parts' / platform).glob('*.py'))
        inputs.append(REPO / 'hardware/parts' / platform / 'stl/manifest.json')
    manifest['source_sha256'] = {
        str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(inputs)
    }
    assert np.linalg.eigvalsh(inertia).min() > 0
    args.out.mkdir(parents=True, exist_ok=True)
    mesh.export(args.out / 'airframe.stl')
    (args.out / 'airframe.json').write_text(json.dumps(manifest, indent=2) + '\n')
    write_sdf(args.out, manifest)
    print(json.dumps({k: manifest[k] for k in ('mass_kg', 'center_of_mass_m', 'inertia_kg_m2', 'bounds_m', 'camera_position_flu_m', 'motor_constant')}, indent=2))


if __name__ == '__main__':
    main()
