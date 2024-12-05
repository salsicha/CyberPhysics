import numpy as np
from stl import mesh
import open3d as o3d


mesh = o3d.io.read_triangle_mesh("car_dolly_closed.stl")

voxel_grid = o3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh, voxel_size=10.0)
o3d.io.write_voxel_grid("voxel_grid.ply", voxel_grid)


vertices, faces, normals, _ = skimage.measure.marching_cubes(voxel_grid, level=0)
mesh = trimesh.Trimesh(vertices=vertices, faces=faces, vertex_normals=normals)
mesh.export('export_o3d.stl')



