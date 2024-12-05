import numpy as np
import pyvista as pv
from stl import mesh

import importlib
import mpl_toolkits
# importlib.import_module('mpl_toolkits.mplot3d').__path__

from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt

import matplotlib.animation as animation

import open3d as o3d
from open3d.web_visualizer import draw
import open3d.core as o3c

import pyntcloud

from IPython.display import display, SVG, HTML

import k3d

if o3d.__DEVICE_API__ == 'cuda':
    import open3d.cuda.pybind.t.pipelines.registration as treg
else:
    import open3d.cpu.pybind.t.pipelines.registration as treg
    
import sys
import time
import copy

# import mit

from pydrake.all import (
    AddMultibodyPlantSceneGraph, BaseField, CameraInfo, ClippingRange, 
    CsdpSolver, DiagramBuilder, DepthImageToPointCloud, DepthRange, 
    DepthRenderCamera, FindResourceOrThrow, ge, MakeRenderEngineVtk, 
    MakePhongIllustrationProperties, MathematicalProgram, 
    Parser, RenderCameraCore, RenderEngineVtkParams, RgbdSensor, 
    RigidTransform, RollPitchYaw, RotationMatrix)

# MeshcatVisualizerCpp, MeshcatPointCloudVisualizer, MeshcatVisualizerParams,

from mesh_to_sdf import mesh_to_voxels, sample_sdf_near_surface, mesh_to_sdf
import skimage
import trimesh
import pyrender

import pandas as pd
from pyntcloud import PyntCloud

import icp


########


mesh = o3d.io.read_triangle_mesh("car_dolly.stl")

draw([mesh],
      lookat=[1.6784, 2.0612, 1.4451],
      up=[-0.3402, -0.9189, -0.1996])


########



########




########





########





########





########




########





########





########




########




########




########




########




########





########




########



########




