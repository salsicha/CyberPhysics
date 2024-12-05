

#### CELL 1 ####


import numpy as np
import os

import open3d as o3d
from open3d.web_visualizer import draw

import copy

from pydrake.all import (
    AddMultibodyPlantSceneGraph, BaseField, Box, 
    DepthImageToPointCloud, DiagramBuilder, Fields, FindResourceOrThrow, Parser, PixelType, PointCloud, RigidTransform, RollPitchYaw, Simulator, SpatialInertia,
)

# MeshcatVisualizerCpp, MeshcatPointCloudVisualizerCpp, 

import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf

# from scenarios import (
#     AddShape, AddRgbdSensors, AddMultibodyTriad
# )

import open3d.core as o3c

# if o3d.__DEVICE_API__ == 'cuda':
#     import open3d.cuda.pybind.t.pipelines.registration as treg
# else:
#     import open3d.cpu.pybind.t.pipelines.registration as treg

# import open3d.cpu.pybind.t.pipelines.registration as treg
# import open3d.cpu.pybind.pipelines.registration as treg
import open3d.pipelines.registration as treg

import sys
import time

sys.path.append('./test_data')


#### CELL 2 ####


"""
Point Cloud Utilities. Converts between three forms of pointcloud objects:
  - Drake Pointcloud Object
  - Open3d Pointcloud Object
  - Nx3 numpy array
"""

def pcl_np2o3d(pcl_np):
  """
  Input: Nx3 np array of points.
  Ouput: Open3d np object. 
  """
  assert(pcl_np.shape[1] == 3) # sanity check
  pcl_o3d = o3d.geometry.PointCloud()
  pcl_o3d.points = o3d.utility.Vector3dVector(pcl_np)
  return pcl_o3d 

def pcl_o3d2np(pcl_o3d):
  """
  Input: Open3d np object.
  Output: Nx3 np array of points. 
  """
  return np.asarray(pcl_o3d.points)
  
def pcl_np2drake(pcl_np, color):
  """
  Input: Nx3 np array of points in mm, and and 3x1 np array of color using uin8 format.  color can alternatively be Nx3, and then it is used directly.
  Output: drake Pointcloud object. 
  """
  assert(pcl_np.shape[1] == 3) # sanity check. 
  pcl_drake = PointCloud(new_size = pcl_np.shape[0],
                         fields= Fields(BaseField.kXYZs | BaseField.kRGBs))
  xyzs = pcl_drake.mutable_xyzs()
  xyzs[:,:] = np.array(pcl_np).transpose()
  rgbs = pcl_drake.mutable_rgbs()
  color = np.asarray(color)
  if len(color.shape) == 1:
      rgbs[:,:] = np.tile(color, (pcl_np.shape[0], 1)).transpose()
  else:
      rgbs[:] = color.T
  return pcl_drake

def pcl_drake2np(pcl_drake):
  """
  Input: drake Pointcloud object.
  Output: Nx3 np array of points. 
  """
  xyzs = pcl_drake.mutable_xyzs().copy().transpose()
  return xyzs 

def rgb_drake2np(pcl_drake):
  rgbs = pcl_drake.mutable_rgbs().copy().transpose()
  return rgbs 



#### CELL 3 ####

from open3d.web_visualizer import draw

def draw_registration_result(source, target, transformation):
    source.transform(transformation)

    # draw([source, target],
    #       lookat=[1.6784, 2.0612, 1.4451],
    #       up=[-0.3402, -0.9189, -0.1996])
    o3d.visualization.draw_geometries([source])


#### CELL 4 ####
    
source = o3d.io.read_point_cloud("./test_data/ICP/cloud_bin_0.pcd")
target = o3d.io.read_point_cloud("./test_data/ICP/cloud_bin_1.pcd")

# print("source: ", source.points.__dir__())
# # For Colored-ICP `colors` attribute must be of the same dtype as `positions` and `normals` attribute.
# source.point["colors"] = source.point["colors"].to(
#     o3d.core.Dtype.Float32) / 255.0
# target.point["colors"] = target.point["colors"].to(
#     o3d.core.Dtype.Float32) / 255.0

# Initial guess transform between the two point-cloud.
# ICP algortihm requires a good initial allignment to converge efficiently.
trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                         [-0.139, 0.967, -0.215, 0.7],
                         [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])


# draw_registration_result(source, target, trans_init)


#### CELL 5 ####


# For Multi-Scale ICP (o3d.utility.DoubleVector):

# `max_correspondence_distances` is proportianal to the resolution or the `voxel_sizes`.
# In general it is recommended to use values between 1x - 3x of the corresponding `voxel_sizes`.
# We may have a higher value of the `max_correspondence_distances` for the first coarse
# scale, as it is not much expensive, and gives us more tolerance to initial allignment.
max_correspondence_distances = o3d.utility.DoubleVector([0.3, 0.14, 0.07])


#### CELL 6 ####


# Initial alignment or source to target transform.
init_source_to_target = np.asarray([[0.862, 0.011, -0.507, 0.5],
                                    [-0.139, 0.967, -0.215, 0.7],
                                    [0.487, 0.255, 0.835, -1.4],
                                    [0.0, 0.0, 0.0, 1.0]])


#### CELL 7 ####


# Select the `Estimation Method`, and `Robust Kernel` (for outlier-rejection).
# estimation = treg.TransformationEstimationPointToPlane()

estimation = treg.TransformationEstimationPointToPlane()


#### CELL 8 ####


# Estimation Method

#     This sets the ICP method to compute the transformation between two point-clouds given the correspondences.

# Options:

#     o3d.t.pipelines.registration.TransformationEstimationPointToPoint()
#         Point to Point ICP.
#     o3d.t.pipelines.registration.TransformationEstimationPointToPlane(robust_kernel)
#         Point to Plane ICP.
#         Requires target point-cloud to have normals attribute (of same dtype as position attribute).
#     o3d.t.pipelines.registration.TransformationEstimationForColoredICP(robust_kernel, lambda)
#         Colored ICP.
#         Requires target point-cloud to have normals attribute (of same dtype as position attribute).
#         Requires source and target point-clouds to have colors attribute (of same dtype as position attribute).
#     o3d.t.pipelines.registration.TransformationEstimationForGeneralizedICP(robust_kernel, epsilon) [To be added].
#         Generalized ICP.



#### CELL 9 ####


# Estimation Method also supports Robust Kernels: Robust kernels are used for outlier rejection. More on this in Robust Kernel section.

# robust_kernel = o3d.t.pipelines.registration.robust_kernel.RobustKernel(method, scale, shape)

# Method options:

#     robust_kernel.RobustKernelMethod.L2Loss
#     robust_kernel.RobustKernelMethod.L1Loss
#     robust_kernel.RobustKernelMethod.HuberLoss
#     robust_kernel.RobustKernelMethod.CauchyLoss
#     robust_kernel.RobustKernelMethod.GMLoss
#     robust_kernel.RobustKernelMethod.TukeyLoss
#     robust_kernel.RobustKernelMethod.GeneralizedLoss




#### CELL 10 ####


# ICP Convergence Criteria [relative rmse, relative fitness, max iterations]

#     This sets the condition for termination or when the scale iterations can be considered to be converged.
#     If the relative (of change in value from the last iteration) rmse and fitness are equal or less than the specified value, the iterations for that scale will be considered as converged/completed.
#     For Multi-Scale ICP it is a list of ICPConvergenceCriteria, for each scale of ICP, to provide more fine control over performance.
#     One may keep the initial values of relative_fitness and relative_rmse low as we just want to get an estimate transformation, and high for later iterations to fine-tune.
#     Iterations on higher-resolution is more costly (takes more time), so we want to do fewer iterations on higher resolution.




#### CELL 10 ####


# Convergence-Criteria for Vanilla ICP:

criteria = treg.ICPConvergenceCriteria(relative_fitness=0.000001,
                                       relative_rmse=0.000001,
                                       max_iteration=50)


#### CELL 11 ####


# List of Convergence-Criteria for Multi-Scale ICP:

# We can control `ConvergenceCriteria` of each `scale` individually.
# We want to keep `relative_fitness` and `relative_rmse` high (more error tolerance)
# for initial scales, i.e. we will be happy to consider ICP converged, when difference
# between 2 successive iterations for that scale is smaller than this value.
# We expect less accuracy (more error tolerance) initial coarse-scale iteration,
# and want our later scale convergence to be more accurate (less error tolerance).
criteria_list = [
    treg.ICPConvergenceCriteria(relative_fitness=0.0001,
                                relative_rmse=0.0001,
                                max_iteration=20),
    treg.ICPConvergenceCriteria(0.00001, 0.00001, 15),
    treg.ICPConvergenceCriteria(0.000001, 0.000001, 10)
]

#### CELL 12 ####


# Vanilla ICP
voxel_size = 0.025


#### CELL 13 ####


# Lower `voxel_size` is equivalent to higher resolution,
# and we want to perform iterations from coarse to dense resolution,
# therefore `voxel_sizes` must be in strictly decressing order.
voxel_sizes = o3d.utility.DoubleVector([0.1, 0.05, 0.025])



#### CELL 14 ####


# Save Loss Log

# When True, it saves the iteration-wise values of fitness, inlier_rmse, transformaton, scale, iteration in loss_log_ in regsitration_result. Default: False.


#### CELL 15 ####


save_loss_log = True


#### CELL 16 ####


# Vanilla ICP

# Input point-clouds
# source = o3d.t.io.read_point_cloud("./test_data/ICP/cloud_bin_0.pcd")
# target = o3d.t.io.read_point_cloud("./test_data/ICP/cloud_bin_1.pcd")

source = o3d.io.read_point_cloud("./test_data/ICP/cloud_bin_0.pcd")
target = o3d.io.read_point_cloud("./test_data/ICP/cloud_bin_1.pcd")
source_t = o3d.t.io.read_point_cloud("./test_data/ICP/cloud_bin_0.pcd")
target_t = o3d.t.io.read_point_cloud("./test_data/ICP/cloud_bin_1.pcd")


#### CELL 17 ####


# Search distance for Nearest Neighbour Search [Hybrid-Search is used].
max_correspondence_distance = 0.07

# Initial alignment or source to target transform.
init_source_to_target = np.asarray([[0.862, 0.011, -0.507, 0.5],
                                    [-0.139, 0.967, -0.215, 0.7],
                                    [0.487, 0.255, 0.835, -1.4],
                                    [0.0, 0.0, 0.0, 1.0]])

init_source_to_target_c = o3d.core.Tensor(init_source_to_target)

# Select the `Estimation Method`, and `Robust Kernel` (for outlier-rejection).
estimation = treg.TransformationEstimationPointToPlane()

# Convergence-Criteria for Vanilla ICP
criteria = treg.ICPConvergenceCriteria(relative_fitness=0.000001,
                                       relative_rmse=0.000001,
                                       max_iteration=50)
# Down-sampling voxel-size.
voxel_size = 0.025

# Save iteration wise `fitness`, `inlier_rmse`, etc. to analyse and tune result.
save_loss_log = True



#### CELL 18 ####


s = time.time()

# registration_icp = treg.icp(source_t, target_t, max_correspondence_distance,
#                             init_source_to_target, estimation, criteria,
#                             voxel_size)

registration_icp = treg.registration_icp(source, target, max_correspondence_distance)


icp_time = time.time() - s
print("Time taken by ICP: ", icp_time)
print("Inlier Fitness: ", registration_icp.fitness)
print("Inlier RMSE: ", registration_icp.inlier_rmse)

# PRINT THIS
# draw_registration_result(source, target, registration_icp.transformation)


#### CELL 19 ####


# Now let's try with poor initial initialisation
init_source_to_target = o3d.core.Tensor.eye(4, o3d.core.Dtype.Float32)
max_correspondence_distance = 0.07


#### CELL 20 ####


s = time.time()

# registration_icp = treg.icp(source, target, max_correspondence_distance,
#                             init_source_to_target, estimation, criteria,
#                             voxel_size, save_loss_log)

registration_icp = treg.registration_icp(source, target, max_correspondence_distance)


icp_time = time.time() - s
# print("Time taken by ICP: ", icp_time)
# print("Inlier Fitness: ", registration_icp.fitness)
# print("Inlier RMSE: ", registration_icp.inlier_rmse)


# PRINT THIS
# draw_registration_result(source, target, registration_icp.transformation)



#### CELL 21 ####


init_source_to_target = o3d.core.Tensor.eye(4, o3c.float32)
max_correspondence_distance = 0.5


#### CELL 22 ####


s = time.time()
# It is highly recommended to down-sample the point-cloud before using
# ICP algorithm, for better performance.

# registration_icp = treg.icp(source, target, max_correspondence_distance,
#                             init_source_to_target, estimation, criteria,
#                             voxel_size, save_loss_log)

registration_icp = treg.registration_icp(source, target, max_correspondence_distance)


icp_time = time.time() - s
# print("Time taken by ICP: ", icp_time)
# print("Inlier Fitness: ", registration_icp.fitness)
# print("Inlier RMSE: ", registration_icp.inlier_rmse)

# draw_registration_result(source, target, registration_icp.transformation)


#### CELL 23 ####


# Multi-scal iCP

voxel_sizes = o3d.utility.DoubleVector([0.1, 0.05, 0.025])

if o3d.__DEVICE_API__ == 'cuda':
    import open3d.cuda.pybind.t.pipelines.registration as treg_t
else:
    import open3d.cpu.pybind.t.pipelines.registration as treg_t

# List of Convergence-Criteria for Multi-Scale ICP:
criteria_list = [
    treg_t.ICPConvergenceCriteria(relative_fitness=0.0001,
                                relative_rmse=0.0001,
                                max_iteration=20),
    treg_t.ICPConvergenceCriteria(0.00001, 0.00001, 15),
    treg_t.ICPConvergenceCriteria(0.000001, 0.000001, 10)
]

# `max_correspondence_distances` for Multi-Scale ICP (o3d.utility.DoubleVector):
max_correspondence_distances = o3d.utility.DoubleVector([0.3, 0.14, 0.07])

# Initial alignment or source to target transform.
init_source_to_target = o3d.core.Tensor.eye(4, o3d.core.Dtype.Float32)

# Select the `Estimation Method`, and `Robust Kernel` (for outlier-rejection).
estimation = treg.TransformationEstimationPointToPlane()

# Save iteration wise `fitness`, `inlier_rmse`, etc. to analyse and tune result.
save_loss_log = True



#### CELL 24 ####


# Setting Verbosity to Debug, helps in fine-tuning the performance.
# o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

s = time.time()

registration_ms_icp = treg_t.multi_scale_icp(source_t, target_t, voxel_sizes,
                                           criteria_list,
                                           max_correspondence_distances)
                                        #    init_source_to_target, estimation,
                                        #    save_loss_log)


ms_icp_time = time.time() - s
print("Time taken by Multi-Scale ICP: ", ms_icp_time)
print("Inlier Fitness: ", registration_ms_icp.fitness)
print("Inlier RMSE: ", registration_ms_icp.inlier_rmse)


np_trans = registration_ms_icp.transformation
print("trans: ", np_trans)

draw_registration_result(source, target, np_trans)


#### CELL 25 ####


from matplotlib import pyplot as plt


def plot_rmse(registration_result):
    fig, axes = plt.subplots(nrows=1, ncols=1, figsize=(20, 5))
    axes.set_title("Inlier RMSE vs Iteration")

    # print("registration_result: ", dir(registration_result))
    # print("rmse: ", registration_result.inlier_rmse)

    # FIX PLOT

    # axes.plot(registration_result.loss_log["index"].numpy(),
    #           registration_result.loss_log["inlier_rmse"].numpy())


def plot_scale_wise_rmse(registration_result):

    # print("transformation: ", dir(registration_result.transformation))
    # print("fitness: ", dir(registration_result.fitness))
    # print("inlier_rmse: ", dir(registration_result.inlier_rmse))
    # print("correspondence_set: ", dir(registration_result.correspondence_set))

    print(dir(registration_result))

    # print(registration_result.iteration_index)
    # print(registration_result.scale_index)

    # TODO: fix plotting, just make list???



    # scales = registration_result.loss_log["scale"].numpy()
    # iterations = registration_result.loss_log["iteration"].numpy()

    # # scales = registration_result.

    # num_scales = scales[-1][0] + 1

    # fig, axes = plt.subplots(nrows=1, ncols=num_scales, figsize=(20, 5))

    # masks = {}
    # for scale in range(0, num_scales):
    #     masks[scale] = registration_result.loss_log["scale"] == scale

    #     rmse = registration_result.loss_log["inlier_rmse"][masks[scale]].numpy()
    #     iteration = registration_result.loss_log["iteration"][
    #         masks[scale]].numpy()

    #     title_prefix = "Scale Index: " + str(scale)
    #     axes[scale].set_title(title_prefix + " Inlier RMSE vs Iteration")
    #     axes[scale].plot(iteration, rmse)




#### CELL 26 ####


print("Vanilla ICP")
plot_rmse(registration_icp)


#### CELL 27 ####


print("Multi Scale ICP")
plot_rmse(registration_ms_icp)
plot_scale_wise_rmse(registration_ms_icp)


#### CELL 28 ####


# TODO: fix plots

# fig, axes = plt.subplots(nrows=1, ncols=1, figsize=(20, 5))

# axes.set_title("Vanilla ICP and Multi-Scale ICP `Inlier RMSE` vs `Iteration`")

# if len(registration_ms_icp.loss_log["index"]) > len(
#         registration_icp.loss_log["inlier_rmse"]):
#     axes.plot(registration_ms_icp.loss_log["index"].numpy(),
#               registration_ms_icp.loss_log["inlier_rmse"].numpy(),
#               registration_icp.loss_log["inlier_rmse"].numpy())
# else:
#     axes.plot(registration_icp.loss_log["index"].numpy(),
#               registration_icp.loss_log["inlier_rmse"].numpy(),
#               registration_ms_icp.loss_log["inlier_rmse"].numpy())


#### CELL 29 ####


# TODO: fix source


# The algorithm runs on the same device as the source and target point-cloud.
# source_cuda = source.cuda(0)
# target_cuda = target.cuda(0)

device = o3d.core.Device("CUDA:0")
dtype = o3d.core.float32

source_pcd = o3d.t.geometry.PointCloud(device)
# source_pcd.point.positions = np.asarray(source.points)
source_pcd.point.positions = o3d.core.Tensor(np.asarray(source.points), dtype, device)

target_pcd = o3d.t.geometry.PointCloud(device)
# target_pcd.point.positions = np.asarray(target.points)
target_pcd.point.positions = o3d.core.Tensor(np.asarray(target.points), dtype, device)

# dtype = o3d.core.float32
# pcd.point.positions
# pcl_o3d = o3d.geometry.PointCloud()
# pcl_o3d.points = o3d.utility.Vector3dVector(pcl_np)



#### CELL 30 ####


s = time.time()

registration_ms_icp = treg_t.multi_scale_icp(source_pcd, target_pcd,
                                           voxel_sizes, criteria_list,
                                           max_correspondence_distances)
                                        #    init_source_to_target, estimation,
                                        #    save_loss_log)

ms_icp_time = time.time() - s
print("Time taken by Multi-Scale ICP: ", ms_icp_time)
print("Inlier Fitness: ", registration_ms_icp.fitness)
print("Inlier RMSE: ", registration_ms_icp.inlier_rmse)

# TODO: fix drawing

# draw_registration_result(source.cpu(), target.cpu(),
#                          registration_ms_icp.transformation)


#### CELL 31 ####


information_matrix = treg_t.get_information_matrix(
    source_t, target_t, max_correspondence_distances[2],
    registration_ms_icp.transformation)

print(information_matrix)


#### CELL 32 ####


# Point-to-Plane ICP Registration

# The point-to-plane ICP algorithm [ChenAndMedioni1992] uses a different objective function

# where is the normal of point

# . [Rusinkiewicz2001] has shown that the point-to-plane ICP algorithm has a faster convergence speed than the point-to-point ICP algorithm.

# The class TransformationEstimationPointToPlane provides functions to compute the residuals and Jacobian matrices of the point-to-plane ICP objective. 


#### CELL 33 ####


estimation = treg.TransformationEstimationPointToPlane()

criteria = treg.ICPConvergenceCriteria(relative_fitness=0.0000001,
                                       relative_rmse=0.0000001,
                                       max_iteration=30)



#### CELL 34 ####



# TODO: this just stalls out


print("Apply Point-to-Plane ICP")
s = time.time()

reg_point_to_plane = treg_t.icp(source_t, target_t, max_correspondence_distance, voxel_size = voxel_size)   
                            #   init_source_to_target, estimation, criteria,
                            #   voxel_size, save_loss_log)

icp_time = time.time() - s
print("Time taken by Point-To-Plane ICP: ", icp_time)
print("Fitness: ", reg_point_to_plane.fitness)
print("Inlier RMSE: ", reg_point_to_plane.inlier_rmse)


# TODO: fix drawing


# draw_registration_result(source, target, reg_point_to_plane.transformation)


#### CELL 35 ####


# Case of no correspondences

max_correspondence_distance = 0.02

init_source_to_target = np.asarray([[1.0, 0.0, 0.0, 5], [0.0, 1.0, 0.0, 7],
                                    [0.0, 0.0, 1.0, 10], [0.0, 0.0, 0.0, 1.0]])

registration_icp = treg_t.icp(source_t, target_t, max_correspondence_distance,
                            init_source_to_target)

print("Inlier Fitness: ", registration_icp.fitness)
print("Inlier RMSE: ", registration_icp.inlier_rmse)
print("Transformation: \n", registration_icp.transformation)

if registration_icp.fitness == 0 and registration_icp.inlier_rmse == 0:
    print("ICP Convergence Failed, as no correspondence were found")


#### CELL 36 ####


# Information matrix

information_matrix = treg_t.get_information_matrix(
    source_t, target_t, max_correspondence_distances[2],
    registration_ms_icp.transformation)

print(information_matrix)



#### CELL 37 ####


# Initial alignment

source = o3d.io.read_point_cloud("../../test_data/ICP/cloud_bin_0.pcd")
target = o3d.io.read_point_cloud("../../test_data/ICP/cloud_bin_1.pcd")

# Initial guess transform between the two point-cloud.
# ICP algortihm requires a good initial allignment to converge efficiently.
trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                         [-0.139, 0.967, -0.215, 0.7],
                         [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])


# TODO: fix drawing

# draw_registration_result(source, target, trans_init)


#### CELL 38 ####


# Search distance for Nearest Neighbour Search [Hybrid-Search is used].
max_correspondence_distance = 0.02

print("Initial alignment")
evaluation = o3d.pipelines.registration.evaluate_registration(source, target, max_correspondence_distance)
                                        # , trans_init)

print("Fitness: ", evaluation.fitness)
print("Inlier RMSE: ", evaluation.inlier_rmse)


#### CELL END ####

sys.exit()
print("end")

