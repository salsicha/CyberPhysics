import numpy as np
import pyvista as pv
from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt
import matplotlib.animation as animation

import open3d as o3d
from open3d.web_visualizer import draw
import open3d.core as o3c

from IPython.display import display, SVG, HTML

if o3d.__DEVICE_API__ == 'cuda':
    import open3d.cuda.pybind.t.pipelines.registration as treg
else:
    import open3d.cpu.pybind.t.pipelines.registration as treg
    
import sys
import time

from pydrake.all import (
    AddMultibodyPlantSceneGraph, BaseField, CameraInfo, ClippingRange, CsdpSolver, DiagramBuilder, DepthImageToPointCloud, DepthRange, DepthRenderCamera, FindResourceOrThrow, ge, MakeRenderEngineVtk, MakePhongIllustrationProperties, MathematicalProgram, MeshcatPointCloudVisualizer, MeshcatVisualizerParams, Parser, RenderCameraCore, RenderEngineVtkParams, RgbdSensor, RigidTransform, RollPitchYaw, RotationMatrix)

# MeshcatVisualizerCpp


def MakeRandomObjectModelAndScenePoints(
    num_model_points=20, 
    noise_std=0, 
    num_outliers=0, 
    yaw_O=None,
    p_O=None, 
    num_viewable_points=None, 
    seed=None):
    """ Returns p_Om, p_s """
    random_state = np.random.RandomState(seed)

    # Make a random set of points to define our object in the x,y plane
    theta = np.arange(0, 2.0*np.pi, 2.0*np.pi/num_model_points)
    l = 1.0 + 0.5*np.sin(2.0*theta) + 0.4*random_state.rand(1, num_model_points)
    p_Om = np.vstack((l * np.sin(theta), l * np.cos(theta), 0 * l))

    # Make a random object pose if one is not specified, and apply it to get the scene points.
    if p_O is None:
        p_O = [2.0*random_state.rand(), 2.0*random_state.rand(), 0.0]
    if len(p_O) == 2:
        p_O.append(0.0)
    if yaw_O is None:
        yaw_O = 0.5*random_state.random()
    X_O = RigidTransform(RotationMatrix.MakeZRotation(yaw_O), p_O)
    if num_viewable_points is None:
        num_viewable_points = num_model_points
    assert num_viewable_points <= num_model_points
    p_s = X_O.multiply(p_Om[:,:num_viewable_points])
    p_s[:2, :]  += random_state.normal(scale=noise_std, size=(2, num_viewable_points))
    if num_outliers:
        outliers = random_state.uniform(low=-1.5, high=3.5, size=(3, num_outliers))
        outliers[2,:] = 0
        p_s = np.hstack((p_s, outliers))

    return p_Om, p_s, X_O

def MakeRectangleModelAndScenePoints(
    num_points_per_side=7,
    noise_std=0, 
    num_outliers=0, 
    yaw_O=None,
    p_O=None, 
    num_viewable_points=None, 
    seed=None):
    random_state = np.random.RandomState(seed)
    if p_O is None:
        p_O = [2.0*random_state.rand(), 2.0*random_state.rand(), 0.0]
    if len(p_O) == 2:
        p_O.append(0.0)
    if yaw_O is None:
        yaw_O = 0.5*random_state.random()
    X_O = RigidTransform(RotationMatrix.MakeZRotation(yaw_O), p_O)
    if num_viewable_points is None:
        num_viewable_points = 4*num_points_per_side
    
    x = np.arange(-1, 1, 2/num_points_per_side)
    half_width = 2
    half_height = 1
    top = np.vstack((half_width*x, half_height + 0*x))
    right = np.vstack((half_width + 0*x, -half_height*x))
    bottom = np.vstack((-half_width*x, -half_height + 0*x))
    left = np.vstack((-half_width + 0*x, half_height*x))
    p_Om = np.vstack((np.hstack((top, right, bottom, left)), np.zeros((1, 4*num_points_per_side))))
    p_s = X_O.multiply(p_Om[:,:num_viewable_points])
    p_s[:2, :]  += random_state.normal(scale=noise_std, size=(2, num_viewable_points))
    if num_outliers:
        outliers = random_state.uniform(low=-1.5, high=3.5, size=(3, num_outliers))
        outliers[2,:] = 0
        p_s = np.hstack((p_s, outliers))

    return p_Om, p_s, X_O


def PlotEstimate(p_Om, p_s, Xhat_O=RigidTransform(), chat=None, X_O=None, ax=None):
    p_m = Xhat_O.multiply(p_Om)
    if ax is None:
        ax = plt.subplot()
    Nm = p_Om.shape[1]
    artists = ax.plot(p_m[0, :], p_m[1, :], 'bo')
    artists += ax.fill(p_m[0, :], p_m[1, :], 'lightblue', alpha=0.5)
    artists += ax.plot(p_s[0, :], p_s[1, :], 'ro')
    if chat is not None:
        artists += ax.plot(np.vstack((p_m[0, chat], p_s[0, :])), np.vstack((p_m[1, chat], p_s[1, :])), 'g--')
    if X_O:
        p_s = X_O.multiply(p_Om)
    artists += ax.fill(p_s[0, :Nm], p_s[1, :Nm], 'lightsalmon')
    ax.axis('equal')
    return artists

def PrintResults(X_O, Xhat_O):
    p = X_O.translation()
    aa = X_O.rotation().ToAngleAxis()
    print(f"True position: {p}")
    print(f"True orientation: {aa}")
    p = Xhat_O.translation()
    aa = Xhat_O.rotation().ToAngleAxis()
    print(f"Estimated position: {p}")
    print(f"Estimated orientation: {aa}")

def PoseEstimationGivenCorrespondences(p_Om, p_s, chat):
    """ Returns optimal X_O given the correspondences """
    # Apply correspondences, and transpose data to support numpy broadcasting
    p_Omc = p_Om[:, chat].T
    p_s = p_s.T

    # Calculate the central points
    p_Ombar = p_Omc.mean(axis=0)
    p_sbar = p_s.mean(axis=0)

    # Calculate the "error" terms, and form the data matrix
    merr = p_Omc - p_Ombar
    serr = p_s - p_sbar
    W = np.matmul(serr.T, merr)

    # Compute R
    U, Sigma, Vt = np.linalg.svd(W)
    R = np.matmul(U, Vt)
    if np.linalg.det(R) < 0:
       print("fixing improper rotation")
       Vt[-1, :] *= -1
       R = np.matmul(U, Vt)

    # Compute p
    p = p_sbar - np.matmul(R, p_Ombar)

    return RigidTransform(RotationMatrix(R), p)

def FindClosestPoints(point_cloud_A, point_cloud_B):
    """
    Finds the nearest (Euclidean) neighbor in point_cloud_B for each
    point in point_cloud_A.
    @param point_cloud_A A 3xN numpy array of points.
    @param point_cloud_B A 3xN numpy array of points.
    @return indices An (N, ) numpy array of the indices in point_cloud_B of each
        point_cloud_A point's nearest neighbor.
    """
    indices = np.empty(point_cloud_A.shape[1], dtype=int)

    # TODO(russt): Replace this with a direct call to flann
    # https://pypi.org/project/flann/
    kdtree = o3d.geometry.KDTreeFlann(point_cloud_B)
    for i in range(point_cloud_A.shape[1]):
        nn = kdtree.search_knn_vector_3d(point_cloud_A[:,i], 1)
        indices[i] = nn[1][0]

    return indices

def IterativeClosestPoint(p_Om, p_s, X_O=None, animate=True):
    Xhat = RigidTransform()
    Nm = p_s.shape[1]
    chat_previous = np.zeros(Nm)

    fig, ax = plt.subplots()
    frames = []
    frames.append(PlotEstimate(p_Om=p_Om, p_s=p_s, Xhat_O=Xhat, chat=None, X_O=X_O, ax=ax))

    while True:
        chat = FindClosestPoints(p_s, Xhat.multiply(p_Om))
        if np.array_equal(chat, chat_previous):
            # Then I've converged.
            break
        chat_previous = chat
        frames.append(PlotEstimate(p_Om=p_Om, p_s=p_s, Xhat_O=Xhat, chat=chat, X_O=X_O, ax=ax))
        Xhat = PoseEstimationGivenCorrespondences(p_Om, p_s, chat)
        frames.append(PlotEstimate(p_Om=p_Om, p_s=p_s, Xhat_O=Xhat, chat=None, X_O=X_O, ax=ax))

    ani = animation.ArtistAnimation(fig, frames, interval=400, repeat=False)

    display(HTML(ani.to_jshtml()))
    plt.close()

    if X_O:
        PrintResults(X_O, Xhat)

    return Xhat, chat
    
