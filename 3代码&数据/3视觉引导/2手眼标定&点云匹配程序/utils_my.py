# -*- coding: utf-8 -*-
"""
201222
@author: xgy
一些工具函数
"""

import numpy as np
from numpy.linalg import inv
import open3d as o3d
import pandas as pd
from math import sqrt, atan2
import copy

# 对n*3的矩阵执行刚性变换
def executeTransform(mtxnx3, mtx4x4):
    assert(mtxnx3.shape[1] == 3)
    
    mtxnx4 = np.column_stack([mtxnx3, np.ones((mtxnx3.shape[0], 1))])
    mtxnx4_trans = np.dot(mtx4x4, mtxnx4.T).T
    mtxnx3_trans = mtxnx4_trans[:, 0:3]
    return mtxnx3_trans

# 两组3D对应点之间的变换估计
# 参考: https://github.com/scikit-image/scikit-image/blob/master/skimage/transform/_geometric.py
def umeyama(src, dst, estimate_scale=False):
    num = src.shape[0]
    dim = src.shape[1]

    # Compute mean of src and dst.
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)

    # Subtract mean from src and dst.
    src_demean = src - src_mean
    dst_demean = dst - dst_mean

    # Eq. (38).
    A = dst_demean.T @ src_demean / num

    # Eq. (39).
    d = np.ones((dim,), dtype=np.double)
    if np.linalg.det(A) < 0:
        d[dim - 1] = -1

    T = np.eye(dim + 1, dtype=np.double)

    U, S, V = np.linalg.svd(A)

    # Eq. (40) and (43).
    rank = np.linalg.matrix_rank(A)
    if rank == 0:
        return np.nan * T
    elif rank == dim - 1:
        if np.linalg.det(U) * np.linalg.det(V) > 0:
            T[:dim, :dim] = U @ V
        else:
            s = d[dim - 1]
            d[dim - 1] = -1
            T[:dim, :dim] = U @ np.diag(d) @ V
            d[dim - 1] = s
    else:
        T[:dim, :dim] = U @ np.diag(d) @ V

    if estimate_scale:
        # Eq. (41) and (42).
        scale = 1.0 / src_demean.var(axis=0).sum() * (S @ d)
    else:
        scale = 1.0

    T[:dim, dim] = dst_mean - scale * (T[:dim, :dim] @ src_mean.T)
    T[:dim, :dim] *= scale

    return T

####################################################################################################
COLOR = {
    "black": (0, 0, 0), # 黑色
    "white": (1, 1, 1), # 黑色
    "red": (1, 0, 0), # 红色
    "green": (0, 1, 0), # 绿色
    "blue": (0, 0, 1), # 蓝色
    "orange": (1, 0.706, 0), # 橙色
    "cyan": (0, 0.651, 0.929), # 青色
    "purple": (0.651, 0, 0.929), # 紫色
}

# numpy的ndarray类型 -> open3d的PointCloud类型
def ndarrayToPointCloud(points):
    dim = points.shape[1]
    assert(dim == 3 or dim == 6)
    
    pcd = o3d.geometry.PointCloud()
    
    if dim == 3:
        pcd.points = o3d.utility.Vector3dVector(points[:,0:3])
    elif dim == 6:
        pcd.points = o3d.utility.Vector3dVector(points[:,0:3])
        pcd.normals = o3d.utility.Vector3dVector(points[:,3:6])
    return pcd

# open3d的PointCloud类型 -> numpy的ndarray类型
def pointCloudToNdarray(pcd):
    points = np.array(pcd.points)
    normals = np.array(pcd.normals)
    
    if normals.shape[0] != 0:
        points = np.hstack([points, normals]) # 横向堆叠
    return points

# 依据参考点, 改变点云中法线的方向
def changeNormalDirection(pcd, ref_point):
    points = pointCloudToNdarray(pcd)
    ref_point = np.array(ref_point)
    assert(points.shape[1] == 6)
    assert(len(ref_point) == 3)
    
    points, normals = points[:,0:3], points[:,3:6]
    
    vec1 = normals
    vec2 = points - ref_point
    vec2 = vec2 / np.linalg.norm(vec2, axis=1, keepdims=True)
    theta_cos = np.sum(np.multiply(vec1, vec2), axis=1, keepdims=True) / (np.linalg.norm(vec1, axis=1, keepdims=True) * np.linalg.norm(vec2, axis=1, keepdims=True))
    theta_cos_bool = theta_cos >= 0
    theta_cos_int = theta_cos_bool.astype(np.int) * 2 - 1 # 正值为1, 负值为-1
    normals = theta_cos_int * normals
    
    pcd = ndarrayToPointCloud(np.hstack([points, normals]))
    return pcd

def drawRegistrationResult(source, target, transformation=np.eye(4), window_name="Open3D"):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color(COLOR["orange"])
    target_temp.paint_uniform_color(COLOR["cyan"])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp], window_name)

def drawRegistrationResultWithBaseFrame(mesh_frame, source, target, transformation=np.eye(4), window_name="Open3D"):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color(COLOR["red"])
    target_temp.paint_uniform_color(COLOR["black"])
    target_temp.transform(inv(transformation))
    o3d.visualization.draw_geometries([mesh_frame, source_temp, target_temp], window_name)

# 按x/y/z轴过滤点云
def filterByAxis(pcd, axis='z', threshold=0, if_reserve_greater=True):
    assert(axis == 'x' or axis == 'y' or axis == 'z')
    points = pointCloudToNdarray(pcd)
    dim = points.shape[1]
    assert(dim == 3 or dim == 6)
    if dim == 3:
        df = pd.DataFrame(points, columns=['x','y','z'])
    elif dim == 6:
        df = pd.DataFrame(points, columns=['x','y','z','nx','ny','nz'])
        
    if if_reserve_greater:
        df_filter = df[df[axis] >= threshold]
    else:
        df_filter = df[df[axis] <= threshold]
    print("filterBy{}:: {}=>{}".format(axis.upper(), df.shape[0], df_filter.shape[0]))
    
    pcd = ndarrayToPointCloud(df_filter.values)
    return pcd

####################################################################################################
# 检查矩阵是否为有效的旋转矩阵
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    if(n >= 1e-6):
        print("n>=1e-6, n={:.6f}".format(n))
    return n < 1e-3 # 1e-6

# 旋转矩阵 -> 欧拉角(弧度)
def rotationMatrixToEulerAngles(R):
    assert(isRotationMatrix(R))
     
    sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
 
    if not singular:
        x = atan2(R[2,1] , R[2,2])
        y = atan2(-R[2,0], sy)
        z = atan2(R[1,0], R[0,0])
    else:
        x = atan2(-R[1,2], R[1,1])
        y = atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z]) # roll, pitch, yaw # 滚转角、俯仰角、偏航角

# 变换矩阵(4*4) -> 欧拉角角度(3*1), 平移向量(3*1)
def separateTransformMatrixToEulerDegT(mtx4x4):
    euler_rad = rotationMatrixToEulerAngles(mtx4x4[0:3,0:3])
    euler_deg = np.rad2deg(euler_rad)
    T = mtx4x4[0:3,3]
    return euler_deg.flatten(), T.flatten()

