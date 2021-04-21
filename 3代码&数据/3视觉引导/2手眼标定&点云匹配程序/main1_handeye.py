# -*- coding: utf-8 -*-
"""
201222
@author: xgy
使用强行映射的方式做手眼标定
"""

import numpy as np
from numpy.linalg import inv, norm
np.set_printoptions(suppress=True) # 用numpy输出时不用科学计数法

from utils_my import umeyama, executeTransform

if __name__ == "__main__":
    # 4个顶点在点云相机坐标系下的坐标
    pt3d_camera = np.array([
        [0.075428, 0.197790, 0.956000],
        [0.070142, 0.081053, 0.889000],
        [-0.153098, 0.089047, 0.891000],
        [-0.142923, 0.210181, 0.959000],        
    ]) * 1000
    
    # 4个顶点在机械臂基坐标系下的坐标
    pt3d_robot = np.array([
        [1670.39, -213.59, 833.06],
        [1817.12, -213.59, 833.06],
        [1814.45, -1.25, 833.06],
        [1667.49, -1.25, 833.06],
    ])
    
    # 计算两组对应点之间的刚性变换
    cam2base_mtx4x4 = umeyama(pt3d_camera, pt3d_robot)
    base2cam_mtx4x4 = umeyama(pt3d_robot, pt3d_camera)
    
    print("点云相机坐标系到机械臂基坐标系的变换关系为:")
    print(cam2base_mtx4x4)
    
#    pt3d_robot_2 = executeTransform(pt3d_camera, mtx4x4=cam2base_mtx4x4)
#    print(pt3d_robot)
#    print(pt3d_robot_2)
#    print(pt3d_robot - pt3d_robot_2)
#    print(norm(pt3d_robot - pt3d_robot_2, axis=0))
