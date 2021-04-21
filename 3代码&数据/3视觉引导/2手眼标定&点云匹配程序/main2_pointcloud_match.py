# -*- coding: utf-8 -*-
"""
201222
@author: xgy
做点云匹配, 获取返回给机械臂的偏移量
"""

import open3d as o3d
import numpy as np
from numpy.linalg import inv
import os.path as osp
np.set_printoptions(suppress=True) # 用numpy输出时不用科学计数法

from utils_my import changeNormalDirection, filterByAxis
from utils_my import separateTransformMatrixToEulerDegT
from utils_my import COLOR, drawRegistrationResult, drawRegistrationResultWithBaseFrame

# 计算点云的法线和fpfh特征
def calcNormalAndFpfh(pcd, radius_normal, radius_feature):
    # 点云计算法线, 并纠正法线方向
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    
    centroid_point = pcd.get_center()
    centroid_point[2] -= 1000 # 改变点云质心的z, 使得法线朝向相机
    pcd = changeNormalDirection(pcd, centroid_point)
    
    # 点云计算fpfh特征
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
            pcd, 
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    
    return pcd, pcd_fpfh

# 打印配准的评价结果
printEvaluation = lambda evaluation: print("RegistrationResult fitness={:.2f} inlier_rmse={:.2f} correspondence_set={}".format(evaluation.fitness, evaluation.inlier_rmse, len(evaluation.correspondence_set)))

####################################################################################################
    
# 手眼标定的结果
cam2base_mtx4x4 = np.array([[  -0.03974753,   -0.86911948,   -0.4930025 , 2321.88173406],
                            [  -0.99912644,    0.04094116,    0.00837735, -158.65519043],
                            [   0.01290318,    0.49290481,   -0.86998756, 1565.96284086],
                            [   0.        ,    0.        ,    0.        ,    1.        ]])

if __name__ == "__main__":
    # 1. 读取点云, 裁剪
    # 从点云相机坐标系, 转到机械臂基坐标系, 再转到以点云质心为原点的坐标系
    fn_ply_model = "model.ply"
    fn_ply_guide = "guide_{}.ply".format(1) # 1~12
    assert(osp.exists(fn_ply_model)) # 验证数据文件是否存在
    assert(osp.exists(fn_ply_guide))
    print("读取的模板点云: {}".format(fn_ply_model))
    print("读取的引导点云: {}".format(fn_ply_guide))
    
    pcd_model = o3d.io.read_point_cloud(fn_ply_model)
#    pcd_model = pcd_model.scale(1000, center=np.array([0,0,0])) # m -> mm
    pcd_model.transform(cam2base_mtx4x4)
    # transToCentroid_mtx4x4的平移向量计算代码: np.around(pcd_model.get_center(), 2)
    
    pcd_guide = o3d.io.read_point_cloud(fn_ply_guide)
#    pcd_guide = pcd_guide.scale(1000, center=np.array([0,0,0])) # m -> mm
    pcd_guide.transform(cam2base_mtx4x4)
    
    # 2. 使用直通滤波器滤波, 从点云中裁剪出鞋垫
    pcd_model = filterByAxis(pcd_model, axis='z', threshold=900, if_reserve_greater=True)
    pcd_model = filterByAxis(pcd_model, axis='z', threshold=1500, if_reserve_greater=False)
    
    pcd_guide = filterByAxis(pcd_guide, axis='z', threshold=880, if_reserve_greater=True)
    pcd_guide = filterByAxis(pcd_guide, axis='z', threshold=1500, if_reserve_greater=False)
    
    # 3. 把点云转到以点云质心为原点的坐标系
    transToCentroid_mtx4x4 = np.eye(4)
    transToCentroid_mtx4x4[0:3, 3] = np.array([1756.52, -180.01,  933.09]) # 鞋垫
    transToCentroid_mtx4x4 = inv(transToCentroid_mtx4x4)
    
    pcd_model.transform(transToCentroid_mtx4x4)
    pcd_guide.transform(transToCentroid_mtx4x4)
    
    pcd_model.paint_uniform_color(COLOR['red'])
    pcd_guide.paint_uniform_color(COLOR['black'])
    
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 0]) # 坐标系, 便于可视化
    
#    o3d.visualization.draw_geometries([pcd_model, pcd_guide, mesh_frame])
    
#    assert(False)
    ####################################################################################################
#    drawRegistrationResult(pcd_model, pcd_guide, window_name="Origin")
#    drawRegistrationResultWithBaseFrame(mesh_frame, pcd_model, pcd_guide, np.eye(4), window_name="Origin")
    
    # 4. 计算点云的法线和fpfh特征
    voxel_size = 15 # TODO 点间距 1.5mm
    radius_normal = voxel_size * 2 # 估算点云法线的半径
    radius_feature = voxel_size * 5 # 计算fpfh特征的半径
    evaluate_distance = 3 # 评价配准结果的距离阈值
    
    pcd_src = pcd_model
    pcd_dst = pcd_guide
    distance_threshold = voxel_size * 1.5
    trans_init = np.eye(4)
    
    pcd_src, pcd_src_fpfh = calcNormalAndFpfh(pcd_src, radius_normal, radius_feature)
    pcd_dst, pcd_dst_fpfh = calcNormalAndFpfh(pcd_dst, radius_normal, radius_feature)
    
    # 5. 点云配准之前的评价
    print("\n::Before alignment")
    evaluation = o3d.registration.evaluate_registration(pcd_src, pcd_dst, evaluate_distance, trans_init)
    printEvaluation(evaluation)
    
    # 6. 粗配准: 使用 RANSAC+FPFH
    print("\n::Apply RANSAC+FPFH")
    reg_ransac = o3d.registration.registration_ransac_based_on_feature_matching(
            pcd_src, pcd_dst, pcd_src_fpfh, pcd_dst_fpfh, distance_threshold,
            o3d.registration.TransformationEstimationPointToPoint(False), 4, [
                o3d.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold),
                o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.registration.CorrespondenceCheckerBasedOnNormal(0.5),
                
            ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
    print([np.around(x, 2) for x in separateTransformMatrixToEulerDegT(reg_ransac.transformation)])
#    drawRegistrationResult(pcd_src, pcd_dst, reg_ransac.transformation, window_name="RANSAC+FPFH")
#    drawRegistrationResultWithBaseFrame(mesh_frame, pcd_src, pcd_dst, reg_ransac.transformation, window_name="RANSAC+FPFH")
#    ransac_trans_mtx4x4 = reg_ransac.transformation
#    print("RANSAC+FPFH 返回offset:", [np.around(x, 2) for x in separateTransformMatrixToEulerDegT(inv(transToCentroid_mtx4x4) @ ransac_trans_mtx4x4 @ transToCentroid_mtx4x4)])
    
    # 7. 粗配准之后的评价
    print("\n::After alignment")
    evaluation = o3d.registration.evaluate_registration(pcd_src, pcd_dst, evaluate_distance, reg_ransac.transformation)
    printEvaluation(evaluation)
    
    # 8. 精配准: 使用 point-to-point ICP
    print("\n::Apply point-to-point ICP")
    reg_p2p = o3d.registration.registration_icp(
            pcd_src, pcd_dst, distance_threshold, reg_ransac.transformation,
            o3d.registration.TransformationEstimationPointToPoint())
    print([np.around(x, 2) for x in separateTransformMatrixToEulerDegT(reg_p2p.transformation)])
#    drawRegistrationResult(pcd_src, pcd_dst, reg_p2p.transformation, window_name="point-to-point ICP")
#    drawRegistrationResultWithBaseFrame(mesh_frame, pcd_src, pcd_dst, reg_p2p.transformation, window_name="point-to-point ICP")
    icp_trans_mtx4x4 = reg_p2p.transformation
    print("point-to-point 返回offset:", [np.around(x, 2) for x in separateTransformMatrixToEulerDegT(inv(transToCentroid_mtx4x4) @ icp_trans_mtx4x4 @ transToCentroid_mtx4x4)])
    
    # 9. 精配准之后的评价
    print("\n::After alignment")
    evaluation = o3d.registration.evaluate_registration(pcd_src, pcd_dst, evaluate_distance, icp_trans_mtx4x4)
    printEvaluation(evaluation)
