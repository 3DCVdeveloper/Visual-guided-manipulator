# -*- coding: utf-8 -*-
"""
201215
@author: xgy
棋盘格相机标定双目, 剔除重投影误差大的点, 只保留70%的点
"""

import numpy as np
import cv2
import os.path as osp

np.set_printoptions(suppress=True) # 输出时不用科学计数法

####################################################################################################
folder = "201215_Paper_Color_IR" # 数据目录
IMG_LEN = 20 # 图片数
PATTERN_SHAPE = (6, 9) # 棋盘格角点数
PATTERN_UNIT = 25.4 # 每个棋盘格的大小
sortScale = 0.7 # 保留70%的角点

obj_pts3d = np.zeros((PATTERN_SHAPE[0] * PATTERN_SHAPE[1], 3), np.float32)
obj_pts3d[:, :2] = np.mgrid[0:PATTERN_SHAPE[0], 0:PATTERN_SHAPE[1]].T.reshape(-1, 2) * PATTERN_UNIT

# 从图像中提取角点
def getCornersFromImg(img, pattern_shape):
    if len(img.shape) == 3:
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    elif len(img.shape) == 2:
        img_gray = img
    ret, corners = cv2.findChessboardCorners(img_gray, pattern_shape, None)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    if ret == True:
        corners2 = cv2.cornerSubPix(img_gray, corners, (11, 11), (-1, -1), criteria)
        return corners2
    return corners

# 剔除重投影误差大的点重新做相机标定
def refineCalibrate(objPts_mxnx3, imgPts_mxnx2, imgSize, rvecs, tvecs, cameraMatrix, distCoeffs, calDist=True, sort=0.8,
                    filterRatio=0.2):
    ObjNum = objPts_mxnx3.shape[0]
    ProjectErrorList = []
    for i in range(ObjNum):
        ProjectPts_nx1x2, Jacobian = cv2.projectPoints(objectPoints=np.float32(objPts_mxnx3[i, :, :]).reshape(-1, 3),
                                                       rvec=rvecs[i], tvec=tvecs[i], cameraMatrix=cameraMatrix,
                                                       distCoeffs=distCoeffs)
        ProjectError_nx1x2 = ProjectPts_nx1x2 - imgPts_mxnx2[i].reshape(-1, 1, 2)
        ProjectError_nx1 = np.linalg.norm(x=ProjectError_nx1x2, axis=2)
        ProjectErrorList.append(ProjectError_nx1)

    ProjectError_mxnx1 = np.array(ProjectErrorList)
    ProjectError_mnx1 = ProjectError_mxnx1.reshape(-1, 1)
    ProjectErrorSortIdx_mnx1 = ProjectError_mnx1.argsort(axis=0)
    ProjectError_mnx1[ProjectErrorSortIdx_mnx1[0:int(ProjectErrorSortIdx_mnx1.size * sort)]] = True
    ProjectError_mnx1[ProjectErrorSortIdx_mnx1[int(ProjectErrorSortIdx_mnx1.size * sort):]] = False
    ProjectError_mnx1 = ProjectError_mnx1 > 0
    GoodPtsMask_mxnx1 = ProjectError_mnx1.reshape(ObjNum, -1, 1)
    GoodCirclePtsList = []
    NewObjPtsList = []
    GoodImgMask_list = []
    for i in range(ObjNum):
        if GoodPtsMask_mxnx1[i].mean() >= filterRatio:
            FilterCirclePts_nx2 = np.float32(imgPts_mxnx2[i][GoodPtsMask_mxnx1[i].reshape(-1)])
            GoodCirclePtsList.append(FilterCirclePts_nx2)
            FilterObjPts_nx2 = np.float32(objPts_mxnx3[i][GoodPtsMask_mxnx1[i].reshape(-1)])
            NewObjPtsList.append(FilterObjPts_nx2)
            GoodImgMask_list.append(True)
        else:
            GoodImgMask_list.append(False)
            print('Error: ', i)

    ReprojErr, CameraMatrix, DistCoeffs, rVecs, tVecs = cv2.calibrateCamera(
            np.array(NewObjPtsList), np.array(GoodCirclePtsList), imgSize, None, None, flags=0)
    return (
        ReprojErr, CameraMatrix, DistCoeffs, rVecs, tVecs, GoodPtsMask_mxnx1, GoodImgMask_list)
    
####################################################################################################
# 1. 读取标定板图片, 提取角点
obj_pts3d_list = []
img_rgb_pts2d_list = []
img_ir_pts2d_list = []
for i in range(1, IMG_LEN+1):
    fn_img_rgb = osp.join(folder, "Color_{}.bmp".format(i))
    fn_img_ir = osp.join(folder, "IR_{}.bmp".format(i))
    
    img_rgb = cv2.imread(fn_img_rgb)
    img_ir = cv2.imread(fn_img_ir)
    
    # TODO
#    img_rgb = cv2.flip(img_rgb, 1)  # 沿y轴镜像翻转
#    img_ir = cv2.flip(img_ir, 1)  # 沿y轴镜像翻转
    
    corners_rgb = getCornersFromImg(img_rgb, PATTERN_SHAPE)
    corners_ir = getCornersFromImg(img_ir, PATTERN_SHAPE)
    
    img_rgb_pts2d_list.append(corners_rgb)
    img_ir_pts2d_list.append(corners_ir)
    obj_pts3d_list.append(obj_pts3d)

####################################################################################################
"""
王工给的Astra Pro相机参数:

// 王工给的参数 L是IR图，R是彩色图
// [fx,fy,cx,cy]
cameraParams.l_intr_p[0] = 578.546021;
cameraParams.l_intr_p[1] = 578.546021;
cameraParams.l_intr_p[2] = 309.359985;
cameraParams.l_intr_p[3] = 232.233002;
// [fx, fy, cx, cy]
cameraParams.r_intr_p[0] = 593.426025;
cameraParams.r_intr_p[1] = 593.426025;
cameraParams.r_intr_p[2] = 315.527008;
cameraParams.r_intr_p[3] = 235.903000;
// [r00,r01,r02;r10,r11,r12;r20,r21,r22]
cameraParams.r2l_r[0] = 0.999968;
cameraParams.r2l_r[1] = 0.004491;
cameraParams.r2l_r[2] = -0.006635;
cameraParams.r2l_r[3] = -0.004534;
cameraParams.r2l_r[4] = 0.999969;
cameraParams.r2l_r[5] = -0.006485;
cameraParams.r2l_r[6] = 0.006605;
cameraParams.r2l_r[7] = 0.006515;
cameraParams.r2l_r[8] = 0.999957;
// [t1,t2,t3]
cameraParams.r2l_t[0] = -24.720200;
cameraParams.r2l_t[1] = 0.074482;
cameraParams.r2l_t[2] = -0.168342;
// [k1,k2,p1,p2,k3]
cameraParams.l_k[0] = -0.076802;
cameraParams.l_k[1] = 0.155459;
cameraParams.l_k[2] = 0.000000;
cameraParams.l_k[3] = 0.000367;
cameraParams.l_k[4] = -0.001332;
// [k1,k2,p1,p2,k3]
cameraParams.r_k[0] = 0.104120;
cameraParams.r_k[1] = -0.108380;
cameraParams.r_k[2] = 0.000000;
cameraParams.r_k[3] = -0.001978;
cameraParams.r_k[4] = -0.003020;
"""

# 2. 执行单目相机标定, 双目相机标定
# 2.1 彩色图
print("...... 优化前的RGB相机标定 ......")
ObjPts, LImgPts, ImgSize = np.array(obj_pts3d_list), np.array(img_rgb_pts2d_list), (640, 480)
ReprojErr, CameraMatrix, DistCoeffs, rVecs, tVecs = cv2.calibrateCamera(ObjPts, LImgPts, ImgSize, None, None, flags=0)
print("ret:", ReprojErr)      # 重投影误差
print("mtx:\n", CameraMatrix) # 内参数矩阵
print("dist:\n", DistCoeffs)  # 畸变系数   distortion cofficients = (k_1,k_2,p_1,p_2,k_3)

print("\n...... 优化后的RGB相机标定 ......")
LOptReprojErr, LOptCameraMatrix, LOptDistCoeffs, LOptrVecs, LOpttVecs, LGoodPtsMask_mxnx1, LGoodImgMask_list = refineCalibrate(
    objPts_mxnx3=ObjPts, imgPts_mxnx2=LImgPts, imgSize=ImgSize, rvecs=rVecs, tvecs=tVecs,
    cameraMatrix=CameraMatrix, distCoeffs=DistCoeffs, calDist=True, sort=sortScale)
print("ret:", LOptReprojErr)
print("mtx:\n", LOptCameraMatrix)
print("dist:\n", LOptDistCoeffs)


# 2.2 红外图
print("\n...... 优化前的IR相机标定 ......")
ObjPts, RImgPts, ImgSize = np.array(obj_pts3d_list), np.array(img_ir_pts2d_list), (640, 480)
ReprojErr, CameraMatrix, DistCoeffs, rVecs, tVecs = cv2.calibrateCamera(ObjPts, RImgPts, ImgSize, None, None, flags=0)
print("ret:", ReprojErr)
print("mtx:\n", CameraMatrix)
print("dist:\n", DistCoeffs)

print("\n...... 优化后的IR相机标定 ......")
ROptReprojErr, ROptCameraMatrix, ROptDistCoeffs, ROptrVecs, ROpttVecs, RGoodPtsMask_mxnx1, RGoodImgMask_list = refineCalibrate(
    objPts_mxnx3=ObjPts, imgPts_mxnx2=RImgPts, imgSize=ImgSize, rvecs=rVecs, tvecs=tVecs,
    cameraMatrix=CameraMatrix, distCoeffs=DistCoeffs, calDist=True, sort=sortScale)
print("ret:", ROptReprojErr)
print("mtx:\n", ROptCameraMatrix)
print("dist:\n", ROptDistCoeffs)


# 3. 双目标定
print("\n...... 优化前的双目标定 ......")
ret_rgb, mtx_rgb, dist_rgb, rvecs, tvecs = cv2.calibrateCamera(
        obj_pts3d_list, img_rgb_pts2d_list, img_rgb.shape[0:2][::-1], None, None)

ret_ir, mtx_ir, dist_ir, rvecs, tvecs = cv2.calibrateCamera(
        obj_pts3d_list, img_ir_pts2d_list, img_ir.shape[0:2][::-1], None, None)

ret_stereo, mtx_rgb2, dist_rgb2, mtx_ir2, dist_ir2, R, t, E, F = cv2.stereoCalibrate(
        obj_pts3d_list, img_rgb_pts2d_list, img_ir_pts2d_list,
        mtx_rgb, dist_rgb, mtx_ir, dist_ir, img_rgb.shape[0:2][::-1])
print('重投影误差:\n ', ret_stereo)
print('RGB相机 内参:\n', mtx_rgb2)
print('IR相机 内参:\n', mtx_ir2)
print('RGB相机 畸变系数:\n', dist_rgb2)
print('IR相机 畸变系数:\n', dist_ir2)
print('RGB->IR R:\n', R)
print('RGB->IR t:\n', t)


print("\n...... 优化后的双目标定 ......")
STEREO_CALIBRATE_VALID_IMG_NUM = 4

ObjPts_mxnx3 = []
ImgPtsLeft_mxnx2 = []
ImgPtsRight_mxnx2 = []

for idx in range(IMG_LEN):
    GoodPtsMask = (LGoodPtsMask_mxnx1[idx] * RGoodPtsMask_mxnx1[idx]).reshape(-1)
    if LImgPts[idx][GoodPtsMask].shape[0] < STEREO_CALIBRATE_VALID_IMG_NUM and \
            RImgPts[idx][GoodPtsMask].shape[0] < STEREO_CALIBRATE_VALID_IMG_NUM:
        continue
    ObjPts_mxnx3.append(ObjPts[idx][GoodPtsMask])
    ImgPtsLeft_mxnx2.append(LImgPts[idx][GoodPtsMask])
    ImgPtsRight_mxnx2.append(RImgPts[idx][GoodPtsMask])

Retval, CameraMatrixL, DistCoeffsL, CameraMatrixR, DistCoeffsR, R, t, E, F = cv2.stereoCalibrate(
    ObjPts_mxnx3, ImgPtsLeft_mxnx2, ImgPtsRight_mxnx2,
    LOptCameraMatrix, LOptDistCoeffs, ROptCameraMatrix, ROptDistCoeffs, ImgSize)
    
print('重投影误差:\n ', Retval)
print('RGB相机 内参:\n', CameraMatrixL)
print('IR相机 内参:\n', CameraMatrixR)
print('RGB相机 畸变系数:\n', DistCoeffsL)
print('IR相机 畸变系数:\n', DistCoeffsR)
print('RGB->IR R:\n', R)
print('RGB->IR t:\n', t)
