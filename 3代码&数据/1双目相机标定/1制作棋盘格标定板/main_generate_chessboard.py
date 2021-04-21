# -*- coding: utf-8 -*-
"""
201214
@author: xgy
生成棋盘格图片(适用于A4纸, 10*7)
参考: opencv生成棋盘格图像 https://blog.csdn.net/dawudayudaxue/article/details/106339491
"""

import cv2
import numpy as np
import sys

# 参数
SCALE = 0.48 # 0.4 便于按比例整体缩放图片
perBoardPixel = int(100 * SCALE * 2) # 每个黑白格的像素数
boardSize = (7, 10) # (height, width) 棋盘格的大小
resolution = (int(1400 * SCALE), int(2000 * SCALE)) # (height, width) 图片大小

if __name__ == "__main__":
    basisHeight = (resolution[0] - perBoardPixel * boardSize[0]) // 2
    basisWidth = (resolution[1] - perBoardPixel * boardSize[1]) // 2
    if basisHeight < 0 or basisWidth < 0:
        print("Resolution doesn't match!")
        sys.exit(0)
    
    image = np.ones(resolution).astype(np.uint8) * 255 # 全白的图片
    flag = 0
    for j in range(0, boardSize[0]):
        for i in range(0, boardSize[1]):
            flag = (i + j) % 2
            if flag == 0:
                for n in range(j * perBoardPixel, (j + 1) * perBoardPixel):
                    for m in range(i * perBoardPixel, (i + 1) * perBoardPixel):
                        image[n + basisWidth, m + basisHeight] = 0 # 赋黑色

#    cv2.imshow("chessBoard", image)
#    cv2.waitKey(0)
    cv2.imwrite("chessBoard.bmp", image)
