# -*- coding: utf-8 -*-
"""
201218
@author: xgy
3D重建的主函数, 使用奥比中光Astra Pro采集的D2C对齐后的彩色图和深度图
参考open3d: https://github.com/intel-isl/Open3D/tree/v0.10.0/examples/Python/ReconstructionSystem
说明: 3D重建实现的是CVPR2015的《Robust Reconstruction of Indoor Scenes》, 点云配准使用的是ICCV2017的《Colored Point Cloud Registration Revisited》
"""

import json
import time
import datetime

import sys
sys.path.append("./utility")
from file import check_folder_structure
from initialize_config import initialize_config
import make_fragments
import register_fragments
import refine_registration
import integrate_scene

if __name__ == "__main__":
    # 读取配置文件
    fn_config = "config/orbbec.json"
    with open(fn_config) as json_file:
        config = json.load(json_file)
        initialize_config(config)
        check_folder_structure(config["path_dataset"])
    
    # 打印配置信息
    print("====================================")
    print("Configuration")
    print("====================================")
    for key, val in config.items():
        print("%40s : %s" % (key, str(val)))

    # 3D重建的主体内容
    times = [0, 0, 0, 0]
    start_time = time.time()
    # 1. 生成场景片段
    make_fragments.run(config)
    times[0] = time.time() - start_time
    start_time = time.time()
    # 2. 配准场景片段
    register_fragments.run(config)
    times[1] = time.time() - start_time
    start_time = time.time()
    # 3. 改善配准结果
    refine_registration.run(config)
    times[2] = time.time() - start_time
    start_time = time.time()
    # 4. 整合片段并重建场景
    integrate_scene.run(config)
    times[3] = time.time() - start_time
    
    # 打印每部分的耗时
    print("====================================")
    print("Elapsed time (in h:m:s)")
    print("====================================")
    print("- Making fragments    %s" % datetime.timedelta(seconds=times[0]))
    print("- Register fragments  %s" % datetime.timedelta(seconds=times[1]))
    print("- Refine registration %s" % datetime.timedelta(seconds=times[2]))
    print("- Integrate frames    %s" % datetime.timedelta(seconds=times[3]))
    print("- Total               %s" % datetime.timedelta(seconds=sum(times)))
    sys.stdout.flush()
