目录目录一、背景介绍1. 项目概述2. 产品化3. 未来市场潜力二、设备使用情况三、系统架构四、部署环境五、方案材料1. 系统构成2. 关键技术创新点相机标定手眼标定引导思路点云匹配3. 算法设计相机标定部分手眼标定部分点云匹配部分点云匹配的重要优化引导部分4. 测试结果六、研发过程记录1. 上手篇1.1点云相机上手1.2点云相机的工具1.3点云相机的SDKwin10的AstraSDKwin10的OpenNI2SDK1.4开发板上手armbian的AstraSDKarmbian的OpenNi2SDK1.5双目相机标定自制标定板执行双目相机标定验证标定精度2. 室内重建篇2.1概述2.2RGBD图采集2.3室内重建3. 视觉引导篇3.1概述3.2系统构成3.3机器人做TCP3.4手眼标定3.5点云匹配&实施引导3.6代码运行环境准备点云采集点云匹配七、技术优势分析八、基于开发套件产生的bug及建议已知bug和问题记录摄像头和开发板性能限制影响开发的问题点描述建议一、背景介绍1. 项目概述项目主题是《基于点云的视觉引导系统》。功能目标是基于点云的轨迹引导，即无论待引导物体以何种位姿摆放（要求该位姿在机械臂的行程范围内），视觉系统均能定位到该物体，并引导机械臂按需要的轨迹实现一定的工艺流程（比如鞋底涂胶等）。应用场景是鞋底涂胶等需要轨迹引导的工业现场。2. 产品化本项目实际引导精度在5mm左右，完全能满足工业现场下cm级的引导需求。如果需要产品化，还需要做的工作是做开发板与机械臂之间的通讯。本项目使用的kuka机械臂，TCP通讯需要安装库卡官方提供的软件包EthernetKRL，而该软件包只有windows版本，没有linux版本。如果想在linux下实现通讯，需要插相应的扩展板卡，走PROFINET通讯。由于时间和条件有限，本项目没有做开发板与机械臂之间的通讯，而是将开发板算出的结果显示在屏幕上，手工通过机器人示教器写入。如果是类似UR机械臂等原生支持TCP通讯的，只要写个简单的socket程序即可，并不需要额外的硬件，在linux下实现起来也较为容易，没有太多限制。3. 未来市场潜力近些年来，随着机器视觉和工业机器人得到越来越广泛的使用，机器换人的趋势日趋明显。工业机器人可以准确高效地完成重复性的工作，但缺乏柔性，故需要添加机器视觉系统加以辅助。而2D图像缺乏深度信息，常常难以满足六自由度的引导要求，故需要3D点

![image-20210406115544095](C:\Users\wangxiong\AppData\Roaming\Typora\typora-user-images\image-20210406115544095.png)