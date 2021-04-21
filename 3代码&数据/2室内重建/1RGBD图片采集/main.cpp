#include "AstraProD2C.h"
#include "AstraD2C.h"
#include "depth_to_color.h"

#include "Draw.h"
#include <string>
using namespace std;

//注意：由于软件D2C需要将设备固件设置成非镜像输出，如果uvc固件Color是镜像输出，那么再Depth和color图像叠加时，需要将Depth镜像显示
//color图像是否镜像输出：0：表示非镜像输出；1表示镜像输出。

#define COLOR_IS_MIRROR  1  

// 配置的两个参数
const int FRAME_COUNT = 200; // 采集200张图片
const int TIME_WAIT = 100; // 每帧之间等待100ms



//裁掉左边80列,
Mat myROI(Mat src)
{
	Rect rect(80, 0, src.cols - 80, src.rows);
	Mat temp(src, rect);

	return temp;
}

//顺时帧旋转90度 atlas depth分辨率是400x640，rgb是640x400，软件D2C时，需要将rgb先顺时帧旋转90度，然后裁剪rgb的左边80列，和depth对齐
Mat matRotateClockWise90(Mat src)
{
	if (src.empty())
	{
		printf("RorateMat src is empty!");
	}

	// 矩阵转置
	transpose(src, src);
	//0: 沿X轴翻转； >0: 沿Y轴翻转； <0: 沿X轴和Y轴翻转

	// 翻转模式，flipCode == 0垂直翻转（沿X轴翻转），flipCode>0水平翻转（沿Y轴翻转），
	//flipCode<0水平垂直翻转（先沿X轴翻转，再沿Y轴翻转，等价于旋转180°）
	flip(src, src, 1);

	return src;
}

int main(int argc, char *argv[])
{
	//
	cout << " *********** soft d2c v1.0*********" << endl << endl;
	printf("D2C.exe  [isUvc:0/1 ] [colorMirror: 0/1]\n");
	//
	cout << " Esc - Exit" << endl << endl;
	int nUvc = 1;
	int nColorMirror = 0; // +++ 原始: 0
	if (argc != 3)
	{
		printf("D2C.exe  [isUvc (0/1) ] [colorMirror: (0/1)]\n");
	}
	else
	{
		//color is uvc device
		nUvc = atoi(argv[1]);
		//color is mirror
		nColorMirror = atoi(argv[2]);
	}


	DepthToColor d2c;
	Draw mDraw;

	printf("Version: %s\n", d2c.GetVersion());
	d2cSwapper *pD2CSwapper = NULL;
	
	if (nUvc == 1)
	{
		//color is uvc
		pD2CSwapper = new AstroProD2C();
	}
	else
	{
		pD2CSwapper = new AstraD2C();
	}
	
	int nRet = pD2CSwapper->CameraInit(SOFTWARE_D2C);
	if (nRet != CAMERA_STATUS_SUCCESS)
	{
		printf("camera init failed\n");
		getchar();
		return -1;
	}

	//获取相机内外参
	SW_D2C soft_d2c;
	memset(&soft_d2c, 0, sizeof(SW_D2C));
	OBCameraParams cameraParams;
	memset(&cameraParams, 0, sizeof(OBCameraParams));

	nRet = pD2CSwapper->GetCameraParam(cameraParams);
	if (nRet != 0)
	{
		//获取相机内外参失败
		printf("get camera param failed\n");
		return -1;
	}

	// +++ 自己给相机参数的变量赋值
	// 我双目标定出来的参数, 自制标定板, 保留70%的角点, 重投影误差0.387
	// L是IR图，R是彩色图
	// [fx,fy,cx,cy]
	cameraParams.l_intr_p[0] = 572.35087383;
	cameraParams.l_intr_p[1] = 576.61281751;
	cameraParams.l_intr_p[2] = 308.60311538;
	cameraParams.l_intr_p[3] = 250.66719589;
	// [fx, fy, cx, cy]
	cameraParams.r_intr_p[0] = 605.38898333;
	cameraParams.r_intr_p[1] = 607.6177079;
	cameraParams.r_intr_p[2] = 306.20670369;
	cameraParams.r_intr_p[3] = 238.36161457;
	// [r00,r01,r02;r10,r11,r12;r20,r21,r22]
	cameraParams.r2l_r[0] = 0.99985107;
	cameraParams.r2l_r[1] = 0.01379429;
	cameraParams.r2l_r[2] = 0.01037083;
	cameraParams.r2l_r[3] = -0.01372216;
	cameraParams.r2l_r[4] = 0.99988138;
	cameraParams.r2l_r[5] = -0.00699429;
	cameraParams.r2l_r[6] = -0.01046608;
	cameraParams.r2l_r[7] = 0.00685094;
	cameraParams.r2l_r[8] = 0.99992176;
	// [t1,t2,t3]
	cameraParams.r2l_t[0] = -25.37136205;
	cameraParams.r2l_t[1] = -1.52660596;
	cameraParams.r2l_t[2] = -4.54607117;
	// [k1,k2,p1,p2,k3]
	cameraParams.l_k[0] = -0.17546213;
	cameraParams.l_k[1] = 1.23176247;
	cameraParams.l_k[2] = -0.00450702;
	cameraParams.l_k[3] = -0.00668543;
	cameraParams.l_k[4] = -2.7598227;
	// [k1,k2,p1,p2,k3]
	cameraParams.r_k[0] = 0.00285554;
	cameraParams.r_k[1] = 0.71769527;
	cameraParams.r_k[2] = -0.00521712;
	cameraParams.r_k[3] = -0.0021043;
	cameraParams.r_k[4] = -1.96128222;


	//获取设备的分辨率
	int nWidth = 0;
	int nHeight = 0;
	nRet = pD2CSwapper->GetCameraResolution(nWidth, nHeight);
	
	if (nRet !=0)
	{
		printf("get camera resolution fail,please check pid\n");
		return -1;
	}

	//
	memcpy(&soft_d2c, &cameraParams, sizeof(SW_D2C));
	//调用D2C库 装载参数

	bool success = d2c.LoadParameters((SW_D2C *)&soft_d2c);

	if (!success)
	{
		fprintf(stderr, "LoadParameters failed!\n");
		return -1;
	}

	
	success = d2c.PrepareDepthResolution(nWidth, nHeight);  //640x400
	if (!success)
	{
		fprintf(stderr, "PrepareDepthResolution failed!\n");
		return -1;
	}

	// 关闭畸变
	success = d2c.EnableDistortion(false);
	if (!success)
	{
		fprintf(stderr, "EnableDistortion failed!\n");
		return -1;
	}
	// fill gaps
	success = d2c.EnableDepthGapFill(true);
	if (!success)
	{
		fprintf(stderr, "EnableDepthGapFill failed!\n");
		return -1;
	}

	// 每个深度单位=1.0 mm
	success = d2c.SetDepthUnit(1);
	if (!success)
	{
		fprintf(stderr, "EnableDistortion failed!\n");
		return -1;
	}

	// 设置原始深度图中，感兴趣的深度范围，根据实际产品的深度范围设置
	success = d2c.SetDepthRange(300.0, 9000.0);  //单位mm +++ (300.0, 2900.0)
	if (!success)
	{
		fprintf(stderr, "EnableDistortion failed!\n");
		return -1;
	}

	//
	const char * _winTitle = (const char *)("D2C Viewer");

	bool rgbMode = true;
	IplImage IplColor, IplDepth, IplReg;
	char chrDepthColorAlpha[32] = { 0 };
	const char* strRegisterInfo = "";
	char strRegisterType[100] = "";

	Vector<IplImage> imgsOne(1);
	Vector<IplImage> imgsTwo(2);
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);

	bool bExit = true;
	uint16_t nPid = pD2CSwapper->GetDevicePid();
	//开关流
	int nDepthWidth = 0;
	int nDepthHeight = 0;

	int nD2CDepthWidth = 0;
	int nD2CDepthHeight = 0;

	//atlas : depth 400x640,rgb 640x480,需要特殊处理
	if (nPid == ATLAS_PID)
	{
		nDepthWidth = IMAGE_WIDTH_400;
		nDepthHeight = IMAGE_HEIGHT_640;

		nD2CDepthWidth = IMAGE_WIDTH_480;
		nD2CDepthHeight = IMAGE_HEIGHT_640;
	}
	else
	{
		nDepthWidth = IMAGE_WIDTH_640;
		nDepthHeight = IMAGE_HEIGHT_480;

		nD2CDepthWidth = IMAGE_WIDTH_640;
		nD2CDepthHeight = IMAGE_HEIGHT_480;
	}



	//while (bExit)
	for (int count = 0; count < FRAME_COUNT+1; count++)
	{
		clock_t time_one_start = clock();

		cv::Mat cv_rgb(IMAGE_HEIGHT_480, IMAGE_WIDTH_640, CV_8UC3);
		cv::Mat cv_depth(nDepthHeight, nDepthWidth, CV_16UC1); //640x480
		cv::Mat cv_rgb_copy; // +++ 用于保存的RGB图片

		cv::Mat aligned_depth(nD2CDepthHeight, nD2CDepthWidth, CV_16UC1);	    //分配的应该为RGB的宽和高,保存D2C后的Depth
		cv::Mat CaliDepthHistogram(nD2CDepthHeight, nD2CDepthWidth, CV_16UC1);  //分配的内存和aligned_depth一样

		int nRet = pD2CSwapper->GetStreamData(cv_rgb, cv_depth);

		if (nPid == ATLAS_PID)
		{
			//旋转rgb
			cv_rgb = matRotateClockWise90(cv_rgb);
		}

		if (nRet == CAMERA_STATUS_SUCCESS)
		{
			//执行D2C的操作
			int ret = d2c.D2C(cv_depth.ptr<uint16_t>(), cv_depth.cols, cv_depth.rows,
				aligned_depth.ptr<uint16_t>(), cv_rgb.cols, cv_rgb.rows);

			if (ret)
			{
				fprintf(stderr, "D2C failed!\n");
				return -1;
			}

			//Depth是否需要镜像，如果color rgb是镜像输出，那么Depth也要镜像

			if (nColorMirror == COLOR_IS_MIRROR)	//1表示镜像，0表示非镜像
			{
				cv::flip(aligned_depth, aligned_depth, 1);
			}


			//opencv 绘图
			mDraw.GetDepthHistogram(aligned_depth, CaliDepthHistogram);

			//atlas 设备软件D2C后，裁剪最左边的80列
			if (nPid == ATLAS_PID)
			{
				cv_rgb = myROI(cv_rgb);
				CaliDepthHistogram = myROI(CaliDepthHistogram);
			}

			Mat imgROI = cv_rgb(Rect(0, 0, CaliDepthHistogram.cols, CaliDepthHistogram.rows));
			cv_rgb_copy = cv_rgb.clone(); // +++ 拷贝一份彩色图
			cv::addWeighted(imgROI, 0.5, CaliDepthHistogram, 0.5, 0.0, imgROI); // +++ 此行改变了cv_rgb
			IplReg = imgROI;
			imgsOne[0] = IplReg;

			IplColor = cv_rgb;
			IplDepth = CaliDepthHistogram;
			imgsTwo[0] = IplColor;
			imgsTwo[1] = IplDepth;

			// Draw info on windows

			cvPutText(&IplReg, strRegisterType, cvPoint(50, 400), &font, cvScalar(0, 0, 255, 255));
			cvPutText(&IplReg, strRegisterInfo, cvPoint(50, 430), &font, cvScalar(0, 0, 255, 255));
			cvPutText(&IplReg, chrDepthColorAlpha, cvPoint(10, 30), &font, cvScalar(0, 0, 255, 255));
			cvPutText(&IplReg, "Over", cvPoint(10, 470), &font, cvScalar(0, 0, 255, 255));

			mDraw.ShowImagesSideBySide(_winTitle,  imgsOne, "", 10, 450);

		}
		else
		{
			printf("receive depth and rgb failed\n");
		}

		//按ESC，退出程序
		int c = cv::waitKey(3);
		if (27 == char(c))
			break;

		// +++ 我加的
		cv::imwrite("cv_rgb_" + std::to_string(count) + ".png", cv_rgb_copy);
		cv::imwrite("cv_depth_" + std::to_string(count) + ".png", cv_depth);
		cv::imwrite("aligned_depth_" + std::to_string(count) + ".png", aligned_depth);
		cv::imwrite("CaliDepthHistogram_" + std::to_string(count) + ".png", CaliDepthHistogram);
	
		_sleep(TIME_WAIT); //延时0.1秒

		clock_t time_one_end = clock();
		float time_one_cost = (double)(time_one_end - time_one_start) / CLOCKS_PER_SEC * 1000;

		printf("Count [%d] Cost Time %.1f ms \n", count, time_one_cost);
	}

	nRet = pD2CSwapper->StreamStop();
	printf("stop stream :%d\n", nRet);
	nRet = pD2CSwapper->CameraUnInit();
	printf("UvcDeInit :%d\n", nRet);

	//
	if (pD2CSwapper)
	{
		delete pD2CSwapper;
		pD2CSwapper = NULL;
	}

	return 0;
}