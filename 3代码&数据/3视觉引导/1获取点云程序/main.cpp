#include <stdio.h>
#include <OpenNI.h>

#include "OniSampleUtilities.h"

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

using namespace openni;

// 配置参数
#define MIN_DISTANCE 20     //单位毫米
#define MAX_DISTANCE 4000   //单位毫米
#define RESOULTION_X 640.0  //标定时的分辨率
#define RESOULTION_Y 480.0  //标定时的分辨率
#define FRAME_COUNT 5       //采集5张深度图, 保存第5张深度图为点云

// 红外相机的内参
const float	g_IrParam_f_x = 572.35087383;
const float	g_IrParam_f_y = 576.61281751;
const float	g_IrParam_c_x = 308.60311538;
const float	g_IrParam_c_y = 250.66719589;

// 把深度图转换为点云, 并保存为ply格式的文件
void convertDepthToPointCloudAndSavePly(const uint16_t *pDepth, int width, int height, const char *ply_filename)
{
	if (NULL == pDepth)
	{
		printf("depth frame is NULL!");
		return;
	}
	//将深度数据转换为点云数据,并将点云数据保存到文件中
	FILE *fp;

	//int res = fopen_s(&fp, ply_filename, "w");
	fp = fopen(ply_filename, "w");

	int valid_count = 0;
	uint16_t max_depth = MAX_DISTANCE;
	uint16_t min_depth = MIN_DISTANCE;

	//统计有效的数据条数
	//int img_size = width * height;
	for (int v = 0; v < height; ++v)
	{
		for (int u = 0; u < width; ++u)
		{
			uint16_t depth = pDepth[v * width + u];
			if (depth <= 0 || depth < min_depth || depth > max_depth)
				continue;

			valid_count += 1;
		}
	}

	//ply点云文件的头
	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "element vertex %d\n", valid_count);
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "end_header\n");

	float world_x, world_y, world_z;
	for (int v = 0; v < height; ++v)
	{
		for (int u = 0; u < width; ++u)
		{
			uint16_t depth = pDepth[v * width + u];
			if (depth <= 0 || depth < min_depth || depth > max_depth)
				continue;

			//分辨率缩放，这里假设标定时的分辨率分RESOULTION_X，RESOULTION_Y
			float fdx = g_IrParam_f_x * ((float)(width) / RESOULTION_X);
			float fdy = g_IrParam_f_y * ((float)(height) / RESOULTION_Y);
			float u0 = g_IrParam_c_x * ((float)(width) / RESOULTION_X);
			float v0 = g_IrParam_c_y * ((float)(height) / RESOULTION_Y);

			float tx = (u - u0) / fdx;
			float ty = (v - v0) / fdy;

			world_x = depth * tx;
			world_y = depth * ty;
			world_z = depth;
			fprintf(fp, "%f %f %f\n", world_x, world_y, world_z);
		}
	}

	fclose(fp);
}

int main(int argc, char* argv[])
{
	Status rc = OpenNI::initialize();
	if (rc != STATUS_OK)
	{
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		return 1;
	}

	Device device;

	if (argc < 2)
		rc = device.open(ANY_DEVICE);
	else
		rc = device.open(argv[1]);

	if (rc != STATUS_OK)
	{
		printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
		return 2;
	}

	VideoStream depth;

	if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
	{
		rc = depth.create(device, SENSOR_DEPTH);
		if (rc != STATUS_OK)
		{
			printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
			return 3;
		}
	}

	rc = depth.start();
	if (rc != STATUS_OK)
	{
		printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
		return 4;
	}

	VideoFrameRef frame;

	//while (!wasKeyboardHit())
	for (int IDX_FRAME = 1; IDX_FRAME < FRAME_COUNT + 1; IDX_FRAME++)
	{
		int changedStreamDummy;
		VideoStream* pStream = &depth;
		rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
		if (rc != STATUS_OK)
		{
			printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
			continue;
		}

		rc = depth.readFrame(&frame);
		if (rc != STATUS_OK)
		{
			printf("Read failed!\n%s\n", OpenNI::getExtendedError());
			continue;
		}

		if (frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_1_MM && frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_100_UM)
		{
			printf("Unexpected frame format\n");
			continue;
		}

		DepthPixel* pDepth = (DepthPixel*)frame.getData();

		int middleIndex = (frame.getHeight() + 1)*frame.getWidth() / 2;

		printf("%d:: [%08llu] %8d\n", IDX_FRAME, (long long)frame.getTimestamp(), pDepth[middleIndex]);

		// 将深度数据转换为点云并保存成ply文件，每帧深度数据对应一个ply文件
		if (IDX_FRAME == FRAME_COUNT) {
			char plyFileName[256] = "pointcloud.ply"; // ply的文件名
			switch (frame.getVideoMode().getPixelFormat())
			{
			case PIXEL_FORMAT_DEPTH_1_MM: // 100
				convertDepthToPointCloudAndSavePly(pDepth, frame.getWidth(), frame.getHeight(), plyFileName);
				printf("Save pointcloud.ply OK\n");
				break;
			default:
				printf("Unknown format\n");
			}

			break;
		}
	}

	depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();

	return 0;
}
