

// include std
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <queue>
#include <thread>
#include <memory>

// opencv
#include <opencv2/opencv.hpp>

#include "SKEncoder.h"

#define FRAME_NUM 300

int main(int argc, char* argv[]) 
{
	//cv::VideoCapture at("t.mp4");
	//cv::Mat tt;
	//int cc = at.get(cv::CAP_PROP_FRAME_WIDTH);
	//at >> tt;



	SKEncoder encoder;
	//const int frameNum = 100;
	const int w = 4112, h = 3008;
	cv::cuda::GpuMat gm[FRAME_NUM][3];
	for (int i = 0; i < FRAME_NUM; i++)
	{
		cv::Mat im = cv::imread(SysUtil::format("data/CUCAU1731016_00_%05d.jpg", i));
		cv::Mat YUV;
		cv::cvtColor(im, YUV, cv::COLOR_BGR2YUV_I420);
		//cv::cvtColor(im, YUV, cv::COLOR_BGR2YUV);

		cv::Mat Y = YUV(cv::Rect(0, 0, im.cols, im.rows));
		cv::Mat U = YUV(cv::Rect(0, im.rows, im.cols, im.rows / 4));
		U = U.reshape(1, im.rows / 2);
		cv::Mat V = YUV(cv::Rect(0, im.rows + im.rows / 4, im.cols, im.rows / 4));
		V = V.reshape(1, im.rows / 2);

		gm[i][0].upload(Y);
		gm[i][1].upload(U);
		gm[i][2].upload(V);

		if(i % 20 == 0)
			SysUtil::infoOutput(SysUtil::format("Buffering Data Frame = %d, All = %d", i, FRAME_NUM));

	}
	SysUtil::infoOutput("Data All Buffered, Start Encoding.");

	encoder.init(FRAME_NUM, cv::Size(w, h));
	for (int i = 0; i < FRAME_NUM; i++)
	{
		std::vector<void*> YUV_pointer(3);
		std::vector<uint32_t> steps(3);

		for (int j = 0; j < 3; j++)
		{
			YUV_pointer[j] = gm[i][j].data;
			steps[j] = gm[i][j].step;
		}
		encoder.encode(YUV_pointer, steps);
	}
	encoder.endEncode();

	getchar();

	return 0;
}
