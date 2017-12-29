// include std
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <queue>
#include <thread>
#include <memory>

// opencv
#include <opencv2/opencv.hpp>

// cuda
#ifdef _WIN32
#include <windows.h>
#endif
#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include "GenCameraDriver.h"

int main(int argc, char* argv[]) {
	std::shared_ptr<cam::GenCamera> cameraPtr 
		= cam::createCamera(cam::CameraModel::XIMEA_xiC);
	cameraPtr->init();
	cameraPtr->startCapture();
	std::vector<cv::Mat> imgs(2);
	cameraPtr->captureOneFrameBayer(imgs);
	cameraPtr->release();
	return 0;
}

