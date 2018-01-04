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

int preview(int argc, char* argv[]) {
	std::vector<cv::Mat> imgs(2);
	std::vector<cam::GenCamInfo> camInfos;
	std::shared_ptr<cam::GenCamera> cameraPtr 
		= cam::createCamera(cam::CameraModel::XIMEA_xiC);
	cameraPtr->init();
	// set camera setting
	cameraPtr->getCamInfos(camInfos);
	cameraPtr->startCapture();
	cameraPtr->setFPS(-1, 20);
	cameraPtr->setAutoExposure(-1, cam::Status::on);
	cameraPtr->setAutoExposureLevel(-1, 40);
	cameraPtr->setAutoWhiteBalance(-1);
	cameraPtr->makeSetEffective();
	// set capturing setting
	cameraPtr->setCaptureMode(cam::GenCamCaptureMode::Continous, 20);
	cameraPtr->startCaptureThreads();
	// capture frames
	for (size_t i = 0; i < 2000; i++) {
		cameraPtr->captureFrame(imgs);
		cv::Mat show1, show2;
		cv::resize(imgs[0], show1, cv::Size(400, 300));
		cv::resize(imgs[1], show2, cv::Size(400, 300));
		cv::imshow("1", show1);
		cv::imshow("2", show2);
		cv::waitKey(1);
	}
	cameraPtr->stopCaptureThreads();
	cameraPtr->release();
	return 0;
}

int record(int argc, char* argv[]) {
	std::vector<cv::Mat> imgs(2);
	std::vector<cam::GenCamInfo> camInfos;
	std::shared_ptr<cam::GenCamera> cameraPtr
		= cam::createCamera(cam::CameraModel::XIMEA_xiC);
	cameraPtr->init();
	// set camera setting
	cameraPtr->startCapture();
	cameraPtr->setFPS(-1, 20);
	cameraPtr->setAutoExposure(-1, cam::Status::on);
	cameraPtr->setAutoExposureLevel(-1, 25);
	cameraPtr->setAutoWhiteBalance(-1);
	cameraPtr->makeSetEffective();
	// set capturing setting
	cameraPtr->setCamBufferType(cam::GenCamBufferType::JPEG);
	cameraPtr->setJPEGQuality(85, 0.15);
	cameraPtr->setCaptureMode(cam::GenCamCaptureMode::Continous, 1000);
	cameraPtr->setCapturePurpose(cam::GenCamCapturePurpose::Recording);
	cameraPtr->setVerbose(true);
	cameraPtr->startCaptureThreads();
	// wait for recoding to finish
	cameraPtr->waitForRecordFinish();
	//cameraPtr->saveImages("test");
	cameraPtr->saveVideos("test");
	cameraPtr->stopCaptureThreads();
	cameraPtr->release();
	return 0;
}


int main(int argc, char* argv[]) {
	record(argc, argv);
	return 0;
}

