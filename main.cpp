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

int record(int argc, char* argv[]) {
	std::vector<cam::GenCamInfo> camInfos;
	std::shared_ptr<cam::GenCamera> cameraPtr
		= cam::createCamera(cam::CameraModel::PointGrey_u3);
		//= cam::createCamera(cam::CameraModel::XIMEA_xiC);
	cameraPtr->init();
	// set camera setting
	cameraPtr->startCapture();
	cameraPtr->setFPS(-1, 10);
	cameraPtr->setAutoExposure(-1, cam::Status::on);
	//cameraPtr->setAutoExposureLevel(-1, 30);
	//cameraPtr->setAutoExposureCompensation(-1, cam::Status::on, -0.5);
	//cameraPtr->setAutoWhiteBalance(-1);
	cameraPtr->setWhiteBalance(-1, 1.6, 1, 2);
	cameraPtr->makeSetEffective();
	// set capturing setting
	cameraPtr->setCapturePurpose(cam::GenCamCapturePurpose::Recording);
	cameraPtr->setCamBufferType(cam::GenCamBufferType::JPEG);
	cameraPtr->setJPEGQuality(90, 0.25);
	cameraPtr->setCaptureMode(cam::GenCamCaptureMode::Continous, 2000);
	cameraPtr->setVerbose(false);
	cameraPtr->makeSetEffective();
    cameraPtr->getCamInfos(camInfos);
	cam::SysUtil::sleep(1000);
	cameraPtr->startCaptureThreads();
	// wait for recoding to finish
	cameraPtr->waitForRecordFinish();
	//cameraPtr->saveImages("test_img");
	cameraPtr->saveVideos("saved");
    for(int i = 0; i < camInfos.size();i++) {
        printf("%d:%s\n",i, camInfos[i].sn.c_str());
		printf("%d: width:%d height:%d\n", i, camInfos[i].width, camInfos[i].height);
    }
	cameraPtr->stopCaptureThreads();
	cameraPtr->release();
	return 0;
}

int record_server(int argc, char* argv[]) {
	// init socket

	std::vector<cam::GenCamInfo> camInfos;
	std::shared_ptr<cam::GenCamera> cameraPtr
		= cam::createCamera(cam::CameraModel::PointGrey_u3);
		//= cam::createCamera(cam::CameraModel::XIMEA_xiC);
	cameraPtr->init();
	// set camera setting
	cameraPtr->startCapture();
	cameraPtr->setFPS(-1, 10);
	cameraPtr->setAutoExposure(-1, cam::Status::on);
	//cameraPtr->setAutoExposureLevel(-1, 30);
	//cameraPtr->setAutoExposureCompensation(-1, cam::Status::on, -0.5);
	//cameraPtr->setAutoWhiteBalance(-1);
	cameraPtr->setWhiteBalance(-1, 1.6, 1, 2);
	cameraPtr->makeSetEffective();
	// set capturing setting
	cameraPtr->setCapturePurpose(cam::GenCamCapturePurpose::Recording);
	cameraPtr->setCamBufferType(cam::GenCamBufferType::JPEG);
	cameraPtr->setJPEGQuality(90, 0.25);
	cameraPtr->setCaptureMode(cam::GenCamCaptureMode::Continous, 2000);
	cameraPtr->setVerbose(false);
	cameraPtr->makeSetEffective();
    cameraPtr->getCamInfos(camInfos);
	cam::SysUtil::sleep(1000);
	cameraPtr->startCaptureThreads();

	// add socket code here to set variable to start capturing 


	// wait for recoding to finish
	cameraPtr->waitForRecordFinish();
	//cameraPtr->saveImages("test_img");
	cameraPtr->saveVideos("saved");
    for(int i = 0; i < camInfos.size();i++) {
        printf("%d:%s\n",i, camInfos[i].sn.c_str());
		printf("%d: width:%d height:%d\n", i, camInfos[i].width, camInfos[i].height);
    }
	cameraPtr->stopCaptureThreads();
	cameraPtr->release();
	return 0;

int main(int argc, char* argv[]) {
	//preview(argc, argv);
	record(argc, argv);
	//testFileCamera();
	return 0;
}

