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
#include "LinuxSocket.hpp"

inline std::string getTimeString()
{
	time_t timep;
	time(&timep);
	char tmp[64];
	strftime(tmp, sizeof(tmp), "__%Y_%m_%d_%H_%M_%S__", localtime(&timep));
	return tmp;
}

int preview(int argc, char* argv[]) {
	// init buffer
	std::vector<cam::Imagedata> imgdatas(2);
	std::vector<cv::Mat> imgs(2);
	// init camera
	std::vector<cam::GenCamInfo> camInfos;
	std::shared_ptr<cam::GenCamera> cameraPtr 
		= cam::createCamera(cam::CameraModel::PointGrey_u3);
	cameraPtr->init();
	// set camera setting
	cameraPtr->startCapture();
	cameraPtr->setFPS(-1, 20);
	cameraPtr->setAutoExposure(-1, cam::Status::on);
	cameraPtr->setAutoExposureLevel(-1, 40);
	cameraPtr->setAutoWhiteBalance(-1);
	cameraPtr->makeSetEffective();
	// set capturing setting
	cameraPtr->setCaptureMode(cam::GenCamCaptureMode::Continous, 200);
	// get camera info
	cameraPtr->getCamInfos(camInfos);
	cameraPtr->startCaptureThreads();
	// capture frames
	for (size_t i = 0; i < 2000; i++) {
		cameraPtr->captureFrame(imgdatas);
		imgs[0] = cv::Mat(camInfos[0].height, camInfos[0].width,
			CV_8U, reinterpret_cast<void*>(imgdatas[0].data));
		imgs[1] = cv::Mat(camInfos[1].height, camInfos[1].width,
			CV_8U, reinterpret_cast<void*>(imgdatas[1].data));
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
	std::vector<std::vector<cam::GenCamInfo>> A_camInfos;
	std::vector<std::shared_ptr<cam::GenCamera>> A_cameraPtr;
	bool wait = false, video = false;
	int port = 0;
	int frameNum = 500;
	LxSoc lxsoc;
	if (argc == 1)
	{
		A_cameraPtr.push_back(cam::createCamera(cam::CameraModel::XIMEA_xiC));
	}
	else
	{
		for (int i = 1; i < argc; i++)
		{
			std::string t = std::string(argv[i]);
			if (t == "XIMEA" || t == "x")
				A_cameraPtr.push_back(cam::createCamera(cam::CameraModel::XIMEA_xiC));
			else if (t == "PTGREY" || t == "PointGrey" || t == "p")
				A_cameraPtr.push_back(cam::createCamera(cam::CameraModel::PointGrey_u3));
			else if (t == "Stereo" || t == "STEREO" || t == "s")
				A_cameraPtr.push_back(cam::createCamera(cam::CameraModel::Stereo));
			else if (t == "wait")
			{
				//format : wait [port = %d] [frameNum = %d]
				wait = true;
				if (i + 2 >= argc)
				{
					cam::SysUtil::errorOutput("when use wait mode, please specify port & frameNum\nSample: ./GenCameraDriver XIMEA wait 22336 200");
					return -1;
				}
				port = atoi(argv[i + 1]);
				frameNum = atoi(argv[i + 2]);
				i += 2;
				lxsoc.init(port);
			}
			else if (t == "video" || t == "v")
			{
				video = true;
			}
		}
	}

	//output
	{
		cam::SysUtil::infoOutput(video ? ("Video Save Mode ON") : ("Images Save Mode ON"));
#ifndef WIN32
		if(wait)
			cam::SysUtil::infoOutput(cv::format("Wait Mode ON, will wait on port %d", port));
#endif
		for (int i = 0; i < A_cameraPtr.size(); i++)
		{
			cam::SysUtil::infoOutput("Will add camera type = " + A_cameraPtr[i]->getCamModelString());
		}
	}

	for (int i = 0; i < A_cameraPtr.size(); i++)
	{
		std::vector<cam::GenCamImgRatio> imgRatios;
		std::vector<cam::GenCamInfo> camInfos;
		std::shared_ptr<cam::GenCamera> cameraPtr = A_cameraPtr[i];
		cameraPtr->init();
		// set camera setting

		cameraPtr->setFPS(-1, 10);
		cameraPtr->startCapture();
		//cameraPtr->setFPS(-1, 10);
		cameraPtr->setAutoExposure(-1, cam::Status::on);
		cameraPtr->setAutoExposureLevel(-1, 30);
		//cameraPtr->setAutoExposureCompensation(-1, cam::Status::on, -0.5);
		//cameraPtr->setAutoWhiteBalance(-1);
		cameraPtr->setWhiteBalance(-1, 2.0, 1.0, 2.0);
		cameraPtr->makeSetEffective();
		// set capturing setting
		cameraPtr->setCamBufferType(cam::GenCamBufferType::JPEG);
		cameraPtr->setJPEGQuality(90, 0.75);
		cameraPtr->setCaptureMode(cam::GenCamCaptureMode::Continous, frameNum);
		cameraPtr->setCapturePurpose(cam::GenCamCapturePurpose::Recording);
		cameraPtr->setVerbose(false);
		cameraPtr->makeSetEffective();
		cameraPtr->getCamInfos(camInfos);

		if (cameraPtr->getCamModelString() == "   XIMEA_xiC" && cam::SysUtil::existFile("./mul_mat.tiff"))
		{
			cv::Mat mul = cv::imread("./mul_mat.tiff", cv::IMREAD_UNCHANGED);
			cv::cuda::GpuMat mul_cuda_(mul);
			std::vector<cv::cuda::GpuMat> muls(camInfos.size(), mul_cuda_);
			cameraPtr->setBrightnessAdjustment(muls);

		}
		A_camInfos.push_back(camInfos);
		cam::SysUtil::sleep(1000);
		// set image ratios
		imgRatios.resize(camInfos.size());
		for (size_t i = 0; i < camInfos.size(); i++) {
			imgRatios[i] = static_cast<cam::GenCamImgRatio>(0);
			//imgRatios[i] = cam::GenCamImgRatio::Octopus;
		}
		cameraPtr->setImageRatios(imgRatios);
	}

	for (int i = 0; i < A_cameraPtr.size(); i++)
	{
		std::vector<cam::GenCamInfo> camInfos = A_camInfos[i];
		std::shared_ptr<cam::GenCamera> cameraPtr = A_cameraPtr[i];
		if (wait)
			cameraPtr->isStartRecord = false;
		cameraPtr->startCaptureThreads();
	}
	if (wait)
	{
		while (lxsoc.waitFor("Action") != 1);
		for (int i = 0; i < A_cameraPtr.size(); i++)
		{
			A_cameraPtr[i]->isStartRecord = true;
		}
	}

	std::string timeStr = getTimeString();

	for (int i = 0; i < A_cameraPtr.size(); i++)
	{
		std::vector<cam::GenCamInfo> camInfos = A_camInfos[i];
		std::shared_ptr<cam::GenCamera> cameraPtr = A_cameraPtr[i];
		cameraPtr->waitForRecordFinish();
		if(!video)
			cameraPtr->saveImages(timeStr);
		else
			cameraPtr->saveVideosGpu(timeStr);
		for (int i = 0; i < camInfos.size(); i++) {
			printf("%d:%s\n", i, camInfos[i].sn.c_str());
			printf("%d: width:%d height:%d\n", i, camInfos[i].width, camInfos[i].height);

		}
		cameraPtr->stopCaptureThreads();
		cameraPtr->stopCapture();
		cameraPtr->release();
	}
	return 0;
}

int testFileCamera() {
	std::vector<cam::GenCamInfo> camInfos;
	std::shared_ptr<cam::GenCamera> cameraPtr
		= cam::createCamera(cam::CameraModel::File, "E:/data/giga/NanshanIPark/2");
	cameraPtr->init();
	// set camera setting
	cameraPtr->startCapture();
	cameraPtr->setFPS(-1, 10);
	cameraPtr->setAutoExposure(-1, cam::Status::on);
	cameraPtr->setAutoExposureLevel(-1, 25);
	cameraPtr->makeSetEffective();
	// set capturing setting
	cameraPtr->setCamBufferType(cam::GenCamBufferType::Raw);
	cameraPtr->setJPEGQuality(85, 0.15);
	cameraPtr->setCaptureMode(cam::GenCamCaptureMode::Continous, 10);
	cameraPtr->setCapturePurpose(cam::GenCamCapturePurpose::Recording);
	cameraPtr->setVerbose(false);
	cameraPtr->makeSetEffective();
	cameraPtr->getCamInfos(camInfos);
	cam::SysUtil::sleep(1000);
	cameraPtr->startCaptureThreads();
	// wait for recoding to finish
	cameraPtr->waitForRecordFinish();
	// get images
	for (int i = 0; i < 10; i++) {
		std::vector<cam::Imagedata> imgs(7);
		cameraPtr->captureFrame(imgs);
		cv::Mat img0(1500, 2000, CV_8U, imgs[0].data);
		cv::Mat img1(1500, 2000, CV_8U, imgs[2].data);
		cv::Mat img2(1500, 2000, CV_8U, imgs[5].data);
		int a = 0;
		a++;
	}
	cameraPtr->stopCaptureThreads();
	cameraPtr->release();
	return 0;
}


int main(int argc, char* argv[]) {
	//preview(argc, argv);
	record(argc, argv);
	//testFileCamera();
	return 0;
}

