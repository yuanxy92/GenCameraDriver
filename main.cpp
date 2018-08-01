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

// camera driver
#include "GenCameraDriver.h"

// socket, linux only
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

// record for single motion 
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
	cameraPtr->setWhiteBalance(-1, 1.8, 1, 2);
	cameraPtr->makeSetEffective();
	// set capturing setting
	cameraPtr->setCapturePurpose(cam::GenCamCapturePurpose::Recording);
	cameraPtr->setCamBufferType(cam::GenCamBufferType::JPEG);
	cameraPtr->setJPEGQuality(90, 0.25);
	cameraPtr->setCaptureMode(cam::GenCamCaptureMode::Continous, 20);
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

// error print function
void error(const char *msg) {
    perror(msg);
    exit(1);
}

// record from network
int record_server(int argc, char* argv[]) {
	// init socket
	int sockfd, newsockfd, portno;
    socklen_t clilen;
    char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    int n;
    if (argc < 2) {
        fprintf(stderr,"ERROR, no port provided\n");
        exit(1);
    }
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    bzero((char *) &serv_addr, sizeof(serv_addr));
    portno = atoi(argv[1]);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
    	error("ERROR on binding");
	listen(sockfd, 5);

	int frameNum = 20;
	if (argc >= 3) {
		frameNum = atoi(argv[2]);
	}	

	// 
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
	cameraPtr->setCaptureMode(cam::GenCamCaptureMode::Continous, frameNum);
	cameraPtr->setVerbose(false);
	cameraPtr->makeSetEffective();
    cameraPtr->getCamInfos(camInfos);
	cam::SysUtil::sleep(1000);
	cameraPtr->startCaptureThreads();

	// add socket code here to set variable to start capturing 
	printf("Waiting for action command !\n");
	clilen = sizeof(cli_addr);
	newsockfd = accept(sockfd, 
                (struct sockaddr *) &cli_addr, 
                &clilen);
    if (newsockfd < 0) 
        error("ERROR on accept");
    bzero(buffer, 256);
    n = read(newsockfd, buffer, 255);
    if (n < 0) error("ERROR reading from socket");
	if (strcmp(buffer, "Action") == 0) {
		cameraPtr->isStartRecord = true;
    	printf("Here is the message: %s\n", buffer);
	}
	else {
		printf("Wrong command ! Here is the message: %s\n", buffer);
	}
	close(newsockfd);
    close(sockfd);

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

int main(int argc, char* argv[]) {
	//preview(argc, argv);
	record_server(argc, argv);
	//testFileCamera();
	return 0;
}

