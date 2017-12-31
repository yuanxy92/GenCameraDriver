/**
@brief General Camera Driver Class
@author Shane Yuan
@date Dec 29, 2017
*/

#include "GenCameraDriver.h"
#include "XIMEACamera.h"
#include "PointGreyCamera.h"
#include <time.h>

// cuda npp JPEG coder
#include "NPPJpegCoder.h"

namespace cam {
	/**
	@breif function to init camera array
	@return
	*/
	std::shared_ptr<GenCamera> createCamera(CameraModel camModel) {
		if (camModel == CameraModel::XIMEA_xiC) {
			std::shared_ptr<GenCameraXIMEA> cameraPtr = std::make_shared<GenCameraXIMEA>();
			return std::static_pointer_cast<GenCamera>(cameraPtr);
		}
		else if (camModel == CameraModel::PointGrey_u3) {
			std::shared_ptr<GenCameraPTGREY> cameraPtr = std::make_shared<GenCameraPTGREY>();
			return std::static_pointer_cast<GenCamera>(cameraPtr);
		}
	}

	GenCamera::GenCamera() : isInit(false), isCapture(false),
		isVerbose(false), bufferType(GenCamBufferType::Raw) {}
	GenCamera::~GenCamera() {}

	/**
	@brief multi-thread capturing function
	used for continous mode
	thread function to get images from camera and buffer to vector
	and wait until the next frame (based on fps)
	*/
	void GenCamera::capture_thread_(int camInd) {
		clock_t begin_time, end_time;
		double time = 1000.0 / static_cast<double>(camInfos[camInd].fps);
		thStatus[camInd] = true;
		for (;;) {
			begin_time = clock();
			// check status
			if (thStatus[camInd] == false)
				break;
			// capture image
			this->captureFrame(camInd, bufferImgs[thBufferInds[camInd]][camInd]);
			end_time = clock();
			float waitTime = time - static_cast<double>(end_time - begin_time) / CLOCKS_PER_SEC * 1000;
			if (waitTime > 0) {
				std::this_thread::sleep_for(std::chrono::milliseconds((long long)waitTime));
			}
			if (isVerbose) {
				printf("Camera %d captures one frame, wait %lld milliseconds for next frame ...\n",
					camInd, waitTime);
			}
			// increase index
			thBufferInds[camInd] = (thBufferInds[camInd] + 1) % bufferSize;
		}
	}

	/**
	@brief multi-thread captureing function
	used for single mode
	thread function to get images from camera and buffer to vector
	*/
	void GenCamera::capture_thread_single_(int camInd, cv::Mat & img) {
		// capture image
		this->captureFrame(camInd, img);
	}

	/**
	@brief set capturing mode
	@param GenCamCaptureMode captureMode: capture mode
	@param int size: buffer size
	@return
	*/
	int GenCamera::setCaptureMode(GenCamCaptureMode captureMode,
		int bufferSize) {
		// get camera info
		this->getCamInfos(camInfos);
		// init capture buffer
		this->captureMode = captureMode;
		this->bufferSize = bufferSize;
		if (captureMode == cam::GenCamCaptureMode::Continous ||
			captureMode == cam::GenCamCaptureMode::ContinousTrigger) {
			if (this->bufferType == GenCamBufferType::Raw) {
				// resize vector
				this->bufferImgs.resize(bufferSize);
				for (size_t i = 0; i < bufferSize; i++) {
					this->bufferImgs[i].resize(this->cameraNum);
				}
				// malloc mat memory
				for (size_t i = 0; i < this->cameraNum; i++) {
					int width, height;
					width = camInfos[i].width;
					height = camInfos[i].height;
					for (size_t j = 0; j < bufferSize; j++) {
						this->bufferImgs[j][i].create(height, width, CV_8U);
					}
				}
			}
			else if (this->bufferType == GenCamBufferType::JPEG) {

			}
		}
		else if (captureMode == cam::GenCamCaptureMode::Single ||
			captureMode == cam::GenCamCaptureMode::SingleTrigger) {
			this->ths.resize(this->cameraNum);
		}
		return 0;
	}

	/**
	@brief start capture threads
	@return int
	*/
	int GenCamera::startCaptureThreads() {
		if (captureMode == cam::GenCamCaptureMode::Continous ||
			captureMode == cam::GenCamCaptureMode::ContinousTrigger) {
			// prepare thread buffers
			ths.resize(this->cameraNum);
			thStatus.resize(this->cameraNum);
			thBufferInds.resize(this->cameraNum);
			for (size_t i = 0; i < this->cameraNum; i++) {
				thStatus[i] = false;
				thBufferInds[i] = 0;
			}
			// start capturing threads
			for (size_t i = 0; i < this->cameraNum; i++) {
				ths[i] = std::thread(&GenCamera::capture_thread_, this, i);
			}
		}
		else {
			SysUtil::warningOutput("This function is only valid when capture mode is " \
				"Continous or ContinousTrigger !");
		}
		return 0;
	}

	/**
	@brief stop capture threads
	@return int
	*/
	int GenCamera::stopCaptureThreads() {
		// set th status to false
		for (size_t i = 0; i < this->cameraNum; i++) {
			thStatus[i] = false;
		}
		// make sure all the threads are terminated
		for (size_t i = 0; i < this->cameraNum; i++) {
			ths[i].join();
			char info[256];
			sprintf(info, "Capturing thread %d terminate correctly !", i);
			SysUtil::infoOutput(std::string(info));
		}
		return 0;
	}

	/**
	@brief capture one frame
	@param std::vector<cv::Mat> & imgs: output captured images
	if in single mode, memory of image mats should be malloced
	before using this function
	@return int
	*/
	int GenCamera::captureFrame(std::vector<cv::Mat> & imgs) {
		if (captureMode == GenCamCaptureMode::Continous ||
			captureMode == GenCamCaptureMode::ContinousTrigger) {
			// get images from buffer
			for (size_t camInd = 0; camInd < this->cameraNum; camInd++) {
				imgs[camInd] = bufferImgs[thBufferInds[camInd]][camInd];
			}

		}
		else if (captureMode == GenCamCaptureMode::Single ||
			captureMode == GenCamCaptureMode::SingleTrigger) {
			// get images from camera
			for (size_t i = 0; i < this->cameraNum; i++) {
				ths[i] = std::thread(&GenCamera::capture_thread_single_, this, i, std::ref(imgs[i]));
			}
			// wait for all the threads to exit
			for (size_t i = 0; i < this->cameraNum; i++) {
				ths[i].join();
			}
		}
		return 0;
	}
}

