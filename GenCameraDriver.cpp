/**
@brief General Camera Driver Class
@author Shane Yuan
@date Dec 29, 2017
*/

#include "GenCameraDriver.h"
#include "XIMEACamera.h"
#include "PointGreyCamera.h"
#include <time.h>
#include <algorithm>
#include <functional>   // std::minus
#include <numeric>      // std::accumulate

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
			SysUtil::warningOutput("GenCamDriver is not funtional for PointGrey cameras yet!");
			std::shared_ptr<GenCameraPTGREY> cameraPtr = std::make_shared<GenCameraPTGREY>();
			return std::static_pointer_cast<GenCamera>(cameraPtr);
		}
	}

	GenCamera::GenCamera() : isInit(false), isCapture(false),
		isVerbose(false), bufferType(GenCamBufferType::Raw),
		camPurpose(GenCamCapturePurpose::Streaming) {}
	GenCamera::~GenCamera() {}

	/**
	@brief multi-thread capturing function
	used for continous mode
	thread function to get images from camera and buffer to vector
	and wait until the next frame (based on fps)
	@param int camInd: index of camera
	*/
	void GenCamera::capture_thread_raw_(int camInd) {
		clock_t begin_time, end_time;
		double time = 1000.0 / static_cast<double>(camInfos[camInd].fps);
		thStatus[camInd] = 1;
		for (;;) {
			begin_time = clock();
			// check status
			if (thStatus[camInd] == 0)
				break;
			// capture image
			this->captureFrame(camInd, bufferImgs[thBufferInds[camInd]][camInd]);
			end_time = clock();
			float waitTime = time - static_cast<double>(end_time - begin_time) / CLOCKS_PER_SEC * 1000;
			// increase index
			if (camPurpose == GenCamCapturePurpose::Streaming)
				thBufferInds[camInd] = (thBufferInds[camInd] + 1) % bufferSize;
			else {
				thBufferInds[camInd] = thBufferInds[camInd] + 1;
				if (thBufferInds[camInd] == bufferSize) {
					thStatus[camInd] = 0;
					continue;
				}
			}
			// wait some time
			if (waitTime > 0) {
				std::this_thread::sleep_for(std::chrono::milliseconds((long long)waitTime));
			}
			if (isVerbose) {
				printf("Camera %d captures one frame, wait %lld milliseconds for next frame ...\n",
					camInd, waitTime);
			}
		}
	}

	/**
	@brief multi-thread captureing function
	used for single mode
	thread function to get images from camera and buffer to vector
	@param int camInd: index of camera
	@param cv::Mat & img: output captured image
	*/
	void GenCamera::capture_thread_single_(int camInd, cv::Mat & img) {
		// capture image
		this->captureFrame(camInd, img);
	}

	/**
	@brief multi-thread capturing function (jpeg buffer)
	used for continous mode
	thread function to get images from camera and wait for compresss
	thread to compress the raw data into jpeg data
	@param int camInd: index of camera
	*/
	void GenCamera::capture_thread_JPEG_(int camInd) {
		clock_t begin_time, end_time;
		double time = 1000.0 / static_cast<double>(camInfos[camInd].fps);
		thStatus[camInd] = 1;
		for (;;) {
			begin_time = clock();
			// check status
			if (thStatus[camInd] == 0)
				break;
			while (thStatus[camInd] == 2) {
				// still in jpeg compression wait for some time
				std::this_thread::sleep_for(std::chrono::milliseconds((long long)5));
			}
			// capture image
			this->captureFrame(camInd, bufferImgs[0][camInd]);
			end_time = clock();
			float waitTime = time - static_cast<double>(end_time - begin_time) / CLOCKS_PER_SEC * 1000;
			// set status to 2, wait for compress
			thStatus[camInd] = 2;
			// wait for some time
			if (waitTime > 0) {
				std::this_thread::sleep_for(std::chrono::milliseconds((long long)waitTime));
			}
			if (isVerbose) {
				printf("Camera %d captures one frame, wait %lld milliseconds for next frame ...\n",
					camInd, waitTime);
			}	
		}
	}

	/**
	@brief single-thread compressing function
	because npp only support single thread, jpeg compress function is not
	thread safe
	thread function to compress raw image into jpeg data
	and wait until the next frame (based on fps)
	*/
	void GenCamera::compress_thread_JPEG_() {
		clock_t begin_time, end_time;
		std::vector<cudaStream_t> streams(this->cameraNum);
		for (size_t camInd = 0; camInd < this->cameraNum; camInd ++) {
			cudaStreamCreate(&streams[camInd]);
		}
		for (;;) {
			// check if all the images are captured
			for (size_t i = 0; i < this->cameraNum; i ++) {
				int sum = std::accumulate(thStatus.begin(), thStatus.end(), 0);
				if (sum != 2 * this->cameraNum) {
					std::this_thread::sleep_for(std::chrono::milliseconds((long long)5));
				}
			}
			// compress images
			for (size_t camInd = 0; camInd < this->cameraNum; camInd ++) {
				coders[camInd].encode(bufferImgs[0][camInd].data, 
					bufferJPEGImgs[thBufferInds[camInd]][camInd].data,
					&bufferJPEGImgs[thBufferInds[camInd]][camInd].length,
					streams[camInd]);
			}
			for (size_t camInd = 0; camInd < this->cameraNum; camInd ++) {	
				// synchronize and destroy threads
				cudaStreamSynchronize(streams[camInd]);
				cudaStreamDestroy(streams[camInd]);
				// set thread status to 1
				thStatus[camInd] = 1;
			}

		}
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
				// resize vector
				this->bufferJPEGImgs.resize(bufferSize);
				for (size_t i = 0; i < bufferSize; i++) {
					this->bufferJPEGImgs[i].resize(this->cameraNum);
				}
				// pre-malloc jpeg data
				for (size_t i = 0; i < this->cameraNum; i++) {
					// pre-calculate compressed jpeg data size
					size_t maxLength = static_cast<size_t>(camInfos[i].width * camInfos[i].height * 0.1f);
					for (size_t j = 0; j < bufferSize; j++) {
						this->bufferJPEGImgs[j][i].data = new uchar[maxLength];
						this->bufferJPEGImgs[j][i].maxLength = maxLength;
					}
				}
				// malloc one frame cv::Mat data
				this->bufferImgs.resize(1);
				this->bufferImgs[0].resize(this->cameraNum);
				for (size_t i = 0; i < this->cameraNum; i++) {
					this->bufferImgs[0][i].create(camInfos[i].height, camInfos[i].width, CV_8U);
				}
				// init npp jpeg coder
				this->coders.resize(this->cameraNum);
				for (size_t i = 0; i < this->cameraNum; i++) {
					coders[i].init(camInfos[i].width, camInfos[i].height, 85);
					coders[i].setCfaBayerType(static_cast<int>(camInfos[i].bayerPattern));
				}
			}
		}
		else if (captureMode == cam::GenCamCaptureMode::Single ||
			captureMode == cam::GenCamCaptureMode::SingleTrigger) {
			this->ths.resize(this->cameraNum);
		}
		return 0;
	}

	/**
	@brief set capture purpose
	@param GenCamCapturePurpose camPurpose: purpose, for streaming or recording
	@return int
	*/
	int GenCamera::setCapturePurpose(GenCamCapturePurpose camPurpose) {
		this->camPurpose = camPurpose;
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
				thStatus[i] = 0;
				thBufferInds[i] = 0;
			}
			// start capturing threads
			for (size_t i = 0; i < this->cameraNum; i++) {
				ths[i] = std::thread(&GenCamera::capture_thread_raw_, this, i);
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
			thStatus[i] = 0;
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
				int index = (thBufferInds[camInd] - 1 + bufferSize) % bufferSize;
				imgs[camInd] = bufferImgs[index][camInd];
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

