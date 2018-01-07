/**
@brief Generic Camera Driver Class
Used when cameras are connected to this computer 
directly
@author Shane Yuan
@date Jan 8, 2018
*/

#include "RealCameraDriver.h"

namespace cam {
    RealCamera::RealCamera() {}
    RealCamera::~RealCamera() {}

    /**
	@brief multi-thread capturing function
	used for continous mode
	thread function to get images from camera and buffer to vector
	and wait until the next frame (based on fps)
	@param int camInd: index of camera
	*/
	void RealCamera::capture_thread_raw_(int camInd) {
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
				std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long long>(waitTime)));
			}
			if (isVerbose) {
				printf("Camera %d captures one frame, wait %lld milliseconds for next frame ...\n",
					camInd, static_cast<long long>(waitTime));
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
	void RealCamera::capture_thread_single_(int camInd, cv::Mat & img) {
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
	void RealCamera::capture_thread_JPEG_(int camInd) {
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
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
				if (isVerbose) {
					SysUtil::warningOutput("Compress thread still not finish compress image yet !" \
						" Please set lower framerate! ");
				}
			}
			// capture image
			this->captureFrame(camInd, bufferImgs[0][camInd]);
			end_time = clock();
			float waitTime = time - static_cast<double>(end_time - begin_time) / CLOCKS_PER_SEC * 1000;
			// set status to 2, wait for compress
			thStatus[camInd] = 2;
			// wait for some time
			if (waitTime > 0) {
				std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long long>(waitTime)));
			}
			if (isVerbose) {
				printf("Camera %d captures one frame, wait %lld milliseconds for next frame ...\n",
					camInd, static_cast<long long>(waitTime));
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
	void RealCamera::compress_thread_JPEG_() {
		cudaStream_t stream;
		cudaStreamCreate(&stream);
		bool hasFrame = false;
		for (;;) {
			// check if threads are need to exit
			int sum = std::accumulate(thBufferInds.begin(), thBufferInds.end(), 0);
			if (sum == bufferSize * this->cameraNum)
				break;
			// compress images
			for (size_t camInd = 0; camInd < this->cameraNum; camInd ++) {
				// check if all the images are captured
				if (thStatus[camInd] != 2 || thBufferInds[camInd] == bufferSize)
					continue;
				else hasFrame = true;
				// copy data to GPU
				cudaMemcpy(this->bufferImgs_cuda[camInd], bufferImgs[0][camInd].data, 
					sizeof(uchar) * camInfos[camInd].width * camInfos[camInd].height,
					cudaMemcpyHostToDevice);
				// compress
				coders[camInd].encode(this->bufferImgs_cuda[camInd],
					bufferImgs[thBufferInds[camInd]][camInd].data,
					&bufferImgs[thBufferInds[camInd]][camInd].length,
					bufferImgs[thBufferInds[camInd]][camInd].maxLength,
					stream);
				cudaStreamSynchronize(stream);
				if (isVerbose) {
					printf("Camera %d compress one frame, buffer to index %d ...\n",
						camInd, thBufferInds[camInd]);
				}
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
				// set thread status to 1
				thStatus[camInd] = 1;
				// if no image is compressed in this for loop, wait 5ms
				if (camInd == this->cameraNum - 1) {
					if (hasFrame == false) {
						std::this_thread::sleep_for(std::chrono::milliseconds((long long)5));
					}
					else
						hasFrame = true;
				}
			}
		}
		cudaStreamDestroy(stream);
		SysUtil::infoOutput("JPEG compress thread exit successfully !");
	}

    /**
    @brief init npp jpeg coder
    @return int
    */
    int RealCamera::initNPPJpegCoder() {
        this->coders.resize(this->cameraNum);
        for (size_t i = 0; i < this->cameraNum; i++) {
            coders[i].init(camInfos[i].width, camInfos[i].height, JPEGQuality);
            coders[i].setCfaBayerType(static_cast<int>(camInfos[i].bayerPattern));
            coders[i].setWBRawType(camInfos[i].isWBRaw);
            coders[i].setWhiteBalanceGain(camInfos[i].redGain, camInfos[i].greenGain, camInfos[i].blueGain);
        }
        return 0;
    }

    /**
	@brief wait for recording threads to finish
	@return int
	*/
	int RealCamera::waitForRecordFinish() {
		if (this->camPurpose != GenCamCapturePurpose::Recording) {
			SysUtil::warningOutput("This function is only valid in recording mode");
			return -1;
		}
		if (this->bufferType == GenCamBufferType::Raw) {
			for (size_t i = 0; i < this->cameraNum; i++) {
				ths[i].join();
			}
		}
		else if (this->bufferType == GenCamBufferType::JPEG) {
			for (size_t i = 0; i < this->cameraNum; i++) {
				ths[i].join();
			}
			this->thJPEG.join();
		}
		return 0;
	}

	/**
	@brief start capture threads
	@return int
	*/
	int RealCamera::startCaptureThreads() {
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
			// start threads based on buffer type
			if (this->bufferType == GenCamBufferType::Raw) {
				// start capturing threads
				for (size_t i = 0; i < this->cameraNum; i++) {
					ths[i] = std::thread(&GenCamera::capture_thread_raw_, this, i);
				}
			}
			else if (this->bufferType == GenCamBufferType::JPEG) {
				// start compress theads
				thJPEG = std::thread(&GenCamera::compress_thread_JPEG_, this);
				// start capturing threads
				for (size_t i = 0; i < this->cameraNum; i++) {
					ths[i] = std::thread(&GenCamera::capture_thread_JPEG_, this, i);
				}
			}
			else if (this->bufferType == GenCamBufferType::RGB) {
				SysUtil::errorOutput("BufferType RGB is not support yet! ");
				exit(-1);
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
	int RealCamera::stopCaptureThreads() {
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
		// release memory
		if (this->bufferType == GenCamBufferType::JPEG) {
			for (size_t i = 0; i < this->cameraNum; i++) {
				cudaFree(this->bufferImgs_cuda[i]);
				for (size_t j = 0; j < this->cameraNum; j++) {
					delete bufferImgs[j][i].data;
					bufferImgs[j][i].maxLength = 0;
				}
			}
		}
		return 0;
	}

};
