/**
@brief Generic Camera Driver Class
Used when cameras are connected to this computer 
directly
@author Shane Yuan
@date Jan 8, 2018
*/

#include <numeric>
#include "RealCameraDriver.h"
#include <opencv2/core/cuda_stream_accessor.hpp>

namespace cam {
    RealCamera::RealCamera():isCaptureThreadRunning(false),
		isCompressThreadRunning(false) {}
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
			if (thexit == 1)
				break;
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
	@param Imagedata & img: output captured image
	*/
	void RealCamera::capture_thread_single_(int camInd, Imagedata & img) {
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
		cudaStream_t stream;
		cudaStreamCreate(&stream);
		cv::cuda::Stream cvstream = cv::cuda::StreamAccessor::wrapStream(stream);
		for (;;) {
			// begin time
			begin_time = clock();
			// check status
			if (thexit == 1)
				break;
			if (thStatus[camInd] == 0)
				break;
			while (thStatus[camInd] == 2) {
				// still in jpeg compression wait for some time
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
				if (isVerbose) {
					SysUtil::warningOutput("Compress thread still not finish compress image yet !" \
						" Please set lower framerate or lower exposure time ! ");
				}
			}
			// capture image
			this->captureFrame(camInd, bufferImgs_data_ptr[camInd]);
			// copy data to GPU
			//cudaMemcpy(this->bufferImgs_cuda[camInd], bufferImgs_singleframe[camInd].data,
			//	sizeof(uchar) * camInfos[camInd].width * camInfos[camInd].height,
			//	cudaMemcpyHostToDevice);
			this->bufferImgs_host[camInd].data = reinterpret_cast<uchar*>(bufferImgs_data_ptr[camInd].data),
			this->bufferImgs_cuda[camInd].upload(this->bufferImgs_host[camInd]
				,std::ref(cvstream));
			cudaStreamSynchronize(stream);
			// end time
			end_time = clock();
			float waitTime = time - static_cast<double>(end_time - begin_time) / CLOCKS_PER_SEC * 1000;
			// set status to 2, wait for compress
			thStatus[camInd] = 2;
			// wait for some time
			if (isVerbose) {
				printf("Camera %d captures one frame, wait %lld milliseconds for next frame ...\n",
					camInd, static_cast<long long>(waitTime));
			}
			if (waitTime > 0) {
				SysUtil::sleep(waitTime);
			}
		}
		cudaStreamDestroy(stream);
		char info[256];
		sprintf(info, "Capturing thread for camera %02d finish, exit successfully !", camInd);
		SysUtil::infoOutput(info);
	}

	/**
	@brief single-thread compressing function
	because npp only support single thread, jpeg compress function is not
	thread safe
	thread function to compress raw image into jpeg data
	and wait until the next frame (based on fps)
	*/
	void RealCamera::compress_thread_JPEG_() {
		clock_t begin_time, end_time;
		cudaStream_t stream;
		cudaStreamCreate(&stream);
		bool hasFrame;
		for (;;) {
			hasFrame = false;
			// check if threads are need to exit
			int sum = std::accumulate(thBufferInds.begin(), thBufferInds.end(), 0);
			if (thexit == 1)
				break;
			if (sum == bufferSize * this->cameraNum)
				break;
			// compress images
			for (size_t camInd = 0; camInd < this->cameraNum; camInd ++) {
				// check if all the images are captured
				if (thStatus[camInd] != 2 || thBufferInds[camInd] == bufferSize)
					continue;
				else hasFrame = true;
				// begin time
				begin_time = clock();
				// compress
				coders[camInd].encode(this->bufferImgs_cuda[camInd],
					reinterpret_cast<uchar*>(bufferImgs[thBufferInds[camInd]][camInd].data),
					&bufferImgs[thBufferInds[camInd]][camInd].length,
					bufferImgs[thBufferInds[camInd]][camInd].maxLength,
					stream);
				cudaStreamSynchronize(stream);
				// end time
				end_time = clock();
				if (isVerbose) {
					float costTime = static_cast<double>(end_time - begin_time) / CLOCKS_PER_SEC * 1000;
					char info[256];
					sprintf(info, "Camera %d compress one frame, buffer to index %d, cost %f miliseconds ...", camInd, 
						thBufferInds[camInd], costTime);
					SysUtil::infoOutput(info);
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
						SysUtil::sleep(5);
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
	@brief set capturing mode
	@param GenCamCaptureMode captureMode: capture mode
	@param int size: buffer size
	@return
	*/
	int RealCamera::setCaptureMode(GenCamCaptureMode captureMode,
		int bufferSize) {
		// get camera info
		this->getCamInfos(camInfos);
		// init capture buffer
		this->captureMode = captureMode;
		this->bufferSize = bufferSize;
		if (captureMode == cam::GenCamCaptureMode::Continous ||
			captureMode == cam::GenCamCaptureMode::ContinousTrigger) {
			// create buffer for raw buffer type
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
					size_t length = width * height * sizeof(uchar);
					for (size_t j = 0; j < bufferSize; j++) {
						this->bufferImgs[j][i].data = new char[length];
						this->bufferImgs[j][i].length = length;
						this->bufferImgs[j][i].maxLength = length;
						this->bufferImgs[j][i].type = this->bufferType;
					}
				}
			}
			// create buffer for JPEG buffer type
			else if (this->bufferType == GenCamBufferType::JPEG) {
				// resize vector
				this->bufferImgs.resize(bufferSize);
				for (size_t i = 0; i < bufferSize; i++) {
					this->bufferImgs[i].resize(this->cameraNum);
				}
				// pre-malloc jpeg data
				for (size_t i = 0; i < this->cameraNum; i++) {
					// pre-calculate compressed jpeg data size
					size_t maxLength = static_cast<size_t>(camInfos[i].width * camInfos[i].height * 0.5);
					for (size_t j = 0; j < bufferSize; j++) {
						this->bufferImgs[j][i].data = new char[maxLength];
						this->bufferImgs[j][i].maxLength = maxLength;
						this->bufferImgs[j][i].type = this->bufferType;
					}
				}
				// pre-malloc cuda memory for debayer and jpeg compression
				this->bufferImgs_cuda.resize(this->cameraNum);
				this->bufferImgs_host.resize(this->cameraNum);
				this->bufferImgs_data_ptr.resize(this->cameraNum);
				for (size_t i = 0; i < this->cameraNum; i++) {
					//cudaMalloc(&this->bufferImgs_cuda[i], sizeof(uchar)
					//	* camInfos[i].width * camInfos[i].height);
					this->bufferImgs_cuda[i].create(camInfos[i].height, camInfos[i].width, CV_8U);
					size_t length = sizeof(uchar) * camInfos[i].width * camInfos[i].height;
					this->bufferImgs_data_ptr[i].data = new char[length];
					this->bufferImgs_data_ptr[i].length = length;
					this->bufferImgs_data_ptr[i].maxLength = length;
					this->bufferImgs_host[i] = cv::Mat(camInfos[i].height, camInfos[i].width, CV_8U, 
						reinterpret_cast<uchar*>(this->bufferImgs_data_ptr[i].data));
				}
				// init NPP jpeg coder	
				this->coders.resize(this->cameraNum);
				for (size_t i = 0; i < this->cameraNum; i++) {
					coders[i].init(camInfos[i].width, camInfos[i].height, JPEGQuality);
					coders[i].setCfaBayerType(static_cast<int>(camInfos[i].bayerPattern));
					coders[i].setWBRawType(camInfos[i].isWBRaw);
					coders[i].setWhiteBalanceGain(camInfos[i].redGain, camInfos[i].greenGain, camInfos[i].blueGain);
				}
			}
		}
		else if (captureMode == cam::GenCamCaptureMode::Single ||
			captureMode == cam::GenCamCaptureMode::SingleTrigger) {
			SysUtil::errorOutput("Single mode is not implemented yet !");
			exit(-1);
		}
		return 0;
	}

    /**
	@brief wait for recording threads to finish
	@return int
	*/
	int RealCamera::waitForRecordFinish() {
		// check capturing purpose 
		if (this->camPurpose != GenCamCapturePurpose::Recording) {
			SysUtil::warningOutput("This function is only valid in recording mode");
			return -1;
		}
		// check thread status
		if (this->isCaptureThreadRunning != true) {
			SysUtil::errorOutput("Capturing thread is not started !");
			return -1;
		}
		if (this->bufferType == GenCamBufferType::JPEG) {
			if (this->isCompressThreadRunning != true) {
				SysUtil::errorOutput("Compression thread is not started !");
				return -1;
			}
		}
		// wait thread to exit
		char info[256];
		for (size_t i = 0; i < this->cameraNum; i++) {
			ths[i].join();
			sprintf(info, "Capturing thread %d exit successfully !", i);
			SysUtil::infoOutput(std::string(info));
		}
		isCaptureThreadRunning = false;
		if (this->bufferType == GenCamBufferType::JPEG) {
			this->thJPEG.join();
			isCompressThreadRunning = false;
			sprintf(info, "Compression thread exit successfully !");
			SysUtil::infoOutput(std::string(info));
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
			thexit = 0;
			for (size_t i = 0; i < this->cameraNum; i++) {
				thStatus[i] = 0;
				thBufferInds[i] = 0;
			}
			// start threads based on buffer type
			if (this->bufferType == GenCamBufferType::Raw) {
				// start capturing threads
				for (size_t i = 0; i < this->cameraNum; i++) {
					ths[i] = std::thread(&RealCamera::capture_thread_raw_, this, i);
				}
				isCaptureThreadRunning = true;
			}
			else if (this->bufferType == GenCamBufferType::JPEG) {
				// start compress theads
				thJPEG = std::thread(&RealCamera::compress_thread_JPEG_, this);
				isCompressThreadRunning = true;
				// start capturing threads
				for (size_t i = 0; i < this->cameraNum; i++) {
					ths[i] = std::thread(&RealCamera::capture_thread_JPEG_, this, i);
				}
				isCaptureThreadRunning = true;
			}
			else if (this->bufferType == GenCamBufferType::RGB24 || 
				this->bufferType == GenCamBufferType::Raw16) {
				SysUtil::errorOutput("BufferType RGB and Raw16 are not support yet! ");
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
		char info[256];
		for (size_t i = 0; i < this->cameraNum; i++) {
			thStatus[i] = 0;
		}
		thexit = 1;
		// make sure all the threads have exited
		if (this->camPurpose == cam::GenCamCapturePurpose::Streaming) {
			if (isCompressThreadRunning == true) {
				thJPEG.join();
				isCompressThreadRunning = false;
				sprintf(info, "Compression thread exit successfully !");
				SysUtil::infoOutput(std::string(info));
			}
			if (isCaptureThreadRunning == true) {
				for (size_t i = 0; i < this->cameraNum; i++) {
					ths[i].join();
					sprintf(info, "Capturing thread %d exit successfully !", i);
					SysUtil::infoOutput(std::string(info));
				}
				isCaptureThreadRunning = false;
			}
		}
		// release memory
		if (this->bufferType == GenCamBufferType::JPEG) {
			for (size_t i = 0; i < this->cameraNum; i++) {
				this->bufferImgs_cuda[i].release();
				delete[] this->bufferImgs_data_ptr[i].data;
				coders[i].release();
				for (size_t j = 0; j < this->cameraNum; j++) {
					delete bufferImgs[j][i].data;
					bufferImgs[j][i].maxLength = 0;
				}
			}
		}
		SysUtil::infoOutput("Real camera driver released successfully !");
		return 0;
	}

};
