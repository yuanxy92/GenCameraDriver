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
		isCompressThreadRunning(false), isCaptureModeSet(false), isCapturedFrameGpuPointer(false), isCapturedFrameDebayered(false){}
    RealCamera::~RealCamera() {}

	/**
	@brief get camera model string
	@return std::string
	*/
	std::string RealCamera::getCamModelString()
	{
		// XIMEA_xiC = 0,
		// PointGrey_u3 = 1,
		// Network = 2,
		// File = 3,
		// Stereo = 4
		if(this->camModel == CameraModel::XIMEA_xiC)
			return "   XIMEA_xiC";
		else if(this->camModel == CameraModel::PointGrey_u3)
			return "PointGrey_u3";
		else if(this->camModel == CameraModel::Network)
			return "     Network";
		else if(this->camModel == CameraModel::File)
			return "        File";
		else if(this->camModel == CameraModel::Stereo)
			return "      Stereo";
		else
			return "   Undefined";
	}

    /**
	@brief multi-thread capturing function
	used for continous mode
	thread function to get images from camera and buffer to vector
	and wait until the next frame (based on fps)
	@param int camInd: index of camera
	*/
	void RealCamera::capture_thread_raw_(int camInd) {
		if (this->camModel == cam::CameraModel::Stereo)
		{
			SysUtil::errorOutput("RealCamera::capture_thread_raw_ raw image is not supported for Stereo Camera!");
			return;
		}

		clock_t begin_time, end_time;
		clock_t stat_last_time = SysUtil::getCurrentTimeMicroSecond();
		int stat_frame_count = 0;
		double time = 1000.0 / static_cast<double>(camInfos[camInd].fps);
		thStatus[camInd] = 1;
		for (;;) {
			begin_time = SysUtil::getCurrentTimeMicroSecond();
			// check status
			if (thexit == 1)
				break;
			if (thStatus[camInd] == 0)
				break;
			// capture image
			this->captureFrame(camInd, bufferImgs[thBufferInds[camInd]][camInd]);
			stat_frame_count++;
			end_time = SysUtil::getCurrentTimeMicroSecond();
			float waitTime = time - static_cast<double>(end_time - begin_time) / 1000;
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
			float stat_pass_time = static_cast<double>(end_time - stat_last_time) / 1000;
			if(stat_pass_time > STAT_FPS_OUTPUT_MS && STAT_FPS_OUTPUT_MS > 0)
			{
				float stat_fps = (float)stat_frame_count / stat_pass_time * 1000.0f;
				SysUtil::infoOutput(cv::format("[Capture Raw FPS] CamModel %s , CamInd %d, fps = %f", this->getCamModelString().c_str(), camInd, stat_fps));
				stat_frame_count = 0;
				stat_last_time = end_time;
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
		clock_t stat_last_time = SysUtil::getCurrentTimeMicroSecond();
		int stat_frame_count = 0;
		double time = 1000.0 / static_cast<double>(camInfos[camInd].fps);
		thStatus[camInd] = 1;
		cv::cuda::Stream cvstream; 
		for (;;) {
			// begin time
			begin_time = SysUtil::getCurrentTimeMicroSecond();
			// check status
			if (thexit == 1)
				break;
			if (thStatus[camInd] == 0)
				break;
			while (thStatus[camInd] == 2) {
				if (thexit == 1)
					break;
				// still in jpeg compression wait for some time
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
				if (isVerbose) {
					SysUtil::warningOutput("Compress thread still not finish compress image yet !" \
						" Please set lower framerate or lower exposure time ! ");
				}
			}
			// capture image
			this->captureFrame(camInd, bufferImgs_data_ptr[camInd]);
			stat_frame_count++;
			// copy data to GPU
			//cudaMemcpy(this->bufferImgs_cuda[camInd], bufferImgs_singleframe[camInd].data,
			//	sizeof(uchar) * camInfos[camInd].width * camInfos[camInd].height,
			//	cudaMemcpyHostToDevice);

			if (this->isCapturedFrameGpuPointer == false)
			{
				this->bufferImgs_host[camInd].data = reinterpret_cast<uchar*>(bufferImgs_data_ptr[camInd].data);
				this->bufferImgs_cuda[camInd].upload(this->bufferImgs_host[camInd]
					, std::ref(cvstream));
				cvstream.waitForCompletion();
			}
			else if(this->isCapturedFrameDebayered == false)
			{
				this->bufferImgs_cuda[camInd].data = reinterpret_cast<uchar*>(bufferImgs_data_ptr[camInd].data);
			}
			else if(bufferImgs_data_ptr[camInd].isJpegCompressd == false)
			{
				this->dabayerImgs_cuda[camInd].data = reinterpret_cast<uchar*>(bufferImgs_data_ptr[camInd].data);
			}



			// end time
			end_time = SysUtil::getCurrentTimeMicroSecond();
			float waitTime = time - static_cast<double>(end_time - begin_time) / 1000;
			// set status to 2, wait for compress
			if (thexit == 1)
				break;
			thStatus[camInd] = 2;
			// wait for some time
			if (isVerbose) {
				printf("Camera %d captures one frame, wait %lld milliseconds for next frame ...\n",
					camInd, static_cast<long long>(waitTime));
			}
			float stat_pass_time = static_cast<double>(end_time - stat_last_time) / 1000;
			//SysUtil::infoOutput(cv::format("stat_pass_time = %f", stat_pass_time));
			if(stat_pass_time > STAT_FPS_OUTPUT_MS && STAT_FPS_OUTPUT_MS > 0)
			{
				float stat_fps = (float)stat_frame_count / stat_pass_time * 1000.0f;
				SysUtil::infoOutput(cv::format("[Capture JpegFPS] CamModel %s , CamInd %d, fps = %f", this->getCamModelString().c_str(), camInd, stat_fps));
				stat_frame_count = 0;
				stat_last_time = end_time;
			}
			//SysUtil::infoOutput(cv::format("wait time = %d", static_cast<int>(waitTime)));
			if (waitTime > 0) {
				//SysUtil::infoOutput(cv::format("wait time = %d", static_cast<int>(waitTime)));
				SysUtil::sleep(static_cast<int>(waitTime));
			}
		}
		char info[256];
		sprintf(info, "Capturing thread for caemera %02d finish, exit successfully !", camInd);
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
		clock_t stat_last_time = SysUtil::getCurrentTimeMicroSecond();
		int stat_frame_count = 0;
		cv::cuda::Stream stream;
		bool hasFrame;
		for (;;) {
			hasFrame = false;
			// check if threads are need to exit
			int sum = std::accumulate(thBufferInds.begin(), thBufferInds.end(), 0);

			//SysUtil::infoOutput(cv::format("[OUTSIDE]thBufferInds[0] : %d, sum : %d", thBufferInds[0], sum));

			if (thexit == 1)
				break;
			if (sum == bufferSize * this->cameraNum)
				break;
			// compress images
			for (size_t camInd = 0; camInd < this->cameraNum; camInd++) {
				// check if all the images are captured
				if (thStatus[camInd] != 2 || thBufferInds[camInd] == bufferSize) {
					// if no image is compressed in this for loop, wait 5ms
					if (camInd == this->cameraNum - 1) {
						if (hasFrame == false) {
							SysUtil::sleep(10);
						}
						else {
							hasFrame = false;
						}
					}
					continue;
				}
				else hasFrame = true;

				//SysUtil::infoOutput(cv::format("[INSIDE] thBufferInds[0] : %d, sum : %d", thBufferInds[0], sum));

				if (thexit == 1)
					break;
				// begin time
				begin_time = SysUtil::getCurrentTimeMicroSecond();



				//TODO (SHADOWK) : added ratio control code here (also more coder is needed)
				int ratioInd = static_cast<int>(imgRatios[camInd]);
				// debayer
				if (this->isCapturedFrameDebayered == false)
				{
					cv::cuda::demosaicing(this->bufferImgs_cuda[camInd], this->dabayerImgs_cuda[camInd],
						npp::bayerPatternNPP2CVRGB(static_cast<NppiBayerGridPosition>(
							static_cast<int>(camInfos[camInd].bayerPattern))), -1, stream);
				}
				//else
				//{
				//	this->dabayerImgs_cuda[camInd] = this->bufferImgs_cuda[camInd];
				//}
				// resize
				if (ratioInd != 0 && bufferImgs_data_ptr[camInd].isJpegCompressd == false) {
					cv::cuda::resize(this->dabayerImgs_cuda[camInd], this->resizedDebayerImgs_cuda[camInd][ratioInd],//this->dabayerImgs_cuda[camInd],
						coders[camInd][ratioInd].getImageSize(), cv::INTER_LINEAR);
				}
				// compress

				//cv::Mat tmp;
				//dabayerImgs_cuda[camInd].download(tmp);

				if (bufferImgs_data_ptr[camInd].isJpegCompressd == false)
				{
					coders[camInd][ratioInd].encode_rgb(
						((ratioInd == 0) ? this->dabayerImgs_cuda[camInd] : this->resizedDebayerImgs_cuda[camInd][ratioInd]),
						reinterpret_cast<uchar*>(bufferImgs[thBufferInds[camInd]][camInd].data),
						&bufferImgs[thBufferInds[camInd]][camInd].length,
						bufferImgs[thBufferInds[camInd]][camInd].maxLength,
						stream);
					bufferImgs[thBufferInds[camInd]][camInd].ratio = static_cast<cam::GenCamImgRatio>(ratioInd);
					//cudaStreamSynchronize(stream);
					stream.waitForCompletion();
				}
				else
				{
					bufferImgs[thBufferInds[camInd]][camInd].length = bufferImgs_data_ptr[camInd].length;
					memcpy(bufferImgs[thBufferInds[camInd]][camInd].data, bufferImgs_data_ptr[camInd].data, bufferImgs_data_ptr[camInd].length);
					bufferImgs[thBufferInds[camInd]][camInd].ratio = cam::GenCamImgRatio::Full;
				}




				stat_frame_count++;
				// end time
				end_time = SysUtil::getCurrentTimeMicroSecond();
				if (isVerbose) {
					float costTime = static_cast<double>(end_time - begin_time) / 1000;
					char info[256];
					sprintf(info, "Camera %d compress one frame, buffer to index %d, cost %f miliseconds ...", camInd,
						thBufferInds[camInd], costTime);
					SysUtil::infoOutput(info);
				}
				float stat_pass_time = static_cast<double>(end_time - stat_last_time) / 1000;
				if(stat_pass_time > STAT_FPS_OUTPUT_MS && STAT_FPS_OUTPUT_MS > 0)
				{
					float stat_fps = (float)stat_frame_count / stat_pass_time * 1000.0f;
					SysUtil::infoOutput(cv::format("[CompressJpegFPS] CamModel %s , fps = %f (With total %d cameras)", this->getCamModelString().c_str(), stat_fps, this->cameraNum));
					stat_frame_count = 0;
					stat_last_time = end_time;
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
				if (thexit == 1)
					break;
				thStatus[camInd] = 1;
			}
		}
		thexit = 1; //hack there is a problem 
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
		if (isCaptureModeSet == true) {
			SysUtil::warningOutput("Capture mode is already set! Please do not set twice!");
			return 0;
		}
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
					size_t maxLength;
					if (this->camModel == cam::CameraModel::Stereo && i >= this->cameraNum / 2)
						maxLength = static_cast<size_t>(camInfos[i].width * camInfos[i].height * 2);
					else
						maxLength = static_cast<size_t>(camInfos[i].width * camInfos[i].height * this->sizeRatio);
					for (size_t j = 0; j < bufferSize; j++) {
						this->bufferImgs[j][i].data = new char[maxLength];
						this->bufferImgs[j][i].maxLength = maxLength;
						this->bufferImgs[j][i].type = this->bufferType;
					}
				}
				// pre-malloc cuda memory for debayer and jpeg compression
				this->bufferImgs_cuda.resize(this->cameraNum);
				this->dabayerImgs_cuda.resize(this->cameraNum);
				this->bufferImgs_host.resize(this->cameraNum);
				this->bufferImgs_data_ptr.resize(this->cameraNum);
				for (size_t i = 0; i < this->cameraNum; i++) {
					//cudaMalloc(&this->bufferImgs_cuda[i], sizeof(uchar)
					//	* camInfos[i].width * camInfos[i].height);
					this->bufferImgs_cuda[i].create(camInfos[i].height, camInfos[i].width, CV_8U);
					this->dabayerImgs_cuda[i].create(camInfos[i].height, camInfos[i].width, CV_8UC3);
					size_t length = sizeof(uchar) * camInfos[i].width * camInfos[i].height;
					this->bufferImgs_data_ptr[i].data = new char[length];
					this->bufferImgs_data_ptr[i].length = length;
					this->bufferImgs_data_ptr[i].maxLength = length;
					this->bufferImgs_host[i] = cv::Mat(camInfos[i].height, camInfos[i].width, CV_8U, 
						reinterpret_cast<uchar*>(this->bufferImgs_data_ptr[i].data));
				}
				// init NPP jpeg coder	
				this->coders.resize(this->cameraNum);
				this->resizedDebayerImgs_cuda.resize(this->cameraNum);
				for (size_t i = 0; i < this->cameraNum; i++) {
					this->coders[i].resize(4);
					this->resizedDebayerImgs_cuda[i].resize(4);
					for (size_t j = 0; j < 4; j++) {
						cv::Size size = cam::GenCamera::makeDoubleSize(cv::Size(camInfos[i].width, camInfos[i].height),
							static_cast<cam::GenCamImgRatio>(j));
						coders[i][j].init(size.width, size.height, JPEGQuality);
						coders[i][j].setCfaBayerType(static_cast<int>(camInfos[i].bayerPattern));
						coders[i][j].setWBRawType(camInfos[i].isWBRaw);
						coders[i][j].setWhiteBalanceGain(camInfos[i].redGain, camInfos[i].greenGain, camInfos[i].blueGain);

						this->resizedDebayerImgs_cuda[i][j].create(size.height, size.width, CV_8UC3);
					}
				}
			}
		}
		else if (captureMode == cam::GenCamCaptureMode::Single ||
			captureMode == cam::GenCamCaptureMode::SingleTrigger) {
			SysUtil::errorOutput("Single mode is not implemented yet !");
			exit(-1);
		}
		this->isCaptureModeSet = true;
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
		if (isCaptureThreadRunning == true) {
			SysUtil::warningOutput("Capturing/compression thread is already running! Please do not start twice!");
			return 0;
		}
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
		SysUtil::sleep(200);
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
					sprintf(info, "Try to stop capturing thread %d ...", i);
					SysUtil::infoOutput(std::string(info));
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
				//delete[] this->bufferImgs_data_ptr[i].data;
				delete[] this->bufferImgs_host[i].data;
				for (size_t j = 0; j < 4; j++) {
					coders[i][j].release();
				}
				for (size_t j = 0; j < this->cameraNum; j++) {
					delete[] bufferImgs[j][i].data;
					bufferImgs[j][i].maxLength = 0;
				}
			}
		}
		this->isCaptureModeSet = false;
		SysUtil::infoOutput("Real camera driver released successfully !");
		return 0;
	}

	/*************************************************************/
	/*            function to update images in buffer            */
	/*************************************************************/
	/**
	@brief buffer next frame
	@return int
	*/
	int RealCamera::bufferNextFrame() {
		SysUtil::warningOutput("Function bufferNextFrame only valid for"\
			"FileCamera with capturing purpose FileCameraRecording.");
		return 0;
	}

	/**
	@brief buffer next frame
	@return int
	*/
	int RealCamera::reBufferFileCamera() {
		SysUtil::warningOutput("Function reBufferFileCamera only valid for"\
			"FileCamera with capturing purpose FileCameraRecording.");
		return 0;
	}

};
