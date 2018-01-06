/**
@brief Generic Camera Driver Class
@author Shane Yuan
@date Dec 29, 2017
*/

#include "GenCameraDriver.h"
#include "XIMEA/XIMEACamera.h"
#include "PointGrey/PointGreyCamera.h"
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
			std::shared_ptr<GenCameraPTGREY> cameraPtr = std::make_shared<GenCameraPTGREY>();
			return std::static_pointer_cast<GenCamera>(cameraPtr);
		}
	}

	GenCamera::GenCamera() : isInit(false), isCapture(false),
		isVerbose(false), bufferType(GenCamBufferType::Raw),
		camPurpose(GenCamCapturePurpose::Streaming),
		JPEGQuality(75), sizeRatio(0.12) {}
	GenCamera::~GenCamera() {}

	/**
	@brief get camera model
	@return
	*/
	CameraModel GenCamera::getCamModel() {
		return camModel;
	}

	/**
	@brief set verbose
	@param bool isVerbose: true, verbose mode, output many infomations
	for debugging
	@return int
	*/
	int GenCamera::setVerbose(bool isVerbose) {
		this->isVerbose = isVerbose;
		return 0;
	}

	/**
	@brief set buffer type
	@param GenCamBufferType type: buffer type
	@return int
	*/
	int GenCamera::setCamBufferType(GenCamBufferType type) {
		this->bufferType = type;
		return 0;
	}

	/**
	@brief set jpeg compression quality
	@param int quality: JPEG compression quality (1 - 100)
	@param float sizeRatio: expected compression ratio used for
	pre-malloc memory
	@return int
	*/
	int GenCamera::setJPEGQuality(int quality, float sizeRatio) {
		this->JPEGQuality = quality;
		this->sizeRatio = sizeRatio;
		return 0;
	}

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
	void GenCamera::compress_thread_JPEG_() {
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
					bufferJPEGImgs[thBufferInds[camInd]][camInd].data,
					&bufferJPEGImgs[thBufferInds[camInd]][camInd].length,
					bufferJPEGImgs[thBufferInds[camInd]][camInd].maxLength,
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
					size_t maxLength = static_cast<size_t>(camInfos[i].width * camInfos[i].height * sizeRatio);
					for (size_t j = 0; j < bufferSize; j++) {
						this->bufferJPEGImgs[j][i].data = new uchar[maxLength];
						this->bufferJPEGImgs[j][i].maxLength = maxLength;
					}
				}
				// pre-malloc cuda memory for debayer and jpeg compression
				this->bufferImgs_cuda.resize(this->cameraNum);
				for (size_t i = 0; i < this->cameraNum; i++) {
					cudaMalloc(&this->bufferImgs_cuda[i], sizeof(uchar)
						* camInfos[i].width * camInfos[i].height);
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
					coders[i].init(camInfos[i].width, camInfos[i].height, JPEGQuality);
					coders[i].setCfaBayerType(static_cast<int>(camInfos[i].bayerPattern));
					coders[i].setWBRawType(camInfos[i].isWBRaw);
					coders[i].setWhiteBalanceGain(camInfos[i].redGain, camInfos[i].greenGain, camInfos[i].blueGain);
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
	@brief wait for recording threads to finish
	@return int
	*/
	int GenCamera::waitForRecordFinish() {
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
	int GenCamera::stopCaptureThreads() {
		// set th status to false
		for (size_t i = 0; i < this->cameraNum; i++) {
			thStatus[i] = 0;
		}
		// make sure all the threads are terminated
		for (size_t i = 0; i < this->cameraNum; i++) {
			if (ths[i].joinable()) {
				ths[i].join();
				char info[256];
				sprintf(info, "Capturing thread %d terminate correctly !", i);
				SysUtil::infoOutput(std::string(info));
			}
		}
		// release memory
		if (this->bufferType == GenCamBufferType::JPEG) {
			for (size_t i = 0; i < this->cameraNum; i++) {
				cudaFree(this->bufferImgs_cuda[i]);
				for (size_t j = 0; j < this->cameraNum; j++) {
					delete bufferJPEGImgs[j][i].data;
					bufferJPEGImgs[j][i].maxLength = 0;
				}
			}
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

	/*************************************************************/
	/*        function to save capture images to files           */
	/*************************************************************/
	/**
	@brief save captured images to dir
	@param std::string dir: input dir to save images
	@return int
	*/
	int GenCamera::saveImages(std::string dir) {
		if (this->bufferType == GenCamBufferType::JPEG) {
			SysUtil::mkdir(dir);
			for (size_t i = 0; i < this->cameraNum; i++) {
				for (size_t j = 0; j < this->bufferSize; j++) {
					char outname[256];
					sprintf(outname, "%s/%02d_%05d.jpg", dir.c_str(), i, j);
					std::ofstream outputFile(outname, std::ios::out | std::ios::binary);
					outputFile.write(reinterpret_cast<const char*>(this->bufferJPEGImgs[j][i].data),
						this->bufferJPEGImgs[j][i].length);
				}
			}
		}
		else {
			SysUtil::errorOutput("Sorry, save function for other buffer types is not support yet. ");
			exit(-1);
		}
		return 0;
	}

	/**
	@brief save captured videos to dir
	@param std::string dir: input dir to save videos
	@return int
	*/
	int GenCamera::saveVideos(std::string dir) {
		if (this->bufferType == GenCamBufferType::JPEG) {
			SysUtil::mkdir(dir);
			for (size_t i = 0; i < this->cameraNum; i++) {
				std::string videoname = cv::format("%s/cam_%02d.avi", dir.c_str(), i);
				cv::VideoWriter writer(videoname, cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 
					camInfos[i].fps, cv::Size(camInfos[i].width, camInfos[i].height), true);
				cv::cuda::GpuMat img_d(camInfos[i].height, camInfos[i].width, CV_8UC3);
				cv::Mat img(camInfos[i].height, camInfos[i].width, CV_8UC3);
				for (size_t j = 0; j < this->bufferSize; j++) {
					coders[i].decode(this->bufferJPEGImgs[j][i].data, 
						this->bufferJPEGImgs[j][i].length,
						img_d);
					img_d.download(img);
					writer << img;
				}
				writer.release();
			}
		}
		else {
			SysUtil::errorOutput("Sorry, save function for other buffer types is not support yet. ");
			exit(-1);
		}
		return 0;
	}

	/*************************************************************/
	/*    function to set mapping vector of capture function     */
	/*************************************************************/
	/**
	@brief set mapping vector of capture function
	@param std::vector<size_t> mappingVector: input mapping vector
	@return int
	*/
	int GenCamera::setMappingVector(std::vector<size_t> mappingVector) {
		this->mappingVector = mappingVector;
		return 0;
	}

	/**
	@brief capture one frame with Mapping
	@param std::vector<cv::Mat> & imgs: output captured images
	if in single mode, memory of image mats should be malloced
	before using this function
	@return int
	*/
	int GenCamera::captureFrameWithMapping(std::vector<cv::Mat> & imgs) {
		size_t camInd;
		if (captureMode == GenCamCaptureMode::Continous ||
			captureMode == GenCamCaptureMode::ContinousTrigger) {
			// get images from buffer
			for (size_t i = 0; i < this->cameraNum; i++) {
				camInd = mappingVector[i];
				int index = (thBufferInds[camInd] - 1 + bufferSize) % bufferSize;
				imgs[i] = bufferImgs[index][camInd];
			}

		}
		else if (captureMode == GenCamCaptureMode::Single ||
			captureMode == GenCamCaptureMode::SingleTrigger) {
			// get images from camera
			for (size_t i = 0; i < this->cameraNum; i++) {
				camInd = mappingVector[i];
				ths[i] = std::thread(&GenCamera::capture_thread_single_, this, camInd, std::ref(imgs[i]));
			}
			// wait for all the threads to exit
			for (size_t i = 0; i < this->cameraNum; i++) {
				ths[i].join();
			}
		}
		return 0;
	}
}

