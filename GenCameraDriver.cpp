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
				this->bufferImgs.resize(bufferSize);
				for (size_t i = 0; i < bufferSize; i++) {
					this->bufferImgs[i].resize(this->cameraNum);
				}
				// pre-malloc jpeg data
				for (size_t i = 0; i < this->cameraNum; i++) {
					// pre-calculate compressed jpeg data size
					size_t maxLength = static_cast<size_t>(camInfos[i].width * camInfos[i].height * sizeRatio);
					for (size_t j = 0; j < bufferSize; j++) {
						this->bufferImgs[j][i].data = new uchar[maxLength];
						this->bufferImgs[j][i].maxLength = maxLength;
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
				// this->coders.resize(this->cameraNum);
				// for (size_t i = 0; i < this->cameraNum; i++) {
				// 	coders[i].init(camInfos[i].width, camInfos[i].height, JPEGQuality);
				// 	coders[i].setCfaBayerType(static_cast<int>(camInfos[i].bayerPattern));
				// 	coders[i].setWBRawType(camInfos[i].isWBRaw);
				// 	coders[i].setWhiteBalanceGain(camInfos[i].redGain, camInfos[i].greenGain, camInfos[i].blueGain);
				// }
				this->initNPPJpegCoder();
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
					outputFile.write(reinterpret_cast<const char*>(this->bufferImgs[j][i].data),
						this->bufferImgs[j][i].length);
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
					coders[i].decode(this->bufferImgs[j][i].data, 
						this->bufferImgs[j][i].length,
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

