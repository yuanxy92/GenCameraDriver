/**
@brief Generic Camera Driver Class
Implementation of file camera
@author Shane Yuan
@date Mar 29, 2018
*/

#include "FileCamera.h"

namespace cam {
	// constructor
	GenCameraFile::GenCameraFile() {}
	GenCameraFile::GenCameraFile(std::string dir) {
		this->camModel = cam::CameraModel::File;
		this->dir = dir;
	}
	GenCameraFile::~GenCameraFile() {}

	/*************************************************************/
	/*                     private function                      */
	/*************************************************************/
	/**
	@brief convert rgb image to bayer image
	@param cv::Mat img: input image CV_8UC3 Color_BGR
	@return cv::Mat out: output bayer image
	*/
	cv::Mat GenCameraFile::colorBGR2BayerRG(cv::Mat img) {
		cv::Mat out(img.rows, img.cols, CV_8U);
		size_t rows = img.rows / 2;
		size_t cols = img.cols / 2;
		for (size_t row = 0; row < rows; row++) {
			for (size_t col = 0; col < cols; col++) {
				out.at<uchar>(row * 2, col * 2) = img.at<cv::Vec3b>(row * 2, col * 2).val[1];
				out.at<uchar>(row * 2, col * 2 + 1) = img.at<cv::Vec3b>(row * 2, col * 2 + 1).val[2];
				out.at<uchar>(row * 2 + 1, col * 2) = img.at<cv::Vec3b>(row * 2 + 1, col * 2).val[0];
				out.at<uchar>(row * 2 + 1, col * 2 + 1) = img.at<cv::Vec3b>(row * 2 + 1, col * 2 + 1).val[1];
			}
		}
		return out;
	}

	/**
	@brief load file information from config file
	@return int
	*/
	int GenCameraFile::loadConfigFile() {
		std::string configname = cv::format("%s/camera_setup.yml", dir.c_str());
		cv::FileStorage fs(configname, cv::FileStorage::READ);
		// read basic information
		int camNum;
		fs["CamNum"] >> camNum;
		this->cameraNum = camNum;
		fs["BufferScale"] >> this->bufferScale;
		filenames.resize(this->cameraNum);
		// read detail information of file cameras
		cv::FileNodeIterator it;
		cv::FileNode node = fs["CamParams"];
		size_t ind = 0;
		for (it = node.begin(); it != node.end(); ++it) {
			// read serialnum
			(*it)["serialnum"] >> filenames[ind];
			ind++;
		}
		fs.release();
		// get camera infos
		camInfos.resize(this->cameraNum);
		this->getCamInfos(camInfos);
		return 0;
	}

	/**
	@brief buffer image data
	@return int
	*/
	int GenCameraFile::bufferImageData() {
		this->bufferImgs.resize(this->bufferSize);
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
		// read images from videos
		for (size_t i = 0; i < this->cameraNum; i++) {
			SysUtil::infoOutput("Buffer video " + filenames[i]);
			char videoname[1024];
			sprintf(videoname, "%s/%s", this->dir.c_str(), filenames[i].c_str());
			cv::VideoCapture reader(videoname);
			cv::Mat img, smallImg, bayerImg;
			for (size_t j = 0; j < bufferSize; j++) {
				reader >> img;
				cv::resize(img, smallImg, cv::Size(camInfos[i].width, camInfos[i].height));
				bayerImg = colorBGR2BayerRG(smallImg);
				this->bufferImgs[j][i].length = sizeof(uchar) * bayerImg.rows * bayerImg.cols;
				memcpy(this->bufferImgs[j][i].data, bayerImg.data, 
					this->bufferImgs[j][i].length);
			}
			reader.release();
		}
		return 0;
	}

	/*************************************************************/
	/*                   basic camera function                   */
	/*************************************************************/
	/**
	@brief init function
	@return int
	*/
	int GenCameraFile::init() {
		// read config file in dir
		this->loadConfigFile();
		// set sign bit
		this->isInit = true;
		return 0;
	}

	/**
	@brief release function
	@return int
	*/
	int GenCameraFile::release() {
		if (isInit) {
			for (size_t i = 0; i < this->cameraNum; i++) {
				for (size_t j = 0; j < bufferSize; j++) {
					delete[] this->bufferImgs[j][i].data;
				}
			}
		}
		return 0;
	}

	/**
	@brief get camera information
	@param std::vector<GenCamInfo> & camInfos: output camera infos
	@return int
	*/
	int GenCameraFile::getCamInfos(std::vector<GenCamInfo>& camInfos) {
		frameCounts.resize(this->cameraNum);
		camInfos.resize(this->cameraNum);
		for (size_t i = 0; i < this->cameraNum; i++) {
			camInfos[i].sn = filenames[i];
			camInfos[i].redGain = 1;
			camInfos[i].greenGain = 1;
			camInfos[i].blueGain = 1;
			camInfos[i].isWBRaw = 1;
			camInfos[i].autoExposure = cam::Status::off;
			camInfos[i].bayerPattern = GenCamBayerPattern::BayerGRBG;
			// read width and height from video file
			char videoname[1024];
			sprintf(videoname, "%s/%s", this->dir.c_str(), filenames[i].c_str());
			cv::VideoCapture reader(videoname);
			camInfos[i].fps = reader.get(CV_CAP_PROP_FPS);
			camInfos[i].width = reader.get(CV_CAP_PROP_FRAME_WIDTH) 
				* this->bufferScale;
			camInfos[i].height = reader.get(CV_CAP_PROP_FRAME_HEIGHT)
				* this->bufferScale;
			frameCounts[i] = reader.get(CV_CAP_PROP_FRAME_COUNT);
			reader.release();
		}
		return 0;
	}

	/*************************************************************/
	/*                  camera setting function                  */
	/*************************************************************/
	/**
	@brief set/get bayer pattern
	@param int camInd: input camera index
	@param GenCamBayerPattern & bayerPattern: output bayer pattern
	@return int
	*/
	int GenCameraFile::getBayerPattern(int camInd, GenCamBayerPattern & bayerPattern) {
		bayerPattern = camInfos[camInd].bayerPattern;
		return 0;
	}

	/*************************************************************/
	/*                     capturing function                    */
	/*************************************************************/
	/**
	@brief set capturing mode
	@param GenCamCaptureMode captureMode: capture mode
	@param int size: buffer size
	@return int
	*/
	int GenCameraFile::setCaptureMode(GenCamCaptureMode captureMode, int bufferSize) {
		if (bufferType != GenCamBufferType::Raw) {
			SysUtil::errorOutput("File camera only support raw type buffer ! ");
			exit(-1);
		}
		if (captureMode != GenCamCaptureMode::Continous) {
			SysUtil::errorOutput("File camera only support continous capture mode ! "\
				"Capture mode set to continous");
		}
		this->captureMode = GenCamCaptureMode::Continous;
		int finalBufferSize = bufferSize;
		for (size_t i = 0; i < this->cameraNum; i++) {
			finalBufferSize = std::min<int>(finalBufferSize, frameCounts[i]);
		}
		if (finalBufferSize != bufferSize) {
			SysUtil::warningOutput(std::string("Input bufferSize " + bufferSize) + " is too small."
				+ std::string("change to " + finalBufferSize) + " !");
		}
		this->bufferSize = finalBufferSize;
		// buffer image data
		this->bufferImageData();
		this->isCapture = true;
		// init frame indices buffer
		this->thBufferInds.resize(this->cameraNum);
		for (size_t i = 0; i < this->cameraNum; i++) {
			thBufferInds[i] = 1;
		}
		return 0;
	}
};