/**
@brief Generic Camera Driver Class
Implementation of file camera
@author Shane Yuan
@date Mar 29, 2018
*/
#ifdef _WIN32
#include <io.h> 
#define access    _access_s
#else
#include <unistd.h>
#endif
#include "FileCamera.h"
#include <opencv2\bgsegm.hpp>

namespace cam {

	inline bool isFileExists(const std::string &Filename) {
		return access(Filename.c_str(), 0) == 0;
	}

	// constructor
	GenCameraFile::GenCameraFile() : hasSyncFile(false) {}
	GenCameraFile::GenCameraFile(std::string dir) : hasSyncFile(false) {
		this->camModel = cam::CameraModel::File;
		this->camPurpose = cam::GenCamCapturePurpose::Streaming;
		this->dir = dir;
		this->startFrameInd = 0;
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
		frameshifts.resize(this->cameraNum);
		// read detail information of file cameras
		cv::FileNodeIterator it;
		cv::FileNode node = fs["CamParams"];
		size_t ind = 0;
		for (it = node.begin(); it != node.end(); ++it) {
			// read serialnum
			(*it)["serialnum"] >> filenames[ind];
			(*it)["frameshift"] >> frameshifts[ind];
			ind++;
		}
		fs["StartFrameInd"] >> this->startFrameInd;
		fs.release();
		// get real file name
		videonames.resize(this->cameraNum);
		// get exist filenames
		cv::String path = this->dir;
		std::vector<cv::String> dirFiles;
		cv::glob(path, dirFiles);
		for (size_t i = 0; i < this->cameraNum; i++) {
			char videoname[1024];
			sprintf(videoname, "%s/%s", this->dir.c_str(), filenames[i].c_str());
			videonames[i] = std::string(videoname);
			if (!isFileExists(videonames[i])) {
				for (size_t k = 0; k < dirFiles.size(); k++) {
					std::size_t found = dirFiles[k].find(filenames[i]);
					if (found != std::string::npos) {
						videonames[i] = dirFiles[k];
						break;
					}
				}
			}
		}
		// get camera infos
		camInfos.resize(this->cameraNum);
		this->getCamInfos(camInfos);

		// check if sync file is existed
		syncfile = cv::format("%s/sync.txt", dir.c_str());
		std::replace(syncfile.begin(), syncfile.end(), '/', '\\');
		if (isFileExists(syncfile)) {
			frameInds.resize(camNum);
			timeStamps.resize(camNum);
			hasSyncFile = true;
			std::fstream fsync(syncfile.c_str(), std::ios::in);
			int trueInd = 0;
			for (;;) {
				int readInd;
				int frameInd;
				uint64_t timeStamp;
				fsync >> readInd;
				if (readInd != trueInd) {
					break;
				}
				for (int j = 0; j < camNum; j++) {
					fsync >> frameInd;
					fsync >> timeStamp;
					frameInds[j].push_back(frameInd);
					timeStamps[j].push_back(timeStamp);
				}
				trueInd++;
			}
			fsync.close();
		}
		return 0;
	}

	/**
	@brief buffer image data
	@return int
	*/
	int GenCameraFile::bufferImageData() {
		//syncInd = 0;
		syncInd = startFrameInd;
		syncIndNext = syncInd + 1;
		this->bufferImgs.resize(this->bufferSize);
		this->readers.resize(this->cameraNum);
		for (size_t i = 0; i < bufferSize; i++) {
			this->bufferImgs[i].resize(this->cameraNum);
		}
		// malloc mat memory
		for (size_t i = 0; i < this->cameraNum; i++) {
			int width, height;
			width = camInfos[i].width;
			height = camInfos[i].height;
			size_t length = width * height;
			for (size_t j = 0; j < bufferSize; j++) {
				if (this->camPurpose != cam::GenCamCapturePurpose::FileCameraRecording) {
					this->bufferImgs[j][i].data = new char[length];
					this->bufferImgs[j][i].length = length * sizeof(uchar);
					this->bufferImgs[j][i].maxLength = length * sizeof(uchar);
					this->bufferImgs[j][i].type = this->bufferType;
				}
				else {
					this->bufferImgs[j][i].data = new char[length * 3];
					this->bufferImgs[j][i].length = length * 3 * sizeof(uchar);
					this->bufferImgs[j][i].maxLength = length * 3 * sizeof(uchar);
					this->bufferImgs[j][i].type = this->bufferType;
				}
			}
		}
		// get video start frame index
		for (size_t i = 0; i < this->cameraNum; i++) {
			SysUtil::infoOutput("Buffer video " + filenames[i]);
			std::string fileExtension = videonames[i].substr(videonames[i].find_last_of(".") + 1);
			if (fileExtension.compare("avi") == 0 || fileExtension.compare("mp4") == 0) {
				readers[i].open(videonames[i]);
				if (hasSyncFile == false) {
					readers[i].set(cv::CAP_PROP_POS_FRAMES, startFrameInd + frameshifts[i]);
					SysUtil::infoOutput(cv::format("Video %s, start buffering from index %d ...",
						videonames[i].c_str(), startFrameInd + frameshifts[i]));
				}
				else {
					readers[i].set(cv::CAP_PROP_POS_FRAMES, frameInds[i][syncInd]);
					SysUtil::infoOutput(cv::format("Video %s, start buffering from index %d ...",
						videonames[i].c_str(), frameInds[i][syncInd]));
				}
				cv::Mat img, smallImg, bayerImg;
				for (size_t j = 0; j < bufferSize; j++) {
					if (hasSyncFile == false || j == 0) {
						readers[i] >> img;
					}
					else {
						int frameNum = frameInds[i][syncIndNext + j] - frameInds[i][syncInd + j - 1];
						for (int k = 0; k < frameNum; k++) {
							readers[i] >> img;
						}
						//syncInd++;
						//syncIndNext++;
						//if (syncIndNext >= frameInds[i].size())
						//	syncIndNext = 0;
					}
					cv::resize(img, smallImg, cv::Size(camInfos[i].width, camInfos[i].height));
					if (this->camPurpose != cam::GenCamCapturePurpose::FileCameraRecording) {
						bayerImg = colorBGR2BayerRG(smallImg);
						this->bufferImgs[j][i].length = sizeof(uchar) * bayerImg.rows * bayerImg.cols;
						memcpy(this->bufferImgs[j][i].data, bayerImg.data,
							this->bufferImgs[j][i].length);
					}
					else {
						this->bufferImgs[j][i].length = sizeof(uchar) * smallImg.rows * smallImg.cols * 3;
						memcpy(this->bufferImgs[j][i].data, smallImg.data,
							this->bufferImgs[j][i].length);
					}
				}
				if (this->camPurpose != cam::GenCamCapturePurpose::FileCameraRecording) {
					readers[i].release();
				}
			}
			else if (fileExtension.compare("jpg") == 0 || fileExtension.compare("png") == 0 ||
				fileExtension.compare("JPG") == 0 || fileExtension.compare("PNG") == 0) {
				cv::Mat img = cv::imread(videonames[i]);
				cv::Mat smallImg, bayerImg;
				cv::resize(img, smallImg, cv::Size(camInfos[i].width, camInfos[i].height));
				bayerImg = colorBGR2BayerRG(smallImg);
				// assign to buffer
				if (this->camPurpose != cam::GenCamCapturePurpose::FileCameraRecording) {
					for (size_t j = 0; j < bufferSize; j++) {
						this->bufferImgs[j][i].length = sizeof(uchar) * bayerImg.rows * bayerImg.cols;
						memcpy(this->bufferImgs[j][i].data, bayerImg.data,
							this->bufferImgs[j][i].length);
					}
				}
				else {
					for (size_t j = 0; j < bufferSize; j++) {
						this->bufferImgs[j][i].length = sizeof(uchar) * smallImg.rows * smallImg.cols * 3;
						memcpy(this->bufferImgs[j][i].data, smallImg.data,
							this->bufferImgs[j][i].length);
					}
				}
			}
			else {
				SysUtil::errorOutput("Unknown file type for FileCamera, only avi, mp4, jpg, png are support !");
			}
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
			std::string fileExtension = videonames[i].substr(videonames[i].find_last_of(".") + 1);
			if (fileExtension.compare("avi") == 0 || fileExtension.compare("mp4") == 0) {
				cv::VideoCapture reader(videonames[i]);
				camInfos[i].fps = reader.get(cv::CAP_PROP_FPS);
				camInfos[i].width = reader.get(cv::CAP_PROP_FRAME_WIDTH)
					* this->bufferScale;
				camInfos[i].height = reader.get(cv::CAP_PROP_FRAME_HEIGHT)
					* this->bufferScale;
				frameCounts[i] = reader.get(cv::CAP_PROP_FRAME_COUNT);
				reader.release();
			}
			else if (fileExtension.compare("jpg") == 0 || fileExtension.compare("png") == 0 ||
				fileExtension.compare("JPG") == 0 || fileExtension.compare("PNG") == 0) {
				cv::Mat img = cv::imread(videonames[i]);
				camInfos[i].fps = 1;
				camInfos[i].width = img.cols * this->bufferScale;
				camInfos[i].height = img.rows * this->bufferScale;
				frameCounts[i] = 1;
			}
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
		this->captureMode = captureMode;
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
			SysUtil::warningOutput(cv::format("Input bufferSize %d is too small change to %d !",
				bufferSize, finalBufferSize));
		}
		this->bufferSize = finalBufferSize;
		if (this->camPurpose == cam::GenCamCapturePurpose::FileCameraRecording) {
			SysUtil::infoOutput("Camera capturing purpose is set to file camera recording, "\
				"reset buffer size to 1 !");
			this->bufferSize = 1;
		}
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

	/*************************************************************/
	/*            function to update images in buffer            */
	/*************************************************************/
	/**
	@brief buffer next frame
	@return int
	*/
	int GenCameraFile::reBufferFileCamera() {
		for (size_t i = 0; i < this->cameraNum; i++) {
			if (readers[i].isOpened() == true) {
				readers[i].release();
			}
			readers[i].open(videonames[i]);
		}
		return 0;
	}

	/**
	@brief buffer next frame
	@return int
	*/
	int GenCameraFile::bufferNextFrame() {
		std::string fileExtension = videonames[0].substr(videonames[0].find_last_of(".") + 1);
		if (fileExtension.compare("avi") == 0 || fileExtension.compare("mp4") == 0) {
			bool isFinalFrame = false;
			for (size_t i = 0; i < this->cameraNum; i++) {
				//SysUtil::infoOutput("Buffer next frame of video " + filenames[i]);
				cv::Mat img, smallImg, bayerImg;
				int j = 0;
				if (hasSyncFile == false) {
					readers[i] >> img;
				}
				else {
					int frameNum = frameInds[i][syncIndNext] - frameInds[i][syncInd];
					for (int k = 0; k < frameNum; k++) {
						readers[i] >> img;
					}
				}

				//// extract background
				////cv::Ptr<cv::bgsegm::BackgroundSubtractorGMG> backSubPtr = cv::bgsegm::createBackgroundSubtractorGMG();
				//cv::Ptr<cv::BackgroundSubtractorMOG2> backSubPtr = cv::createBackgroundSubtractorMOG2();
				//// apply background subtractor
				//cv::Mat mask;
				//for (int backidx = 0; backidx < 40; backidx++) {
				//	backSubPtr->apply(img, mask, -1);
				//	readers[i] >> img;
				//	readers[i] >> img;
				//	readers[i] >> img;
				//	readers[i] >> img;
				//	readers[i] >> img;
				//	readers[i] >> img;
				//}
				//backSubPtr->getBackgroundImage(img);

				if (img.rows > 0) {
					if (img.rows != camInfos[i].height || img.cols != camInfos[i].width) {
						cv::resize(img, smallImg, cv::Size(camInfos[i].width, camInfos[i].height));
					}
					else smallImg = img;
					//bayerImg = colorBGR2BayerRG(smallImg);
					this->bufferImgs[j][i].length = sizeof(uchar) * smallImg.rows * smallImg.cols * 3;
					memcpy(this->bufferImgs[j][i].data, smallImg.data,
						this->bufferImgs[j][i].length);
				}
				else {
					isFinalFrame = true;
					break;
				}
			}
			if (hasSyncFile) {
				syncInd++;
				syncIndNext++;
				if (syncIndNext >= frameInds[0].size()) {
					isFinalFrame = true;
				}
			}
			if (isFinalFrame) {
				for (size_t i = 0; i < this->cameraNum; i++)
					readers[i].release();
				return 1;
			}
			return 0;
		}
		else {
			return 0;
		}
	}
};