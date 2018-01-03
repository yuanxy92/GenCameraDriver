/**
@brief General Camera Driver Class
Implementation of XIMEA camera
@author Shane Yuan
@date Dec 31, 2017
*/

#include "PointGreyCamera.h"

namespace cam {
	// function to check XIMEA function error
	void cam::checkPTGREYInternal(int result, char const *const func,
		const char *const file, int const line) {
		if (result != 0) {
			char info[256];
			sprintf(info, "PointGrey camera error at %s:%d function: %s\n",
				file, line, func);
			SysUtil::errorOutput(info);
			exit(-1);
		}
	}
	
	// constructor
	GenCameraPTGREY::GenCameraPTGREY() {}
	GenCameraPTGREY::~GenCameraPTGREY() {}

	/**
	@brief init camera
	@return int
	*/
	int GenCameraPTGREY::init() {
		// get camera lists
		sysPtr = Spinnaker::System::GetInstance();
		camList = sysPtr->GetCameras();
		this->cameraNum = camList.GetSize();
		// init cameras
		try {
			// init camera
			for (size_t i = 0; i < this->cameraNum; i++) {
				camList.GetByIndex(i)->Init();
			}
			// set some default values
			for (size_t i = 0; i < this->cameraNum; i ++) {
				// set camera inside buffers to 1 (can get the newest image)
				Spinnaker::CameraPtr pCam = camList.GetByIndex(i);	
				Spinnaker::GenApi::INodeMap & sNodeMap = pCam->GetStreamNodeMap();
				CIntegerPtr StreamNode = sNodeMap.GetNode(“StreamDefaultBufferCount”);
				INT64 bufferCount = StreamNode->GetValue();
				StreamNode->SetValue(1);
				// set pixel format to bayer 8

			}
		}
		catch (Spinnaker::Exception &e) {
			SysUtil::errorOutput(e.what());
		}
		// init images
		ptgreyImages.resize(this->cameraNum);
		ths.resize(this->cameraNum);
		thStatus.resize(this->cameraNum);
		this->isInit = true;
		return 0;
	}

	/**
	@brief get camera information
	@param std::vector<GenCamInfo> & camInfos: output camera infos
	@return int
	*/
	int GenCameraPTGREY::getCamInfos(std::vector<GenCamInfo> & camInfos) {
		camInfos.resize(this->cameraNum);
		try {
			for (size_t i = 0; i < this->cameraNum; i++) {
				// Select camera
				Spinnaker::CameraPtr pCam = camList.GetByIndex(i);
				Spinnaker::GenApi::INodeMap & nodeMap = pCam->GetNodeMap();
				// get serial number
				Spinnaker::GenApi::CStringPtr ptrStringSerial = nodeMap.GetNode("DeviceSerialNumber");
				camInfos[i].sn = ptrStringSerial->GetValue();
				// get image size
				Spinnaker::GenApi::CIntegerPtr widthPtr = nodeMap.GetNode("Width");
				camInfos[i].width = widthPtr->GetValue();
				Spinnaker::GenApi::CIntegerPtr heightPtr = nodeMap.GetNode("Height");
				camInfos[i].height = heightPtr->GetValue();
				// get fps
				Spinnaker::GenApi::CFloatPtr fpsPtr = nodeMap.GetNode("AcquisitionFrameRate");
				camInfos[i].fps = fpsPtr->GetValue();
				// get auto exposure status
				Spinnaker::GenApi::CEnumerationPtr exposureAutoPtr = nodeMap.GetNode("ExposureAuto");
				if (exposureAutoPtr->GetIntValue() == exposureAutoPtr->GetEntryByName("Off")->GetValue()) {
					camInfos[i].autoExposure = Status::off;
				}
				else {
					camInfos[i].autoExposure = Status::on;
				}
				// get bayer pattern
				Spinnaker::GenApi::CStringPtr bayerPatternPtr = nodeMap.GetNode("PixelColorFilter");
				std::string bayerPattern = bayerPatternPtr->GetValue();
				if (bayerPattern.compare("BayerRG") == 0) {
					camInfos[i].bayerPattern = GenCamBayerPattern::BayerRGGB;
				}
				else if (bayerPattern.compare("BayerBG") == 0) {
					camInfos[i].bayerPattern = GenCamBayerPattern::BayerBGGR;
				}
				else if (bayerPattern.compare("BayerGB") == 0) {
					camInfos[i].bayerPattern = GenCamBayerPattern::BayerGBRG;
				}
				else if (bayerPattern.compare("BayerGR") == 0) {
					camInfos[i].bayerPattern = GenCamBayerPattern::BayerGBRG;
				}
				// get white balance gain
				Spinnaker::GenApi::CFloatPtr balanceRatioPtr = nodeMap.GetNode("BalanceRatio");
				Spinnaker::GenApi::CEnumerationPtr balanceRatioSelectorPtr = nodeMap.GetNode("BalanceRatioSelector");
				balanceRatioSelectorPtr->SetIntValue(balanceRatioSelectorPtr->GetEntryByName("Blue")->GetValue());
				camInfos[i].blueGain = balanceRatioPtr->GetValue();
				balanceRatioSelectorPtr->SetIntValue(balanceRatioSelectorPtr->GetEntryByName("Red")->GetValue());
				camInfos[i].redGain = balanceRatioPtr->GetValue();
				camInfos[i].greenGain = 1.0f;
			}
		}
		catch (Spinnaker::Exception &e) {
			SysUtil::errorOutput(e.what());
		}
		return 0;
	}

	/**
	@brief start capture images
	@return int
	*/
	int GenCameraPTGREY::startCapture() {
		return 0;
	}

	/**
	@brief stop capture images
	@return int
	*/
	int GenCameraPTGREY::stopCapture() {
		return 0;
	}

	/**
	@brief release camera
	@return int
	*/
	int GenCameraPTGREY::release() {
		return 0;
	}

	/***********************************************************/
	/*                  camera setting functions               */
	/***********************************************************/
	/**
	@brief set frame rate
	@param float fps: input fps
	@return int
	*/
	int GenCameraPTGREY::setFPS(int camInd, float fps) {
		try {
			size_t beginInd, endInd;
			if (camInd == -1) {
				beginInd = 0;
				endInd = this->cameraNum - 1;
			}
			else {
				beginInd = camInd;
				endInd = camInd;
			}
			for (size_t i = beginInd; i <= endInd; i ++) {
				// Select camera
				Spinnaker::CameraPtr pCam = camList.GetByIndex(i);
				Spinnaker::GenApi::INodeMap & nodeMap = pCam->GetNodeMap();
				// turn of FPS auto to off
				
				// set fps
				Spinnaker::GenApi::CFloatPtr fpsPtr = nodeMap.GetNode("AcquisitionFrameRate");
				fpsPtr->SetValue(fps);	
			}
		}
		catch (Spinnaker::Exception &e) {
			SysUtil::errorOutput(e.what());
		}
		return 0;
	}

	/**
	@brief set auto white balance
	@param int ind: index of camera (-1 means all the cameras)
	@return int
	*/
	int GenCameraPTGREY::setWhiteBalance(int camInd) {
		return 0;
	}

	/**
	@brief set auto exposure
	@param int ind: index of camera (-1 means all the cameras)
	@param Status autoExposure: if use auto exposure
	@return int
	*/
	int GenCameraPTGREY::setAutoExposure(int camInd, Status autoExposure) {
		return 0;
	}

	/**
	@brief set auto exposure level
	for pointgrey camera auto exposure level is adjusted by EV
	@param int ind: index of camera (-1 means all the cameras)
	@param float level: auto exposure level, average intensity of output
	signal AEAG should achieve
	@return int
	*/
	int GenCameraPTGREY::setAutoExposureLevel(int camInd, float level) {
		return 0;
	}

	/**
	@brief set exposure time
	@param int ind: index of camera (-1 means all the cameras)
	@param int time: exposure time (in microseconds)
	@return int
	*/
	int GenCameraPTGREY::setExposure(int camInd, int time) {
		return 0;
	}

	/**
	@brief set/get bayer pattern
	@param int camInd: input camera index
	@param GenCamBayerPattern & bayerPattern: output bayer pattern
	@return int
	*/
	int GenCameraPTGREY::getBayerPattern(int camInd, GenCamBayerPattern & bayerPattern) {
		try {
			Spinnaker::CameraPtr pCam = camList.GetByIndex(camInd);
			Spinnaker::GenApi::INodeMap & nodeMap = pCam->GetNodeMap();
			Spinnaker::GenApi::CStringPtr bayerPatternPtr = nodeMap.GetNode("PixelColorFilter");
			std::string bayerPattern = bayerPatternPtr->GetValue();
			if (bayerPattern.compare("BayerRG") == 0) {
				bayerPattern = GenCamBayerPattern::BayerRGGB;
			}
			else if (bayerPattern.compare("BayerBG") == 0) {
				bayerPattern = GenCamBayerPattern::BayerBGGR;
			}
			else if (bayerPattern.compare("BayerGB") == 0) {
				bayerPattern = GenCamBayerPattern::BayerGBRG;
			}
			else if (bayerPattern.compare("BayerGR") == 0) {
				bayerPattern = GenCamBayerPattern::BayerGBRG;
			}
		}
		catch (Spinnaker::Exception &e) {
			SysUtil::errorOutput(e.what());
		} 
		return 0;
	}

	/**
	@brief make setting effective
	by capturing some frames
	@param int k: capture image frames (default is 10)
	@return int
	*/
	int GenCameraPTGREY::makeSetEffective(int k) {
		return 0;
	}

	/*************************************************************/
	/*                     capturing function                    */
	/*************************************************************/
	/**
	@brief capture single image of single camera in camera array
	@param int camInd: input index of camera
	@param cv::Mat & img: output captured images (pre-allocated memory)
	@return int
	*/
	int GenCameraPTGREY::captureFrame(int camInd, cv::Mat & img) {
		return 0;
	}
}