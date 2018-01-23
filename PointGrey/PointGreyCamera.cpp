/**
@brief Generic Camera Driver Class
Implementation of PointGrey camera
@author Shane Yuan
@date Dec 31, 2017
*/

#include "PointGreyCamera.h"

namespace cam {
	// function to check XIMEA function error
	void checkPTGREYInternal(int result, char const *const func,
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
	GenCameraPTGREY::GenCameraPTGREY() {
		this->camModel = CameraModel::PointGrey_u3;
	}
	GenCameraPTGREY::~GenCameraPTGREY() {}

	/***********************************************************/
	/*                   basic camera functions                */
	/***********************************************************/
	/**
	@brief make setting effective
	by capturing some frames
	@param int k: capture image frames (default is 10)
	@return int
	*/
	int GenCameraPTGREY::makeSetEffective(int k) {
		if (!this->isCapture) {
			SysUtil::warningOutput("This function must be executed after " \
				"starting capturing !");
			return -1;
		}
		Spinnaker::ImagePtr image;
		try {
			for (size_t k = 0; k < 10; k++) {
				for (size_t i = 0; i < this->cameraNum; i++) {
					image = camList.GetByIndex(i)->GetNextImage();
				}
			}
		}
		catch (Spinnaker::Exception &e) {
			SysUtil::errorOutput(e.GetFullErrorMessage());
			exit(-1);
		}
		return 0;
	}

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

				// set camera inside buffers strategy
				Spinnaker::CameraPtr pCam = camList.GetByIndex(i);	
				Spinnaker::GenApi::INodeMap & sNodeMap = pCam->GetTLStreamNodeMap();
				Spinnaker::GenApi::CIntegerPtr StreamNode = sNodeMap.GetNode("StreamDefaultBufferCount");
				StreamNode->SetValue(3);
				Spinnaker::GenApi::CEnumerationPtr StreamModeNode = sNodeMap.GetNode("StreamBufferHandlingMode");
				StreamModeNode->SetIntValue(StreamModeNode->GetEntryByName("OldestFirst")->GetValue());

				// set pixel format to bayer 8
				// get pixel format
				Spinnaker::GenApi::INodeMap & nodeMap = pCam->GetNodeMap();
				Spinnaker::GenApi::CEnumerationPtr pixelFormatPtr = nodeMap.GetNode("PixelFormat");
				// get pixel color filter
				Spinnaker::GenApi::CEnumerationPtr bayerPatternPtr = nodeMap.GetNode("PixelColorFilter");
				std::string bayerPattern = bayerPatternPtr->GetCurrentEntry()->GetName().c_str();
				std::string dstPixelFormat;
				if (bayerPattern.compare("EnumEntry_PixelColorFilter_BayerRG") == 0) {
					dstPixelFormat = "BayerRG8";
				}
				else if (bayerPattern.compare("EnumEntry_PixelColorFilter_BayerBG") == 0) {
					dstPixelFormat = "BayerBG8";
				}
				else if (bayerPattern.compare("EnumEntry_PixelColorFilter_BayerGB") == 0) {
					dstPixelFormat = "BayerGB8";
				}
				else if (bayerPattern.compare("EnumEntry_PixelColorFilter_BayerGR") == 0) {
					dstPixelFormat = "BayerGR8";
				}
				pixelFormatPtr->SetIntValue(pixelFormatPtr->
					GetEntryByName(Spinnaker::GenICam::gcstring(dstPixelFormat.c_str()))->GetValue());

				// Set acquisition mode to continuous
				Spinnaker::GenApi::CEnumerationPtr acquisitionModePtr = nodeMap.GetNode("AcquisitionMode");
				acquisitionModePtr->SetIntValue(acquisitionModePtr->GetEntryByName("Continuous")->GetValue());

				// Set auto exposure and auto gain
				Spinnaker::GenApi::CEnumerationPtr exposureModePtr = nodeMap.GetNode("ExposureMode");
				exposureModePtr->SetIntValue(exposureModePtr->GetEntryByName("Timed")->GetValue());
				Spinnaker::GenApi::CEnumerationPtr gainAutoPtr = nodeMap.GetNode("GainAuto");
				gainAutoPtr->SetIntValue(gainAutoPtr->GetEntryByName("Continuous")->GetValue());
				Spinnaker::GenApi::CEnumerationPtr exposureAutoPtr = nodeMap.GetNode("ExposureAuto");
				exposureAutoPtr->SetIntValue(exposureAutoPtr->GetEntryByName("Continuous")->GetValue());
			}
		}
		catch (Spinnaker::Exception &e) {
			SysUtil::errorOutput(e.GetFullErrorMessage());
			exit(-1);
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
				Spinnaker::GenApi::INodeMap & deviceNodeMap = pCam->GetTLDeviceNodeMap();
				// get serial number
				Spinnaker::GenApi::CStringPtr ptrStringSerial = deviceNodeMap.GetNode("DeviceSerialNumber");
				camInfos[i].sn = ptrStringSerial->GetValue();
				// get image size
				Spinnaker::GenApi::INodeMap & nodeMap = pCam->GetNodeMap();
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
				Spinnaker::GenApi::CEnumerationPtr bayerPatternPtr = nodeMap.GetNode("PixelColorFilter");
				std::string bayerPattern = bayerPatternPtr->GetCurrentEntry()->GetName().c_str();
				if (bayerPattern.compare("EnumEntry_PixelColorFilter_BayerRG") == 0) {
					camInfos[i].bayerPattern = GenCamBayerPattern::BayerRGGB;
				}
				else if (bayerPattern.compare("EnumEntry_PixelColorFilter_BayerBG") == 0) {
					camInfos[i].bayerPattern = GenCamBayerPattern::BayerBGGR;
				}
				else if (bayerPattern.compare("EnumEntry_PixelColorFilter_BayerGB") == 0) {
					camInfos[i].bayerPattern = GenCamBayerPattern::BayerGBRG;
				}
				else if (bayerPattern.compare("EnumEntry_PixelColorFilter_BayerGR") == 0) {
					camInfos[i].bayerPattern = GenCamBayerPattern::BayerGBRG;
				}
				// disable white balance auto
				Spinnaker::GenApi::CEnumerationPtr whiteBalanceAutoPtr = nodeMap.GetNode("BalanceWhiteAuto");
				if (Spinnaker::GenApi::IsWritable(whiteBalanceAutoPtr)) {
					whiteBalanceAutoPtr->SetIntValue(whiteBalanceAutoPtr->GetEntryByName("Off")->GetValue());
				}
				// get white balance gain
				Spinnaker::GenApi::CFloatPtr balanceRatioPtr = nodeMap.GetNode("BalanceRatio");
				Spinnaker::GenApi::CEnumerationPtr balanceRatioSelectorPtr = nodeMap.GetNode("BalanceRatioSelector");
				balanceRatioSelectorPtr->SetIntValue(balanceRatioSelectorPtr->GetEntryByName("Blue")->GetValue());
				camInfos[i].blueGain = balanceRatioPtr->GetValue();
				balanceRatioSelectorPtr->SetIntValue(balanceRatioSelectorPtr->GetEntryByName("Red")->GetValue());
				camInfos[i].redGain = balanceRatioPtr->GetValue();
				camInfos[i].greenGain = 1.0f;
				// set raw type, after white balance
				camInfos[i].isWBRaw = true;
			}
		}
		catch (Spinnaker::Exception &e) {
			SysUtil::errorOutput(e.GetFullErrorMessage());
			exit(-1);
		}
		return 0;
	}

	/**
	@brief start capture images
	@return int
	*/
	int GenCameraPTGREY::startCapture() {
		try {
			for (size_t camInd = 0; camInd < this->cameraNum; camInd ++) {
				camList.GetByIndex(camInd)->BeginAcquisition();
			}
		}
		catch (Spinnaker::Exception &e) {
			SysUtil::errorOutput(e.GetFullErrorMessage());
			exit(-1);
		}
		this->isCapture = true;
		return 0;
	}

	/**
	@brief stop capture images
	@return int
	*/
	int GenCameraPTGREY::stopCapture() {
		try {
			for (size_t camInd = 0; camInd < this->cameraNum; camInd++) {
				camList.GetByIndex(camInd)->EndAcquisition();
			}
		}
		catch (Spinnaker::Exception &e) {
			SysUtil::errorOutput(e.GetFullErrorMessage());
			exit(-1);
		}
		this->isCapture = false;
		return 0;
	}

	/**
	@brief release camera
	@return int
	*/
	int GenCameraPTGREY::release() {
		if (this->isCapture == true)
			this->stopCapture();
		// close cameras
		try {
			// de-init cameras
			for (size_t camInd = 0; camInd < this->cameraNum; camInd++) {
				camList.GetByIndex(camInd)->DeInit();
			}
			// clear camera list
			camList.Clear();
			// release system
			sysPtr->ReleaseInstance();
		}
		catch (Spinnaker::Exception &e) {
			SysUtil::errorOutput(e.GetFullErrorMessage());
			exit(-1);
		}
		return 0;
	}

	/***********************************************************/
	/*                  camera setting functions               */
	/***********************************************************/
	/**
	@brief set frame rate
	@param float fps: input fps
	@param float exposureUpperLimitRatio: exposure upper limit time, make
		exposure upper limit time = 1000000us / fps * 0.8
	@return int
	*/
	int GenCameraPTGREY::setFPS(int camInd, float fps, float exposureUpperLimitRatio) {
		// calculate max exposure time
		float maxExposureTime = 1000000.0f / fps * exposureUpperLimitRatio;
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
				Spinnaker::GenApi::CEnumerationPtr fpsAutoPtr = nodeMap.GetNode("AcquisitionFrameRateAuto");
				fpsAutoPtr->SetIntValue(fpsAutoPtr->GetEntryByName("Off")->GetValue());
				// set fps
				Spinnaker::GenApi::CFloatPtr fpsPtr = nodeMap.GetNode("AcquisitionFrameRate");
				fpsPtr->SetValue(fps);	
				// set auto exposure upper limit
				Spinnaker::GenApi::CFloatPtr exposureUpperLimitPtr = nodeMap.GetNode("AutoExposureTimeUpperLimit");
				exposureUpperLimitPtr->SetValue(maxExposureTime);
			}
		}
		catch (Spinnaker::Exception &e) {
			SysUtil::errorOutput(e.GetFullErrorMessage());
			exit(-1);
		}
		return 0;
	}

	/**
	@brief set auto white balance
	@param int ind: index of camera (-1 means all the cameras)
	@return int
	*/
	int GenCameraPTGREY::setAutoWhiteBalance(int camInd) {
		SysUtil::warningOutput("This function setAutoWhiteBalance "\
			" for PointGrey camera is not implemented yet.");
		return 0;
	}

	/**
	@brief set auto white balance
	@param int ind: index of camera (-1 means all the cameras)
	@param float redGain: red gain of the white balance
	@param float greenGain: green gain of the white balance
	@param float blueGain: blue gain of the white balance
	@return int
	*/
	int GenCameraPTGREY::setWhiteBalance(int camInd, float redGain,
		float greenGain, float blueGain) {
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

			for (size_t i = beginInd; i <= endInd; i++) {
				// Select camera
				Spinnaker::CameraPtr pCam = camList.GetByIndex(i);
				Spinnaker::GenApi::INodeMap & nodeMap = pCam->GetNodeMap();
				// make green gain into 1.0 first
				redGain = redGain / greenGain;
				blueGain = blueGain / greenGain;
				greenGain = 1.0f;
				// get white balance gain
				Spinnaker::GenApi::CFloatPtr balanceRatioPtr = nodeMap.GetNode("BalanceRatio");
				Spinnaker::GenApi::CEnumerationPtr balanceRatioSelectorPtr = nodeMap.GetNode("BalanceRatioSelector");
				balanceRatioSelectorPtr->SetIntValue(balanceRatioSelectorPtr->GetEntryByName("Blue")->GetValue());
				balanceRatioPtr->SetValue(blueGain);
				balanceRatioSelectorPtr->SetIntValue(balanceRatioSelectorPtr->GetEntryByName("Red")->GetValue());
				balanceRatioPtr->SetValue(redGain);
			}
		}
		catch (Spinnaker::Exception &e) {
			SysUtil::errorOutput(e.GetFullErrorMessage());
			exit(-1);
		}
		return 0;
	}

	/**
	@brief set auto exposure
	@param int ind: index of camera (-1 means all the cameras)
	@param Status autoExposure: if use auto exposure
	@return int
	*/
	int GenCameraPTGREY::setAutoExposure(int camInd, Status autoExposure) {
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

			Spinnaker::GenICam::gcstring status;
			if (autoExposure == Status::on)
				status = "Continuous";
			else if (autoExposure == Status::off)
				status = "Off";

			for (size_t i = beginInd; i <= endInd; i++) {
				// Select camera
				Spinnaker::CameraPtr pCam = camList.GetByIndex(i);
				Spinnaker::GenApi::INodeMap & nodeMap = pCam->GetNodeMap();
				// set auto exposure and auto gain status
				Spinnaker::GenApi::CEnumerationPtr gainAutoPtr = nodeMap.GetNode("GainAuto");
				gainAutoPtr->SetIntValue(gainAutoPtr->GetEntryByName("Continuous")->GetValue());
				Spinnaker::GenApi::CEnumerationPtr exposureAutoPtr = nodeMap.GetNode("ExposureAuto");
				exposureAutoPtr->SetIntValue(exposureAutoPtr->GetEntryByName("Continuous")->GetValue());
			}
		}
		catch (Spinnaker::Exception &e) {
			SysUtil::errorOutput(e.GetFullErrorMessage());
			exit(-1);
		}
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
		SysUtil::warningOutput("setAutoExposureLevel function is not "\
			"support for PointGrey camera. \n"\
			"Please use setAutoExposureCompensation instead !");
		return 0;
	}

	/**
	@brief set auto exposure compensation (only support PointGrey cameras)
	@param int ind: index of camera (-1 means all the cameras)
	@param Status status: if use auto EV value
	@param float relativeEV: only valid when the second argument is off.
	The reason why use relative EV value here is to directly set a absolute
	value is difficult
	@return int
	*/
	int GenCameraPTGREY::setAutoExposureCompensation(int camInd,
		Status status, float relativeEV) {
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

			Spinnaker::ImagePtr image;
			for (size_t camInd = beginInd; camInd <= endInd; camInd++) {
				// Select camera
				Spinnaker::CameraPtr pCam = camList.GetByIndex(camInd);
				Spinnaker::GenApi::INodeMap & nodeMap = pCam->GetNodeMap();
				// set auto exposure and auto gain status
				Spinnaker::GenApi::CEnumerationPtr evAuto = nodeMap.GetNode("pgrExposureCompensationAuto");
				evAuto->SetIntValue(evAuto->GetEntryByName("Continuous")->GetValue());
				image = camList.GetByIndex(camInd)->GetNextImage();
				SysUtil::sleep(500);
				image = camList.GetByIndex(camInd)->GetNextImage();
				SysUtil::sleep(500);
				if (status == Status::off) {
					// get value
					Spinnaker::GenApi::CFloatPtr evPtr = nodeMap.GetNode("pgrExposureCompensation");
					float evVal = evPtr->GetValue();
					// turn off auto EV
					evAuto->SetIntValue(evAuto->GetEntryByName("Off")->GetValue());
					// set new value
					evPtr->SetValue(evVal + relativeEV);
					image = camList.GetByIndex(camInd)->GetNextImage();
					SysUtil::sleep(500);
					image = camList.GetByIndex(camInd)->GetNextImage();
					SysUtil::sleep(500);
					char info[256];
					sprintf(info, "PointGrey camera %d, exposure compensation set to %f EV", camInd, evVal + relativeEV);
					SysUtil::infoOutput(info);
				}
			}
		}
		catch (Spinnaker::Exception &e) {
			SysUtil::errorOutput(e.GetFullErrorMessage());
			exit(-1);
		}
		return 0;
	}

	/**
	@brief set exposure time
	@param int ind: index of camera (-1 means all the cameras)
	@param int time: exposure time (in microseconds)
	@return int
	*/
	int GenCameraPTGREY::setExposure(int camInd, int time) {
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

			Spinnaker::ImagePtr image;
			for (size_t i = beginInd; i <= endInd; i++) {
				// Select camera
				Spinnaker::CameraPtr pCam = camList.GetByIndex(i);
				Spinnaker::GenApi::INodeMap & nodeMap = pCam->GetNodeMap();
				// set auto exposure and auto gain status
				Spinnaker::GenApi::CEnumerationPtr exposureAutoPtr = nodeMap.GetNode("ExposureAuto");
				exposureAutoPtr->SetIntValue(exposureAutoPtr->GetEntryByName("Off")->GetValue());
				Spinnaker::GenApi::CEnumerationPtr exposureModePtr = nodeMap.GetNode("ExposureMode");
				exposureModePtr->SetIntValue(exposureModePtr->GetEntryByName("Timed")->GetValue());
				Spinnaker::GenApi::CFloatPtr exposureTimePtr = nodeMap.GetNode("ExposureTime");
				exposureTimePtr->SetValue(time);
			}
		}
		catch (Spinnaker::Exception &e) {
			SysUtil::errorOutput(e.GetFullErrorMessage());
			exit(-1);
		}
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
			Spinnaker::GenApi::CEnumerationPtr bayerPatternPtr = nodeMap.GetNode("PixelColorFilter");
			std::string bayerPatternStr = bayerPatternPtr->GetCurrentEntry()->GetName().c_str();
			if (bayerPatternStr.compare("EnumEntry_PixelColorFilter_BayerRG") == 0) {
				bayerPattern = GenCamBayerPattern::BayerRGGB;
			}
			else if (bayerPatternStr.compare("EnumEntry_PixelColorFilter_BayerBG") == 0) {
				bayerPattern = GenCamBayerPattern::BayerBGGR;
			}
			else if (bayerPatternStr.compare("EnumEntry_PixelColorFilter_BayerGB") == 0) {
				bayerPattern = GenCamBayerPattern::BayerGBRG;
			}
			else if (bayerPatternStr.compare("EnumEntry_PixelColorFilter_BayerGR") == 0) {
				bayerPattern = GenCamBayerPattern::BayerGBRG;
			}
		}
		catch (Spinnaker::Exception &e) {
			SysUtil::errorOutput(e.GetFullErrorMessage());
			exit(-1);
		} 
		return 0;
	}

	/*************************************************************/
	/*                     capturing function                    */
	/*************************************************************/
	/**
	@brief capture single image of single camera in camera array
	@param int camInd: input index of camera
	@param Imagedata & img: output captured images (pre-allocated memory)
	@return int
	*/
	int GenCameraPTGREY::captureFrame(int camInd, Imagedata & img) {
		// capture images
		try {
			ptgreyImages[camInd] = camList.GetByIndex(camInd)->GetNextImage();
		}
		catch (Spinnaker::Exception &e) {
			SysUtil::errorOutput(e.GetFullErrorMessage());
			exit(-1);
		}
		// copy to opencv mat
		std::memcpy(img.data, ptgreyImages[camInd]->GetData(), sizeof(unsigned char) *
			ptgreyImages[camInd]->GetWidth() * ptgreyImages[camInd]->GetHeight());
		return 0;
	}
}