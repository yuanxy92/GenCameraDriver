/**
@brief General Camera Driver Class
Implementation of XIMEA camera
@author Shane Yuan
@date Dec 29, 2017
*/

#include "XIMEACamera.h"

namespace cam {
	// function to check XIMEA function error
	void cam::checkXIMEAInternal(XI_RETURN result, char const *const func,
		const char *const file, int const line) {
		if (result != XI_OK) {
			char info[256];
			sprintf(info, "XIMEA camera error at %s:%d function: %s\n",
				file, line, func);
			SysUtil::errorOutput(info);
			exit(-1);
		}
	}

	GenCameraXIMEA::GenCameraXIMEA() {}
	GenCameraXIMEA::~GenCameraXIMEA() {}

	/***********************************************************/
	/*                   basic camera functions                */
	/***********************************************************/
	/**
	@brief make setting effective
	by capturing some frames
	@return int
	*/
	int GenCameraXIMEA::makeSetEffective(int k) {
		if (!this->isCapture) {
			SysUtil::warningOutput("This function must be executed after " \
				"starting capturing !");
			return -1;
		}
		for (size_t k = 0; k < 10; k++) {
			for (size_t i = 0; i < this->cameraNum; i++) {
				checkXIMEAErrors(xiGetImage(hcams[i], 500, &xiImages[i]));
			}
		}
		return 0;
	}


	/**
	@brief init camera
	@return int
	*/
	int GenCameraXIMEA::init() {
		// get camera numbers
		PDWORD num = new DWORD;
		checkXIMEAErrors(xiGetNumberDevices(num));
		this->cameraNum = static_cast<size_t>(*num);
		delete num;
		// set parameter
		// open cameras
		hcams.resize(this->cameraNum);
		for (size_t i = 0; i < this->cameraNum; i++) {
			checkXIMEAErrors(xiOpenDevice(i, &hcams[i]));
		}
		// set output to raw/bayer 8
		for (size_t i = 0; i < this->cameraNum; i++) {
			checkXIMEAErrors(xiSetParamInt(hcams[i], 
				XI_PRM_IMAGE_DATA_FORMAT, XI_RAW8));
		}
		// set camera inside buffers to 1 (can get the newest image)
		for (size_t i = 0; i < this->cameraNum; i++) {
			xiSetParamInt(hcams[i], XI_PRM_BUFFERS_QUEUE_SIZE, 1);
		}
		// init images
		xiImages.resize(this->cameraNum);
		for (size_t i = 0; i < this->cameraNum; i++) {
			memset(&xiImages[i], 0, sizeof(xiImages[i]));
			xiImages[i].size = sizeof(XI_IMG);
		}
		ths.resize(this->cameraNum);
		thStatus.resize(this->cameraNum);
		this->isInit = true;
		return 0;
	}

	/**
	@brief start capture images
	@return int
	*/
	int GenCameraXIMEA::startCapture() {
		for (size_t i = 0; i < this->cameraNum; i++) {
			checkXIMEAErrors(xiSetParamInt(hcams[i], XI_PRM_AEAG, 1));
			checkXIMEAErrors(xiStartAcquisition(hcams[i]));
		}
		this->isCapture = true;
		return 0;
	}

	/**
	@brief stop capture images
	@return int
	*/
	int GenCameraXIMEA::stopCapture() {
		for (size_t i = 0; i < this->cameraNum; i++) {
			checkXIMEAErrors(xiStopAcquisition(hcams[i]));
		}
		this->isCapture = false;
		return 0;
	}

	/**
	@brief release camera
	@return int
	*/
	int GenCameraXIMEA::release() {
		if (this->isCapture == true)
			this->stopCapture();
		// close cameras
		for (size_t i = 0; i < this->cameraNum; i++) {
			checkXIMEAErrors(xiCloseDevice(hcams[i]));
		}
		this->isCapture = true;
		return 0;
	}

	/**
	@brief get camera information
	@param std::vector<GenCamInfo> & camInfos: output camera infos
	@return int
	*/
	int GenCameraXIMEA::getCamInfos(std::vector<GenCamInfo> & camInfos) {
		camInfos.resize(this->cameraNum);
		for (size_t i = 0; i < this->cameraNum; i++) {
			char sn[20] = "";
			xiGetParamString(hcams[i], XI_PRM_DEVICE_SN, sn, sizeof(sn));
			camInfos[i].sn = std::string(sn);
			int width, height;
			checkXIMEAErrors(xiGetParamInt(hcams[i], XI_PRM_WIDTH, &width));
			checkXIMEAErrors(xiGetParamInt(hcams[i], XI_PRM_HEIGHT, &height));
			camInfos[i].width = width;
			camInfos[i].height = height;
			float fps;
			int autoExposure, autoWhiteBalance;
			checkXIMEAErrors(xiGetParamFloat(hcams[i], XI_PRM_FRAMERATE, &fps));
			checkXIMEAErrors(xiGetParamInt(hcams[i], XI_PRM_AEAG, &autoExposure));
			checkXIMEAErrors(xiGetParamInt(hcams[i], XI_PRM_AUTO_WB, &autoWhiteBalance));
			camInfos[i].fps = fps;
			camInfos[i].autoExposure = static_cast<cam::Status>(autoExposure);
			camInfos[i].autoWhiteBalance = static_cast<cam::Status>(autoWhiteBalance);
		}
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
	int GenCameraXIMEA::setFPS(int camInd, float fps) {
		if (camInd == -1) {
			for (size_t i = 0; i < this->cameraNum; i++) {
				checkXIMEAErrors(xiSetParamFloat(hcams[i], XI_PRM_FRAMERATE, fps));
			}
		}
		else
			checkXIMEAErrors(xiSetParamFloat(hcams[camInd], XI_PRM_FRAMERATE, fps));
		return 0;
	}

	/**
	@brief set white balance
	@param int ind: index of camera (-1 means all the cameras)
	@param Status autoWB: if use auto white balance
	@return int
	*/
	int GenCameraXIMEA::setAutoWhiteBalance(int camInd, Status autoWB) {
		if (camInd == -1) {
			for (size_t i = 0; i < this->cameraNum; i++) {
				checkXIMEAErrors(xiSetParamInt(hcams[i], XI_PRM_AUTO_WB, 
					static_cast<int>(autoWB)));
			}
		}
		else
			checkXIMEAErrors(xiSetParamInt(hcams[camInd], XI_PRM_AUTO_WB,
				static_cast<int>(autoWB)));
		return 0;
	}

	/**
	@brief set white balance
	@param int camInd: index of camera (-1 means all the cameras)
	@param float red: gain of red channel
	@param float green: gain of green channel
	@param float blue: gain of blur channel
	@return int
	*/
	int GenCameraXIMEA::setWhiteBalance(int camInd, float red,
		float green, float blue) {
		if (camInd == -1) {
			for (size_t i = 0; i < this->cameraNum; i++) {
				// set auto white balance to off
				checkXIMEAErrors(xiSetParamInt(hcams[i], XI_PRM_AUTO_WB, 0));
				// set dest white balance
				checkXIMEAErrors(xiSetParamFloat(hcams[i], XI_PRM_WB_KR, red));
				checkXIMEAErrors(xiSetParamFloat(hcams[i], XI_PRM_WB_KG, green));
				checkXIMEAErrors(xiSetParamFloat(hcams[i], XI_PRM_WB_KB, blue));
			}
		}
		else {
			// set auto white balance to off
			checkXIMEAErrors(xiSetParamInt(hcams[camInd], XI_PRM_AUTO_WB, 0));
			// set dest white balance
			checkXIMEAErrors(xiSetParamFloat(hcams[camInd], XI_PRM_WB_KR, red));
			checkXIMEAErrors(xiSetParamFloat(hcams[camInd], XI_PRM_WB_KG, green));
			checkXIMEAErrors(xiSetParamFloat(hcams[camInd], XI_PRM_WB_KB, blue));
		}
		return 0;
	}

	/**
	@brief set exposure time
	@param int ind: index of camera (-1 means all the cameras)
	@param Status autoExposure: if use auto exposure
	@return int
	*/
	int GenCameraXIMEA::setAutoExposure(int camInd, Status autoExposure) {
		if (camInd == -1) {
			for (size_t i = 0; i < this->cameraNum; i++) {
				checkXIMEAErrors(xiSetParamInt(hcams[i], XI_PRM_AEAG, 
					static_cast<int>(autoExposure)));
			}
		}
		else
			checkXIMEAErrors(xiSetParamInt(hcams[camInd], XI_PRM_AEAG, 
				static_cast<int>(autoExposure)));
		
		return 0;
	}

	/**
	@brief set auto exposure level
	@param int ind: index of camera (-1 means all the cameras)
	@param float level: auto exposure level, average intensity of output
	signal AEAG should achieve
	@return int
	*/
	int GenCameraXIMEA::setAutoExposureLevel(int camInd, float level) {
		if (camInd == -1) {
			for (size_t i = 0; i < this->cameraNum; i++) {
				checkXIMEAErrors(xiSetParamFloat(hcams[i], XI_PRM_AEAG_LEVEL,
					level));
			}
		}
		else
			checkXIMEAErrors(xiSetParamFloat(hcams[camInd], XI_PRM_AEAG_LEVEL,
				level));

		for (size_t frame = 0; frame < 20; frame++) {
			for (size_t i = 0; i < this->cameraNum; i++) {
				checkXIMEAErrors(xiGetImage(hcams[i], 500, &xiImages[i]));
			}
		}
		return 0;
	}

	/**
	@brief set exposure time
	@param int camInd: index of camera (-1 means all the cameras)
	@param int time: exposure time (in microseconds)
	@return int
	*/
	int GenCameraXIMEA::setExposure(int camInd, int time) {
		if (camInd == -1) {
			for (size_t i = 0; i < this->cameraNum; i++) {
				checkXIMEAErrors(xiSetParamInt(hcams[i], XI_PRM_EXPOSURE, time));
			}
		}
		else
			checkXIMEAErrors(xiSetParamInt(hcams[camInd], XI_PRM_EXPOSURE, time));
		return 0;
	}

	/*************************************************************/
	/*                     capturing function                    */
	/*************************************************************/
	/**
	@brief capture single image of single camera in camera array
	@param int camInd: input index of camera
	@param cv::Mat & img: output captured images
	@return int
	*/
	int GenCameraXIMEA::captureFrame(int camInd, cv::Mat & img) {
		// capture images
		checkXIMEAErrors(xiGetImage(hcams[camInd], 500, &xiImages[camInd]));
		// copy to opencv mat
		memcpy(img.data, xiImages[camInd].bp, sizeof(unsigned char) *
			xiImages[camInd].width * xiImages[camInd].height);
		return 0;
	}

}