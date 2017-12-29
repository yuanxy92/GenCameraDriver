/**
@brief General Camera Driver Class
Implementation of XIMEA camera
@author Shane Yuan
@date Dec 29, 2017
*/



#include "XIMEACamera.h"

namespace cam {
	// function to check XIMEA function error
	void cam::check(XI_RETURN result, char const *const func,
		const char *const file, int const line) {
		if (result != XI_OK) {
			fprintf(stderr, "XIMEA camera error at %s:%d function: %s\n",
				file, line, func);
			exit(-1);
		}
	}

	GenCameraXIMEA::GenCameraXIMEA() {}
	GenCameraXIMEA::~GenCameraXIMEA() {}

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
		// init images
		xiImages.resize(this->cameraNum);
		for (size_t i = 0; i < this->cameraNum; i++) {
			memset(&xiImages[i], 0, sizeof(xiImages[i]));
			xiImages[i].size = sizeof(XI_IMG);
		}
		this->isInit = true;
		return 0;
	}

	/**
	@brief start capture images
	@return int
	*/
	int GenCameraXIMEA::startCapture() {
		for (size_t i = 0; i < this->cameraNum; i++) {
			//checkXIMEAErrors(xiSetParamInt(hcams[i], XI_PRM_EXPOSURE, 50000));
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
	@brief capture images
	@param std::vector<cv::Mat> & imgs: output captured images
	@return int
	*/
	int GenCameraXIMEA::captureOneFrameBayer(std::vector<cv::Mat> & imgs) {
		// capture images
		for (size_t i = 0; i < this->cameraNum; i++) {
			checkXIMEAErrors(xiGetImage(hcams[i], 100, &xiImages[i]));
		}
		// copy to opencv mat
		for (size_t i = 0; i < this->cameraNum; i++) {
			imgs[i].create(xiImages[i].height, xiImages[i].width, CV_8U);
			memcpy(imgs[i].data, xiImages[i].bp, sizeof(unsigned char) *
				xiImages[i].width * xiImages[i].height);
		}
		return 0;
	}

}