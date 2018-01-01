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
		return 0;
	}

	/**
	@brief get camera information
	@param std::vector<GenCamInfo> & camInfos: output camera infos
	@return int
	*/
	int GenCameraPTGREY::getCamInfos(std::vector<GenCamInfo> & camInfos) {
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
		return 0;
	}

	/**
	@brief set auto white balance
	@param int ind: index of camera (-1 means all the cameras)
	@param Status autoWB: if use auto white balance
	@return int
	*/
	int GenCameraPTGREY::setAutoWhiteBalance(int camInd, Status autoWB) {
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
	int GenCameraPTGREY::setWhiteBalance(int camInd, float red,
		float green, float blue) {
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