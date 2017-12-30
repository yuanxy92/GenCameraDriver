/**
@brief General Camera Driver Class
Implementation of XIMEA camera
@author Shane Yuan
@date Dec 29, 2017
*/

#ifndef __GENERAL_CAMERA_DRIVER_XIMEA_HPP__
#define __GENERAL_CAMERA_DRIVER__XIMEA_HPP__

#include "GenCameraDriver.h"

#ifdef WIN32
#include <windows.h>
#include "xiApi.h"       // Windows
#else
#include <m3api/xiApi.h> // Linux, OSX
#endif
#include <memory.h>

namespace cam {

	// function to check XIMEA function error
	void check(XI_RETURN result, char const *const func,
		const char *const file, int const line);

	// XIMEA function safe call
	#define checkXIMEAErrors(val)  check ( (val), #val, __FILE__, __LINE__ )

	class GenCameraXIMEA : public GenCamera {
	private:
		std::vector<HANDLE> hcams;
		std::vector<XI_IMG> xiImages;
	public:

	private:
		

	public:
		GenCameraXIMEA();
		~GenCameraXIMEA();

		/***********************************************************/
		/*                   basic camera functions                */
		/***********************************************************/
		/**
		@brief init camera
		@return int
		*/
		int init() override;

		/**
		@brief get camera information
		@param std::vector<GenCamInfo> & camInfos: output camera infos
		@return int
		*/
		int getCamInfos(std::vector<GenCamInfo> & camInfos) override;

		/**
		@brief start capture images
		@return int
		*/
		int startCapture() override;

		/**
		@brief stop capture images
		@return int
		*/
		int stopCapture() override;

		/**
		@brief release camera
		@return int
		*/
		int release() override;

		/***********************************************************/
		/*                  camera setting functions               */
		/***********************************************************/
		/**
		@brief set frame rate
		@param float fps: input fps
		@return int
		*/
		int setFPS(int camInd, float fps) override;

		/**
		@brief set auto white balance
		@param int ind: index of camera (-1 means all the cameras)
		@param Status autoWB: if use auto white balance
		@return int
		*/
		int setAutoWhiteBalance(int camInd, Status autoWB) override;

		/**
		@brief set white balance
		@param int camInd: index of camera (-1 means all the cameras)
		@param float red: gain of red channel
		@param float green: gain of green channel
		@param float blue: gain of blur channel
		@return int
		*/
		int setWhiteBalance(int camInd, float red,
			float green, float blue) override;

		/**
		@brief set auto exposure
		@param int ind: index of camera (-1 means all the cameras)
		@param Status autoExposure: if use auto exposure 
		@return int
		*/
		int setAutoExposure(int camInd, Status autoExposure) override;

		/**
		@brief set auto exposure level
		@param int ind: index of camera (-1 means all the cameras)
		@param float level: auto exposure level, average intensity of output
		signal AEAG should achieve
		@return int
		*/
		int setAutoExposureLevel(int camInd, float level) override;

		/**
		@brief set exposure time
		@param int ind: index of camera (-1 means all the cameras)
		@param int time: exposure time (in microseconds)
		@return int
		*/
		int setExposure(int camInd, int time) override;

		/**
		@brief make setting effective
		by capturing some frames
		@param int k: capture image frames (default is 10)
		@return int
		*/
		int makeSetEffective(int k = 10) override;

		/*************************************************************/
		/*                     capturing function                    */
		/*************************************************************/
		/**
		@brief set capturing mode
		@param GenCamCaptureMode captureMode: capture mode
		@param int size: buffer size
		@return
		*/
		int setCaptureMode(GenCamCaptureMode captureMode,
			int bufferSize) override;

		/**
		@brief capture images
		@param std::vector<cv::Mat> & imgs: output captured images
		@return int
		*/
		int captureOneFrameBayer(std::vector<cv::Mat> & imgs) override;
	};

};

#endif