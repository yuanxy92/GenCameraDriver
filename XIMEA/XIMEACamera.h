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

		/**
		@brief init camera
		@return int
		*/
		int init() override;

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

		/**
		@brief capture images
		@param std::vector<cv::Mat> & imgs: output captured images
		@return int
		*/
		int captureOneFrameBayer(std::vector<cv::Mat> & imgs) override;
	};

};

#endif