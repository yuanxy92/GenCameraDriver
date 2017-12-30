/**
@brief General Camera Driver Class
@author Shane Yuan
@date Dec 29, 2017
*/

#ifndef __GENERAL_CAMERA_DRIVER_HPP__
#define __GENERAL_CAMERA_DRIVER_HPP__

// include std
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <queue>
#include <thread>
#include <memory>

// opencv
#include <opencv2/opencv.hpp>

// cuda
#ifdef _WIN32
#include <windows.h>
#endif
#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

namespace cam {

	class GenCamInfo {
		std::string sn;
		int width;
		int height;
	}

	enum class CameraModel {
		XIMEA_xiC,
		PointGrey_u3
	};

	class GenCamera {
	protected:
		CameraModel camModel;
		bool isInit;
		bool isCapture;
		std::vector<std::vector<cv::Mat>> bufferImgs;
	public:

	protected:
		size_t cameraNum;
	public:
		GenCamera();
		~GenCamera();

		/**
		@brief init camera
		@return int
		*/
		virtual int init() = 0;

		/**
		@brief start capture images
		@return int
		*/
		virtual int startCapture() = 0;

		/**
		@brief stop capture images
		@return int
		*/
		virtual int stopCapture() = 0;

		/**
		@brief release camera
		@return int
		*/
		virtual int release() = 0;

		/**
		@brief set frame rate
		@param int fps: input fps
		@return int
		*/
		virtual int setFPS(int fps) = 0;

		/**
		@brief set camera buffer size
		@param int bufferSize: input buffer size
		@return int
		*/
		virtual int setBufferSize(int bufferSize) = 0;

		/**
		@brief get camera information
		@param std::vector<GenCamInfo> & camInfos: output camera infos
		@return int
		*/
		virtual int getCamInfos(std::vector<GenCamInfo> camInfos) = 0;

		/**
		@brief capture images
		@param std::vector<cv::Mat> & imgs: output captured images
		@return int
		*/
		virtual int captureOneFrameBayer(std::vector<cv::Mat> & imgs) = 0;

	};

	/**
	@breif function to init camera array
	@return 
	*/
	std::shared_ptr<GenCamera> createCamera(CameraModel camModel);
};



#endif