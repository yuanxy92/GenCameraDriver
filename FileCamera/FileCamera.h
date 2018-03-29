/**
@brief Generic Camera Driver Class
Implementation of PointGrey camera
@author Shane Yuan
@date Dec 31, 2017
*/

#ifndef __GENERIC_CAMERA_DRIVER_FILE_H__
#define __GENERIC_CAMERA_DRIVER_FILE_H__

#include <memory.h>

#include "GenCameraDriver.h"

namespace cam {

	class GenCameraFile : public GenCamera {
	private:
		// config information
		std::string dir;
		float bufferScale;
		std::vector<std::string> filenames;
		std::vector<size_t> frameCounts;
		// pre-define bayer pattern
		GenCamBayerPattern bayerPattern;

	public:

	private:
		/**
		@brief convert rgb image to bayer image
		@param cv::Mat img: input image CV_8UC3 Color_BGR
		@return cv::Mat out: output bayer image 
		*/
		static cv::Mat colorBGR2BayerRG(cv::Mat img);

		/**
		@brief load file information from config file
		@return int
		*/
		int loadConfigFile();

		/**
		@brief buffer image data
		@return int
		*/
		int bufferImageData();

	public:
		// constructor
		GenCameraFile();
		GenCameraFile(std::string dir);
		~GenCameraFile();

		/*************************************************************/
		/*          list of not used function in FileCamera          */
		/*************************************************************/
		int startCapture() { return 0; }
		int stopCapture() { return 0; }
		int setFPS(int camInd, float fps, 
			float exposureUpperLimitRatio) { return 0; }
		int setAutoWhiteBalance(int camInd) { return 0; }
		int setWhiteBalance(int camInd, float redGain,
			float greenGain, float blueGain) { return 0; }
		int setAutoExposure(int camInd, Status autoExposure) { return 0; }
		int setAutoExposureLevel(int camInd, float level) { return 0; }
		int setAutoExposureCompensation(int camInd,
			Status status, float relativeEV) { return 0;}
		int setExposure(int camInd, int time) { return 0; }
		int makeSetEffective(int k) { return 0; }
		int waitForRecordFinish() { return 0; }
		int startCaptureThreads() { return 0; }
		int stopCaptureThreads() { return 0; }

		/*************************************************************/
		/*                   basic camera function                   */
		/*************************************************************/
		/**
		@brief init function
		@return int
		*/
		int init();

		/**
		@brief release camera
		@return int
		*/
		int release();

		/**
		@brief get camera information
		@param std::vector<GenCamInfo> & camInfos: output camera infos
		@return int
		*/
		int getCamInfos(std::vector<GenCamInfo> & camInfos);

		/*************************************************************/
		/*                  camera setting function                  */
		/*************************************************************/
		/**
		@brief set/get bayer pattern
		@param int camInd: input camera index
		@param GenCamBayerPattern & bayerPattern: output bayer pattern
		@return int
		*/
		int getBayerPattern(int camInd, GenCamBayerPattern & bayerPattern);

		/*************************************************************/
		/*                     capturing function                    */
		/*************************************************************/
		/**
		@brief set capturing mode
		@param GenCamCaptureMode captureMode: capture mode
		@param int size: buffer size
		@return int
		*/
		int setCaptureMode(GenCamCaptureMode captureMode,
			int bufferSize);

	};

};

#endif