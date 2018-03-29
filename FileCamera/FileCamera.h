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
		size_t camNum;
		float bufferScale;
		std::vector<std::string> filenames;
		// pre-define bayer pattern
		GenCamBayerPattern bayerPattern;

	public:

	private:
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
		int startCapture() {}
		int stopCapture() {}
		int setFPS(int camInd, float fps, float exposureUpperLimitRatio) {}
		int setAutoWhiteBalance(int camInd) {}
		int setWhiteBalance(int camInd, float redGain,
			float greenGain, float blueGain) {}
		int setAutoExposure(int camInd, Status autoExposure) {}
		int setAutoExposureLevel(int camInd, float level) {}
		int setAutoExposureCompensation(int camInd,
			Status status, float relativeEV) {}
		int setExposure(int camInd, int time) {}
		int makeSetEffective(int k) {}
		int waitForRecordFinish() {}
		int startCaptureThreads() {}
		int stopCaptureThreads() {}

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