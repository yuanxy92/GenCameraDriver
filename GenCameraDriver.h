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
#include <direct.h>
#endif
#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

namespace cam {

	// colorful terminal output and file utility
	enum class ConsoleColor {
		red = 12,
		blue = 9,
		green = 10,
		yellow = 14,
		white = 15,
		pink = 13,
		cyan = 11
	};

	class SysUtil {
	public:
		/***********************************************************/
		/*                    mkdir function                       */
		/***********************************************************/
		static int mkdir(char* dir) {
#ifdef WIN32
			_mkdir(dir);
#else
			char command[COMMAND_STRING_LENGTH];
			sprintf(command, "mkdir %s", dir);
			system(command);
#endif
			return 0;
		}
		static int mkdir(std::string dir) {
			return mkdir((char *)dir.c_str());
		}

		/***********************************************************/
		/*                    sleep function                       */
		/***********************************************************/
		static int sleep(size_t miliseconds) {
			std::this_thread::sleep_for(std::chrono::milliseconds(miliseconds));
			return 0;
		}

		/***********************************************************/
		/*             make colorful console output                */
		/***********************************************************/
		static int setConsoleColor(ConsoleColor color) {
			SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), static_cast<int>(color));
			return 0;
		}

		/***********************************************************/
		/*                 warning error output                    */
		/***********************************************************/
		static int errorOutput(std::string info) {
			SysUtil::setConsoleColor(ConsoleColor::red);
			std::cerr << "ERROR: " << info.c_str() << std::endl;
			SysUtil::setConsoleColor(ConsoleColor::white);
			return 0;
		}

		static int warningOutput(std::string info) {
			SysUtil::setConsoleColor(ConsoleColor::yellow);
			std::cerr << "WARNING: " << info.c_str() << std::endl;
			SysUtil::setConsoleColor(ConsoleColor::white);
			return 0;
		}

		static int infoOutput(std::string info) {
			SysUtil::setConsoleColor(ConsoleColor::green);
			std::cerr << "INFO: " << info.c_str() << std::endl;
			SysUtil::setConsoleColor(ConsoleColor::white);
			return 0;
		}

		static int debugOutput(std::string info) {
			SysUtil::setConsoleColor(ConsoleColor::pink);
			std::cerr << "DEBUG INFO: " << info.c_str() << std::endl;
			SysUtil::setConsoleColor(ConsoleColor::white);
			return 0;
		}
	};

	enum class Status {
		on = 1,
		off = 0
	};

	enum class CameraModel {
		XIMEA_xiC,
		PointGrey_u3
	};

	/**
	@brief camera info class
	*/
	class GenCamInfo {
	public:
		std::string sn;
		int width;
		int height;
		float fps;
		Status autoExposure;
		Status autoWhiteBalance;
	};

	/**
	@brief capture mode class
	*/
	enum class GenCamCaptureMode {
		Single, // capture one by one without buffer
		Continous, // capture and buffer images
		SingleTrigger,
		ContinousTrigger
	};

	/**
	@brief buffer type
	*/
	enum class GenCamBufferType {
		Raw,  // save raw images in buffer
		JPEG, // save jpeg compressed images in buffer
			  // usually need as power GPU to compress the raw images
		RGB   // save demosaiced 3 channel RGB images in buffer 
	};
	
	/**
	@brief class to save JPEG data
	*/
	class JPEGdata {
	public:
		char* data; // jpeg data pointer
		size_t length; // jpeg data length
	};

	/**
	@brief general camera class
	*/
	class GenCamera {
	protected:
		// camera model
		CameraModel camModel;
		std::vector<GenCamInfo> camInfos;
		// camera status
		bool isInit;
		bool isCapture;
		bool isVerbose; // enable log in capturing images
		// camera buffer
		GenCamBufferType bufferType;
		std::vector<std::vector<cv::Mat>> bufferImgs;
		std::vector<std::vector<JPEGdata>> bufferJPEGImgs;
		int bufferSize;
		size_t cameraNum;
		// threads used for capturing images
		GenCamCaptureMode captureMode;
		// threads to capture images
		std::vector<std::thread> ths; 
		// status of capturing threads
		// true: keep capturing images, false: stop capturing images
		std::vector<bool> thStatus; 
		// frame indices in buffer
		std::vector<int> thBufferInds; 
	public:

	protected:

	public:
		GenCamera();
		~GenCamera();

		/*************************************************************/
		/*                   basic camera function                   */
		/*************************************************************/
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
		@brief get camera information
		@param std::vector<GenCamInfo> & camInfos: output camera infos
		@return int
		*/
		virtual int getCamInfos(std::vector<GenCamInfo> & camInfos) = 0;

		/*************************************************************/
		/*                  camera setting function                  */
		/*************************************************************/
		/**
		@brief set frame rate
		@param float fps: input fps
		@return int
		*/
		virtual int setFPS(int camInd, float fps) = 0;

		/**
		@brief set auto white balance
		@param int ind: index of camera (-1 means all the cameras)
		@param Status autoWB: if use auto white balance
		@return int
		*/
		virtual int setAutoWhiteBalance(int camInd, Status autoWB) = 0;

		/**
		@brief set white balance
		@param int camInd: index of camera (-1 means all the cameras)
		@param float red: gain of red channel
		@param float green: gain of green channel
		@param float blue: gain of blur channel
		@return int
		*/
		virtual int setWhiteBalance(int camInd, float red,
			float green, float blue) = 0;

		/**
		@brief set auto exposure
		@param int ind: index of camera (-1 means all the cameras)
		@param Status autoExposure: if use auto exposure
		@return int
		*/
		virtual int setAutoExposure(int camInd, Status autoExposure) = 0;

		/**
		@brief set auto exposure level
		@param int ind: index of camera (-1 means all the cameras)
		@param float level: auto exposure level, average intensity of output
		signal AEAG should achieve
		@return int
		*/
		virtual int setAutoExposureLevel(int camInd, float level) = 0;

		/**
		@brief set exposure time
		@param int camInd: index of camera (-1 means all the cameras)
		@param int time: exposure time (in microseconds)
		@return int
		*/
		virtual int setExposure(int camInd, int time) = 0;

		/**
		@brief make setting effective
		by capturing some frames
		@return int
		*/
		virtual int makeSetEffective(int k = 10) = 0;

		/*************************************************************/
		/*                     capturing function                    */
		/*************************************************************/

		/**
		@brief capture single image of single camera in camera array
		@param int camInd: input index of camera 
		@param cv::Mat & img: output captured images 
		@return int
		*/
		virtual int captureFrame(int camInd, cv::Mat & img) = 0;

		/**
		@brief multi-thread capturing function 
		used for continous mode
		thread function to get images from camera and buffer to vector
		and wait until the next frame (based on fps)
		*/
		void capture_thread_(int camInd);

		/**
		@brief multi-thread captureing function
		used for single mode
		thread function to get images from camera and buffer to vector
		*/
		void capture_thread_single_(int camInd, cv::Mat & img);

		/**
		@brief set capturing mode
		@param GenCamCaptureMode captureMode: capture mode
		@param int size: buffer size
		@return int
		*/
		int setCaptureMode(GenCamCaptureMode captureMode,
			int bufferSize);

		/**
		@brief start capture threads
		@return int 
		*/
		int startCaptureThreads();

		/**
		@brief stop capture threads
		@return int
		*/
		int stopCaptureThreads();

		/**
		@brief capture one frame
		@param std::vector<cv::Mat> & imgs: output captured images
		if in single mode, memory of image mats should be malloced 
		before using this function
		@return int
		*/
		int captureFrame(std::vector<cv::Mat> & imgs);

	};

	/**
	@breif function to init camera array
	@return 
	*/
	std::shared_ptr<GenCamera> createCamera(CameraModel camModel);
};



#endif