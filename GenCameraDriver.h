/**
@brief Generic Camera Driver Class
@author Shane Yuan
@date Dec 29, 2017
*/

#ifndef __GENERIC_CAMERA_DRIVER_HPP__
#define __GENERIC_CAMERA_DRIVER_HPP__

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

// cuda npp JPEG coder
#include "NPPJpegCoder.h"

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

#ifndef WIN32
#define BLACK_TEXT(x) "\033[30;1m" x "\033[0m"
#define RED_TEXT(x) "\033[31;1m" x "\033[0m"
#define GREEN_TEXT(x) "\033[32;1m" x "\033[0m"
#define YELLOW_TEXT(x) "\033[33;1m" x "\033[0m"
#define BLUE_TEXT(x) "\033[34;1m" x "\033[0m"
#define MAGENTA_TEXT(x) "\033[35;1m" x "\033[0m"
#define CYAN_TEXT(x) "\033[36;1m" x "\033[0m"
#define WHITE_TEXT(x) "\033[37;1m" x "\033[0m"
#endif

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
#ifdef WIN32
			SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), static_cast<int>(color));
#endif
			return 0;
		}

		/***********************************************************/
		/*                 warning error output                    */
		/***********************************************************/
		static int errorOutput(std::string info) {
#ifdef WIN32
			SysUtil::setConsoleColor(ConsoleColor::red);
			std::cerr << "ERROR: " << info.c_str() << std::endl;
			SysUtil::setConsoleColor(ConsoleColor::white);
#else
			std::cerr << RED_TEXT("ERROR: ") << RED_TEXT(info.c_str())
				<< std::endl;
#endif
			return 0;
		}

		static int warningOutput(std::string info) {
#ifdef WIN32
			SysUtil::setConsoleColor(ConsoleColor::yellow);
			std::cerr << "WARNING: " << info.c_str() << std::endl;
			SysUtil::setConsoleColor(ConsoleColor::white);
#else
			std::cerr << YELLOW_TEXT("ERROR: ") << RED_TEXT(info.c_str())
				<< std::endl;
#endif
			return 0;
		}

		static int infoOutput(std::string info) {
#ifdef WIN32
			SysUtil::setConsoleColor(ConsoleColor::green);
			std::cerr << "INFO: " << info.c_str() << std::endl;
			SysUtil::setConsoleColor(ConsoleColor::white);
#else
			std::cerr << GREEN_TEXT("ERROR: ") << RED_TEXT(info.c_str())
				<< std::endl;
#endif
			return 0;
		}

		static int debugOutput(std::string info) {
#ifdef WIN32
			SysUtil::setConsoleColor(ConsoleColor::pink);
			std::cerr << "DEBUG INFO: " << info.c_str() << std::endl;
			SysUtil::setConsoleColor(ConsoleColor::white);
#else
			std::cerr << MAGENTA_TEXT("ERROR: ") << RED_TEXT(info.c_str())
				<< std::endl;
#endif
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
	@brief bayer pattern type (same as Nvidia NPP setting)
	*/
	enum class GenCamBayerPattern {
		BayerBGGR = 0,
		BayerRGGB = 1,
		BayerGBRG = 2,
		BayerGRBG = 3
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
		GenCamBayerPattern bayerPattern;	
		float redGain;
		float greenGain;
		float blueGain;
		// raw wb type
		// true: raw data is after white balance
		// false: raw data is before white balance
		bool isWBRaw;
	};

	/**
	@brief capture mode 
	*/
	enum class GenCamCaptureMode {
		Single, // capture one by one without buffer
		Continous, // capture and buffer images
		SingleTrigger,
		ContinousTrigger
	};

	/**
	@brief capture purpose
	*/
	enum class GenCamCapturePurpose {
		Streaming, // capture images to buffers circularly  
		Recording // capture images to fill the buffer once
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
		uchar* data; // jpeg data pointer
		size_t maxLength; // max malloced memory size
		size_t length; // jpeg data length
	};

	/**
	@brief generic camera class
	*/
	class GenCamera {
	protected:
		// camera model
		CameraModel camModel;

		// capture for real-time view or saving
		GenCamCapturePurpose camPurpose;
		std::vector<GenCamInfo> camInfos;

		// camera status
		bool isInit;
		bool isCapture;
		bool isVerbose; // enable log in capturing images

		// camera buffer
		GenCamBufferType bufferType;
		std::vector<std::vector<cv::Mat>> bufferImgs;
		std::vector<std::vector<JPEGdata>> bufferJPEGImgs;
		std::vector<uchar*> bufferImgs_cuda;
		int bufferSize;
		size_t cameraNum;

		// capture model
		GenCamCaptureMode captureMode;

		// threads to capture images
		std::vector<std::thread> ths; 
		// thread to compress raw image into jpeg
		std::thread thJPEG;

		// status of capturing threads
		// 0: stop capturing images, exit
		// 1: capturing images
		// 2: compress images use jpeg
		std::vector<int> thStatus; 

		// frame indices in buffer
		std::vector<int> thBufferInds; 

		// npp jpeg coder class
		int JPEGQuality;
		float sizeRatio;
		std::vector<npp::NPPJpegCoder> coders;

		// image mapping vector in capturing function
		std::vector<size_t> mappingVector;

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

		/**
		@brief get camera model
		@return 
		*/
		CameraModel getCamModel();

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
		@return int
		*/
		virtual int setAutoWhiteBalance(int camInd) = 0;

		/**
		@brief set auto white balance
		@param int ind: index of camera (-1 means all the cameras)
		@param float redGain: red gain of the white balance
		@param float greenGain: green gain of the white balance
		@param float blueGain: blue gain of the white balance
		@return int
		*/
		virtual int setWhiteBalance(int camInd, float redGain,
			float greenGain, float blueGain) = 0;

		/**
		@brief set auto exposure
		@param int ind: index of camera (-1 means all the cameras)
		@param Status autoExposure: if use auto exposure
		@return int
		*/
		virtual int setAutoExposure(int camInd, Status autoExposure) = 0;

		/**
		@brief set auto exposure level (only support XIMEA cameras)
		@param int ind: index of camera (-1 means all the cameras)
		@param float level: auto exposure level, average intensity of output
		signal AEAG should achieve
		@return int
		*/
		virtual int setAutoExposureLevel(int camInd, float level) = 0;

		/**
		@brief set auto exposure compensation (only support PointGrey cameras)
		@param int ind: index of camera (-1 means all the cameras)
		@param Status status: if use auto EV value
		@param float relativeEV: only valid when the second argument is off.
		The reason why use relative EV value here is to directly set a absolute 
		value is difficult
		@return int
		*/
		virtual int setAutoExposureCompensation(int camInd, 
			Status status, float relativeEV) = 0;

		/**
		@brief set exposure time
		@param int camInd: index of camera (-1 means all the cameras)
		@param int time: exposure time (in microseconds)
		@return int
		*/
		virtual int setExposure(int camInd, int time) = 0;

		/**
		@brief set/get bayer pattern
		@param int camInd: input camera index
		@param GenCamBayerPattern & bayerPattern: output bayer pattern
		@return int
		*/
		virtual int getBayerPattern(int camInd, GenCamBayerPattern & bayerPattern) = 0;

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

		/*************************************************************/
		/*                   non-virtual function                    */
		/*************************************************************/
		/**
		@brief set verbose 
		@param bool isVerbose: true, verbose mode, output many infomations
		for debugging
		@return int
		*/
		int setVerbose(bool isVerbose);

		/**
		@brief set buffer type
		@param GenCamBufferType type: buffer type
		@return int
		*/
		int setCamBufferType(GenCamBufferType type);

		/**
		@brief set jpeg compression quality
		@param int quality: JPEG compression quality (1 - 100)
		@param float sizeRatio: expected compression ratio used for 
			pre-malloc memory, too small will cause npp jpeg coder crash
			(default 0.2)
		@return int
		*/
		int setJPEGQuality(int quality, float sizeRatio = 0.2);

		/**
		@brief multi-thread capturing function (raw buffer)
		used for continous mode
		thread function to get images from camera and buffer to vector
		and wait until the next frame (based on fps)
		@param int camInd: index of camera
		*/
		void capture_thread_raw_(int camInd);

		/**
		@brief multi-thread captureing function
		used for single mode
		thread function to get images from camera and buffer to vector
		@param int camInd: index of camera
		@param cv::Mat & img: output captured image
		*/
		void capture_thread_single_(int camInd, cv::Mat & img);

		/**
		@brief multi-thread capturing function (jpeg buffer)
		used for continous mode
		thread function to get images from camera and wait for compresss
		thread to compress the raw data into jpeg data
		@param int camInd: index of camera
		*/
		void capture_thread_JPEG_(int camInd);

		/**
		@brief single-thread compressing function
		because npp only support single thread, jpeg compress function is not 
		thread safe
		thread function to compress raw image into jpeg data
		and wait until the next frame (based on fps)
		*/
		void compress_thread_JPEG_();

		/**
		@brief set capturing mode
		@param GenCamCaptureMode captureMode: capture mode
		@param int size: buffer size
		@return int
		*/
		int setCaptureMode(GenCamCaptureMode captureMode,
			int bufferSize);

		/**
		@brief set capture purpose
		@param GenCamCapturePurpose camPurpose: purpose, for streaming or recording
		@return int
		*/
		int setCapturePurpose(GenCamCapturePurpose camPurpose);

		/**
		@brief wait for recording threads to finish
		@return int
		*/
		int waitForRecordFinish();

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

		/*************************************************************/
		/*        function to save capture images to files           */
		/*************************************************************/
		/**
		@brief save captured images to dir
		@param std::string dir: input dir to save images
		@return int
		*/
		int saveImages(std::string dir);

		/**
		@brief save captured videos to dir
		@param std::string dir: input dir to save videos
		@return int
		*/
		int saveVideos(std::string dir);

		/*************************************************************/
		/*    function to set mapping vector of capture function     */
		/*************************************************************/
		/**
		@brief set mapping vector of capture function
		@param std::vector<size_t> mappingVector: input mapping vector
		@return int
		*/
		int setMappingVector(std::vector<size_t> mappingVector);

		/**
		@brief capture one frame with Mapping
		@param std::vector<cv::Mat> & imgs: output captured images
		if in single mode, memory of image mats should be malloced
		before using this function
		@return int
		*/
		int captureFrameWithMapping(std::vector<cv::Mat> & imgs);

	};

	/**
	@breif function to init camera array
	@return 
	*/
	std::shared_ptr<GenCamera> createCamera(CameraModel camModel);
};



#endif