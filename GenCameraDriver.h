/**
@brief Generic Camera Driver Class
@author Shane Yuan
@date Dec 29, 2017
*/

#ifndef __GENERIC_CAMERA_DRIVER_H__
#define __GENERIC_CAMERA_DRIVER_H__

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
		XIMEA_xiC = 0,
		PointGrey_u3 = 1,
		Network = 2,
		File = 3
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
		Raw,  // save 8-bit raw images in buffer
		JPEG, // save jpeg compressed images in buffer
			  // usually need as power GPU to compress the raw images
		RGB24,   // save demosaiced 3 channel RGB images in buffer 
		Raw16 // save 16-bit raw images in buffer
	};

	
	/**
	@brief class to save JPEG data
	*/
	struct Imagedata {
		char* data; // data pointer
		GenCamBufferType type; // buffer data type
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
		std::vector<std::vector<Imagedata>> bufferImgs;
		int bufferSize;
		size_t cameraNum;

		// capture model
		GenCamCaptureMode captureMode;

		// frame indices in buffer
		std::vector<int> thBufferInds; 

		// npp jpeg coder class
		int JPEGQuality;
		float sizeRatio;

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
		@brief let cameras start capturing images
		@return int
		*/
		virtual int startCapture() = 0;

		/**
		@brief let cameras stop capturing images
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
		@param float exposureUpperLimitRatio: exposure upper limit time, make
			exposure upper limit time = 1000000us / fps * 0.8
		@return int
		*/
		virtual int setFPS(int camInd, float fps, float exposureUpperLimitRatio = 0.8) = 0;

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
		@brief set capturing mode
		@param GenCamCaptureMode captureMode: capture mode
		@param int size: buffer size
		@return int
		*/
		virtual int setCaptureMode(GenCamCaptureMode captureMode,
			int bufferSize) = 0;

		/**
		@brief wait for recording threads to finish
		@return int
		*/
		virtual int waitForRecordFinish() = 0;

		/**
		@brief start capturing threads
		capturing threads captures images from cameras, and buffer to
		bufferImgs vector, if buffer type is jpeg, this function will start
		a thread to compress captured images into buffer vector
		@return int 
		*/
		virtual int startCaptureThreads() = 0;

		/**
		@brief stop capturing threads
		@return int
		*/
		virtual int stopCaptureThreads() = 0;

		/*************************************************************/
		/*                   non-virtual setting function                    */
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
		@brief set capture purpose
		@param GenCamCapturePurpose camPurpose: purpose, for streaming or recording
		@return int
		*/
		int setCapturePurpose(GenCamCapturePurpose camPurpose);

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
		@brief capture one frame
		@param std::vector<Imagedata> & imgs: output captured images
		if in single mode, memory of image mats should be malloced 
		before using this function
		@return int
		*/
		int captureFrame(std::vector<Imagedata> & imgs);

		/**
		@brief capture one frame with mapping
		@param std::vector<Imagedata> & imgs: output captured images
		if in single mode, memory of image mats should be malloced
		before using this function
		@return int
		*/
		int captureFrameWithMapping(std::vector<Imagedata> & imgs);

		/**
		@brief get camera infos list with mapping
		@param std::vector<cam::GenCamInfo> & camInfos: output camera info list
		@return int
		*/
		int getCameraInfoListsWithMapping(std::vector<cam::GenCamInfo> & camInfos);

	};

	/**
	@breif function to init camera array
	@return 
	*/
	std::shared_ptr<GenCamera> createCamera(CameraModel camModel);
};



#endif