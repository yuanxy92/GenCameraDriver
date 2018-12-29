/**
@brief Generic Camera Driver Class
Implementation of Stereo camera
@author zhu-ty
@date Jul 25, 2018
*/

#ifndef __GENERIC_CAMERA_DRIVER_STEREO_H__
#define __GENERIC_CAMERA_DRIVER_STEREO_H__

#ifdef WIN32
#include <windows.h>
#endif
#include <memory.h>

#include "GenCameraDriver.h"
#include "RealCameraDriver.h"
#include "Stereo/StereoRectify.h"
#include "ExposureFusion.h"
#include "DisparityProcessor.h"

#ifndef WIN32
#include "DepthMapUpdater.h"
#else
#define JIANING_WIDTH 1000
#define JIANING_HEIGHT 750
#endif

#define EXPOSURE_DIFF 0

#define DEPTH_MAP_WIDTH JIANING_WIDTH
#define DEPTH_MAP_HEIGHT JIANING_HEIGHT


namespace cam {


	/*Arrange:
	MIX_CAMERA_0
	MIX_CAMERA_1
	...
	DEPTH_CAMERA_0
	DEPTH_CAMERA_1
	...
	*/

	class GenCameraStereo : public RealCamera {
	private:
		struct SubCamera
		{
			// The SN for real camera
			std::string sn;
			// The index for the real camera in sub_cameraPtr
			int index;
			// The Cpu mat for image (raw)
			cv::Mat cpu_raw_img;
			// The Gpu mat for image
			cv::cuda::GpuMat gpu_raw_img;
			cv::cuda::GpuMat gpu_rgb_img;
			cv::cuda::GpuMat gpu_rec_img;
			//cv::cuda::GpuMat gpu_remap_img;
			
			Npp32f wbTwist[3][4] = {
				{ 1.0, 0.0, 0.0, 0.0 },
				{ 0.0, 1.0, 0.0, 0.0 },
				{ 0.0, 0.0, 1.0, 0.0 }
			};
			SubCamera& operator =(const SubCamera& another)
			{
				this->sn = another.sn;
				this->index = another.index;
				this->cpu_raw_img = another.cpu_raw_img;
				this->gpu_raw_img = another.gpu_raw_img;
				this->gpu_rgb_img = another.gpu_rgb_img;
				this->gpu_rec_img = another.gpu_rec_img;
				return *this;
			}
			int& operator=(const int& infoIDX)
			{
				this->index = infoIDX;
				return this->index;
			}
			std::string& operator=(const std::string& infoSTR)
			{
				this->sn = infoSTR;
				return this->sn;
			}
		};
		struct StereoPair
		{
			bool isFirstPairCaptured = false;
			SubCamera master, slave;
			std::string int_path;
			std::string ext_path;
			cv::Mat depth_img,disparity_img;
			cv::cuda::GpuMat _gpu_depth_img,_gpu_disparity_img;
#ifndef WIN32
			DepthMapUpdater dUpdater;
#endif
			/*
			CameraMatrixK = (make sure that b/d stands for the half height/width of the image)
			[
				a 0 width/2
				0 c height/2
				0 0 1
			]
			CameraMatrixK^-1=
			[
				A 0 B
				0 C D
				0 0 1
			]
			DepthCoef = (E)
			MaxDepth = 50000 (definded)

			Mat Ki = [A,B,C,D,E]
			*/
			cv::Mat Ki;
			cv::cuda::GpuMat _gpu_Ki;

			bool inv; // Tell whether we have to change the order to do the rectify
			SubCamera& operator[](int i)
			{
				return (i == 0) ? master : slave;
			}
			StereoRectify sr;
		};
	private:
		std::string config_file_path;
		CameraModel sub_model;
		std::vector<cam::GenCamInfo> sub_camInfos;
		std::shared_ptr<cam::GenCamera> sub_cameraPtr;
		std::vector<StereoPair> pair_infos;
		std::vector<Imagedata> raw_imgs;

	private:
		int load_config(std::string path);

		int search_camera(std::string sn,std::vector<GenCamInfo> list);

		int search_pair(std::string sn, std::vector<StereoPair> list);

		int process_disparity_with_mask(cv::cuda::GpuMat &disparity, cv::cuda::GpuMat &Ki, cv::cuda::GpuMat &depth);

	public:
		GenCameraStereo();
		~GenCameraStereo();

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
		@param float exposureUpperLimitRatio: exposure upper limit time, make
			exposure upper limit time = 1000000us / fps * 0.8
		@return int
		*/
		int setFPS(int camInd, float fps, float exposureUpperLimitRatio = 0.8) override;

		/**
		@brief set auto white balance
		@param int ind: index of camera (-1 means all the cameras)
		@return int
		*/
		int setAutoWhiteBalance(int camInd) override;

		/**
		@brief set auto white balance
		@param int ind: index of camera (-1 means all the cameras)
		@param float redGain: red gain of the white balance
		@param float greenGain: green gain of the white balance
		@param float blueGain: blue gain of the white balance
		@return int
		*/
		int setWhiteBalance(int camInd, float redGain,
			float greenGain, float blueGain) override;

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
		@brief set auto exposure compensation (only support PointGrey cameras)
		@param int ind: index of camera (-1 means all the cameras)
		@param Status status: if use auto EV value
		@param float relativeEV: only valid when the second argument is off.
		The reason why use relative EV value here is to directly set a absolute
		value is difficult
		@return int
		*/
		int setAutoExposureCompensation(int camInd,
			Status status, float relativeEV) override;

		/**
		@brief set brightness time
		@param int camInd: index of camera
		@param int brightness: input brightness
			+1: brighten, -1: darken, 0: do nothing
		@return int
		*/
		int adjustBrightness(int camInd, int brightness) override;

		/**
		@brief set exposure time
		@param int ind: index of camera (-1 means all the cameras)
		@param int time: exposure time (in microseconds)
		@return int
		*/
		int setExposure(int camInd, int time) override;

		/**
		@brief set/get bayer pattern
		@param int camInd: input camera index
		@param GenCamBayerPattern & bayerPattern: output bayer pattern
		@return int
		*/
		int getBayerPattern(int camInd, GenCamBayerPattern & bayerPattern) override;

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
		@brief capture single image of single camera in camera array
		@param int camInd: input index of camera
		@param Imagedata & img: output captured images (pre-allocated memory)
		@return int
		*/
		int captureFrame(int camInd, Imagedata & img) override;


	};

};

#endif //__GENERIC_CAMERA_DRIVER_STEREO_H__
