/**
@brief Generic Camera Driver Class
Implementation of Stereo camera
@author zhu-ty
@date 2018
*/

#include "StereoCamera.h"
#include "INIReader.h"
#include <npp.h>
#include "Exceptions.h"
//#include <nppi_compression_functions.h>
//#include <cuda_runtime.h>
//
//#include <opencv2/opencv.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/cudaimgproc.hpp>
//#include <opencv2/core/cuda_stream_accessor.hpp>
//
//#include "helper_string.h"
#include "helper_cuda.h"

namespace cam {

	int GenCameraStereo::load_config(std::string path)
	{
		INIReader reader(path);
		if (reader.ParseError() < 0)
		{
			SysUtil::errorOutput(std::string("GenCameraStereo::load_config Can't load :") + path);
			return -1;
		}
		int stereo_count = reader.GetInteger("Global", "stereo_count", 0);
		sub_cameraBufferCount = reader.GetInteger("Global", "buffer_count", 200);
		fileCameraPath = reader.Get("Global", "file_camera_path", ".");
		std::string camModel = reader.Get("Global", "camera_model", "XIMEA");
		if (SysUtil::toLower(camModel) == "ximea" || SysUtil::toLower(camModel) == "x")
			this->sub_model = cam::CameraModel::XIMEA_xiC;
		else if (SysUtil::toLower(camModel) == "file" || SysUtil::toLower(camModel) == "f")
			this->sub_model = cam::CameraModel::File;
		else if (SysUtil::toLower(camModel) == "ptgrey" || SysUtil::toLower(camModel) == "pointgrey" || SysUtil::toLower(camModel) == "p")
			this->sub_model = cam::CameraModel::PointGrey_u3;
		else
			this->sub_model = cam::CameraModel::XIMEA_xiC;
		for (int i = 0; i < stereo_count; i++)
		{
			std::string stridx = cv::format("Stereo%d", i);
			StereoPair sp;
			sp[0] = reader.Get(stridx, "master_sn", "default_master_sn");
			sp[1] = reader.Get(stridx, "slave_sn", "default_slave_sn");
			sp.int_path = reader.Get(stridx, "int_path", "default_int_path");
			sp.ext_path = reader.Get(stridx, "ext_path", "default_ext_path");
			//sp.warp_x_path = reader.Get(stridx, "warp_x_path", "default_warp_x_path");
			//sp.warp_y_path = reader.Get(stridx, "warp_y_path", "default_warp_y_path");
			sp.inv = reader.GetBoolean(stridx, "inv", false);
			std::string strmp,strsp,strdp;
			cv::Mat matmp, matsp, matdp;
			strmp = reader.Get(stridx, "master_color", "./master.jpg");
			strsp = reader.Get(stridx, "slave_color", "./slave.jpg");
			strdp = reader.Get(stridx, "depth", "./depth.jpg");
			matmp = cv::imread(strmp);
			matsp = cv::imread(strsp);
			matdp =	cv::imread(strdp,cv::IMREAD_UNCHANGED);
			//SysUtil::infoOutput("dUpdater before init done");
#ifndef WIN32
			sp.dUpdater.init(matmp, matsp, matdp);
#endif
			
			//SysUtil::infoOutput("dUpdater init done");

			double a,b,c,d,E,A,B,C,D,K;
			a = reader.GetReal(stridx, "a", 1000.0);
			b = reader.GetReal(stridx, "b", 1000.0);
			c = reader.GetReal(stridx, "c", 1000.0);
			d = reader.GetReal(stridx, "d", 1000.0);
			E = reader.GetReal(stridx, "E", 1000.0);

			E = E / ((float)matmp.cols / (float)JIANING_WIDTH); //big depth to small depth
			// SysUtil::infoOutput(cv::format("E = %f", E));
			// getchar();


			K = JIANING_WIDTH / (2 * b);
			A = 1 / (K * a);
			B = - b / a;
			C = 1 / (K * c);
			D = - c / d;
			sp.Ki.create(1, 5, CV_32FC1);
			//SysUtil::infoOutput("Create Ki CPU done");
			sp.Ki.at<float>(0, 0) = A;
			sp.Ki.at<float>(0, 1) = B;
			sp.Ki.at<float>(0, 2) = C;
			sp.Ki.at<float>(0, 3) = D;
			sp.Ki.at<float>(0, 4) = E;
			sp._gpu_Ki.upload(sp.Ki);
			//SysUtil::infoOutput("upload Ki GPU done");

			this->pair_infos.push_back(sp);
		}
		return 0;
	}

	int GenCameraStereo::search_camera(std::string sn, std::vector<GenCamInfo> list)
	{
		for (int i = 0; i < list.size(); i++)
		{
			if (list[i] == sn)
			{
				return i;
			}
		}
		return -1;
	}

	int GenCameraStereo::search_pair(std::string sn, std::vector<StereoPair> list)
	{
		for (int i = 0; i < list.size(); i++)
		{
			if (list[i][0].sn == sn || list[i][1].sn == sn )
			{
				return i;
			}
		}
		return -1;
	}


	GenCameraStereo::GenCameraStereo() {
		this->camModel = CameraModel::Stereo;
	}

	GenCameraStereo::~GenCameraStereo() {}

	int GenCameraStereo::init() {
		// check 
		if (this->isInit == true) {
			SysUtil::warningOutput("GenCameraStereo is already initialized. Please do not init twice !");
			return 0;
		}
		//TODO : How do we input the config file path?
		config_file_path = "./StereoConfig.ini";
		load_config(this->config_file_path);
		SysUtil::infoOutput(cv::format("GenCameraStereo::init %d Stereo cameras detected", pair_infos.size()));
		if (pair_infos.size() == 0)
		{
			SysUtil::errorOutput("GenCameraStereo::init failed! 0 pairs detected!");
			return -1;
		}

		
		//TODO : Now can only open every camera
		sub_cameraPtr = cam::createCamera(this->sub_model, fileCameraPath);
		sub_cameraPtr->init();
		sub_cameraPtr->getCamInfos(sub_camInfos);
		sub_cameraPtr->setCamBufferType(GenCamBufferType::Raw);
		sub_cameraPtr->setCaptureMode(GenCamCaptureMode::Continous, sub_cameraBufferCount);
		sub_cameraPtr->setCapturePurpose(GenCamCapturePurpose::Streaming);
		if (sub_cameraPtr->getCamModelString() == "   XIMEA_xiC" && cam::SysUtil::existFile("./mul_mat.tiff"))
		{
			cv::Mat mul = cv::imread("./mul_mat.tiff", cv::IMREAD_UNCHANGED);
			_gpu_sub_camera_brightness_adjustment.upload(mul);
		}
		
		raw_imgs.resize(pair_infos.size() * 2);
		for (int i = 0; i < pair_infos.size(); i++)
		{
			//Searching Master
			int idx = search_camera(pair_infos[i][0].sn, sub_camInfos);
			if (idx == -1)
			{
				SysUtil::errorOutput("GenCameraStereo::init failed! master sn not found! Searching for " + pair_infos[i][0].sn);
				return -1;
			}
			pair_infos[i][0] = idx;
			pair_infos[i][0].wbTwist[0][0] = sub_camInfos[idx].redGain;
			pair_infos[i][0].wbTwist[1][1] = sub_camInfos[idx].greenGain;
			pair_infos[i][0].wbTwist[2][2] = sub_camInfos[idx].blueGain;
			//Searching Slave
			idx = search_camera(pair_infos[i][1].sn, sub_camInfos);
			if (idx == -1)
			{
				SysUtil::errorOutput("GenCameraStereo::init failed! slave sn not found! Searching for " + pair_infos[i][1].sn);
				return -1;
			}
			pair_infos[i][1] = idx;
			pair_infos[i][1].wbTwist[0][0] = sub_camInfos[idx].redGain;
			pair_infos[i][1].wbTwist[1][1] = sub_camInfos[idx].greenGain;
			pair_infos[i][1].wbTwist[2][2] = sub_camInfos[idx].blueGain;

			//fusion bool
			//pair_infos[i].isFusionInit = false;

			//prepare warping mat
			//pair_infos[i].warp_x = cv::imread(pair_infos[i].warp_x_path, cv::IMREAD_ANYDEPTH);
			//pair_infos[i].warp_x.convertTo(pair_infos[i].warp_x, CV_32FC1);
			//pair_infos[i].warp_y = cv::imread(pair_infos[i].warp_y_path, cv::IMREAD_ANYDEPTH);
			//pair_infos[i].warp_y.convertTo(pair_infos[i].warp_y, CV_32FC1);
			//pair_infos[i].gpu_warp_x.upload(pair_infos[i].warp_x);
			//pair_infos[i].gpu_warp_y.upload(pair_infos[i].warp_y);

			//prepare rectify class
			pair_infos[i].sr.init(pair_infos[i].int_path, pair_infos[i].ext_path, cv::Size(sub_camInfos[idx].width, sub_camInfos[idx].height));
			//SysUtil::infoOutput("pair_infos[i].int_path = " + pair_infos[i].int_path);

			//prepare fusion filter kernal
			//pair_infos[i].ef.Raman_init_filter(sub_camInfos[idx].height);

			//prepare raw img space
			size_t length = sizeof(uchar) * sub_camInfos[idx].width * sub_camInfos[idx].height;
			raw_imgs[i * 2].data = new char[length];
			raw_imgs[i * 2].length = length;
			raw_imgs[i * 2].maxLength = length;
			raw_imgs[i * 2 + 1].data = new char[length];
			raw_imgs[i * 2 + 1].length = length;
			raw_imgs[i * 2 + 1].maxLength = length;

			//prepare capture space
			pair_infos[i][0].cpu_raw_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8U);
			pair_infos[i][1].cpu_raw_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8U);

			pair_infos[i][0].gpu_raw_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8U);
			pair_infos[i][1].gpu_raw_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8U);

			pair_infos[i][0].gpu_rgb_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8UC3);
			pair_infos[i][1].gpu_rgb_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8UC3);

			pair_infos[i][0].gpu_rec_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8UC3);
			pair_infos[i][1].gpu_rec_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8UC3);

			pair_infos[i][0].gpu_final_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8UC3);
			pair_infos[i][1].gpu_final_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8UC3);

			//pair_infos[i][0].gpu_remap_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8UC3);
			//pair_infos[i][1].gpu_remap_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8UC3);
			
		}
		cameraNum = pair_infos.size() * 2; //Now added a EXTRA camera for each pair (depth & mask)
		ths.resize(cameraNum);
		thStatus.resize(cameraNum);
		isInit = true;
		//isFusionInit = false;
		// init image ratio vector
		imgRatios.resize(cameraNum);
		for (size_t i = 0; i < cameraNum; i++) {
			imgRatios[i] = GenCamImgRatio::Full;
		}

		//Tell the real camera driver that we are different
		this->isCapturedFrameGpuPointer = true;
		this->isCapturedFrameDebayered = true;

		return 0;
	}

	int GenCameraStereo::startCapture() {
		this->sub_cameraPtr->startCapture();
		this->sub_cameraPtr->startCaptureThreads();
		this->isCapture = true;
		return 0;
	}

	int GenCameraStereo::stopCapture() {
		this->sub_cameraPtr->stopCaptureThreads();
		this->sub_cameraPtr->stopCapture();
		this->isCapture = false;
		return 0;
	}

	int GenCameraStereo::release() {
		if (this->isCapture == true)
		{
			this->stopCapture();
			//this->sub_cameraPtr->stopCapture();
		}
		// close cameras
		this->sub_cameraPtr->release();
		if (this->isInit)
		{
			for (int i = 0; i < pair_infos.size() * 2; i++)
			{
				//delete this->raw_imgs[i].data;
			}
		}
		return 0;
	}

	int GenCameraStereo::getCamInfos(std::vector<GenCamInfo> & camInfos) {
		camInfos.resize(cameraNum);
		sub_cameraPtr->getCamInfos(sub_camInfos);
		//each pair info = master.info
		for (size_t i = 0; i < this->cameraNum / 2; i++)
		{
			camInfos[i] = sub_camInfos[pair_infos[i][0].index];
			camInfos[i].sn = "MIX_" + sub_camInfos[pair_infos[i][0].index].sn + "_" + sub_camInfos[pair_infos[i][1].index].sn;
			camInfos[i].isWBRaw = true;
			//set twist
			for (int j = 0; j < 2; j++)
			{
				pair_infos[i][j].wbTwist[0][0] = sub_camInfos[pair_infos[i][j].index].redGain;
				pair_infos[i][j].wbTwist[1][1] = sub_camInfos[pair_infos[i][j].index].greenGain;
				pair_infos[i][j].wbTwist[2][2] = sub_camInfos[pair_infos[i][j].index].blueGain;
			}
		}
		for (size_t i = this->cameraNum / 2; i < this->cameraNum; i++)
		{
			GenCamInfo tmp_info = sub_camInfos[pair_infos[i - this->cameraNum / 2][1].index];
			tmp_info.height = DEPTH_MAP_HEIGHT;
			tmp_info.width = DEPTH_MAP_WIDTH;
			tmp_info.isWBRaw = true;
			tmp_info.sn = "RAW_DEPTH_MIX_" + 
				sub_camInfos[pair_infos[i - this->cameraNum / 2][0].index].sn + 
				"_" + sub_camInfos[pair_infos[i - this->cameraNum / 2][1].index].sn;
			camInfos[i] = tmp_info;
		}
		return 0;
	}

	int GenCameraStereo::setFPS(int camInd, float fps, float exposureUpperLimitRatio) {
		if (camInd == -1) 
		{
			this->sub_cameraPtr->setFPS(camInd, fps, exposureUpperLimitRatio);
		}
		else if(camInd < this->cameraNum / 2)
		{
			//TODO : now will affect 2 real cameras
			this->sub_cameraPtr->setFPS(pair_infos[camInd][0].index, fps, exposureUpperLimitRatio);
			this->sub_cameraPtr->setFPS(pair_infos[camInd][1].index, fps, exposureUpperLimitRatio);
		}
		return 0;
	}

	int GenCameraStereo::setAutoWhiteBalance(int camInd) {
		if (camInd == -1)
		{
			this->sub_cameraPtr->setAutoWhiteBalance(camInd);
		}
		else if (camInd < this->cameraNum / 2)
		{
			//TODO : now will affect 2 real cameras
			this->sub_cameraPtr->setAutoWhiteBalance(pair_infos[camInd][0].index);
			this->sub_cameraPtr->setAutoWhiteBalance(pair_infos[camInd][1].index);
		}
		return 0;
	}

	int GenCameraStereo::setWhiteBalance(int camInd, float redGain,
		float greenGain, float blueGain) {
		if (camInd == -1)
		{
			this->sub_cameraPtr->setWhiteBalance(camInd, redGain, greenGain, blueGain);
		}
		else if (camInd < this->cameraNum / 2)
		{
			//TODO : now will affect 2 real cameras
			this->sub_cameraPtr->setWhiteBalance(pair_infos[camInd][0].index, redGain, greenGain, blueGain);
			this->sub_cameraPtr->setWhiteBalance(pair_infos[camInd][1].index, redGain, greenGain, blueGain);
		}
		return 0;
	}

	int GenCameraStereo::setAutoExposure(int camInd, Status autoExposure) {
		if (camInd == -1)
		{
			this->sub_cameraPtr->setAutoExposure(camInd, autoExposure);
		}
		else if (camInd < this->cameraNum / 2)
		{
			//TODO : now will affect 2 real cameras
			this->sub_cameraPtr->setAutoExposure(pair_infos[camInd][0].index, autoExposure);
			this->sub_cameraPtr->setAutoExposure(pair_infos[camInd][1].index, autoExposure);
		}
		return 0;
	}

	int GenCameraStereo::setAutoExposureLevel(int camInd, float level) {
		if (camInd == -1)
		{
			//this->sub_cameraPtr->setAutoExposureLevel(camInd, level);
			for (int i = 0; i < this->cameraNum / 2; i++)
			{
				this->sub_cameraPtr->setAutoExposureLevel(pair_infos[i][0].index, level + EXPOSURE_DIFF);
				this->sub_cameraPtr->setAutoExposureLevel(pair_infos[i][1].index, level - EXPOSURE_DIFF);
				//pair_infos[i].isFusionInit = false;
			}
		}
		else if (camInd < this->cameraNum / 2)
		{
			//TODO : now will affect 2 real cameras
			this->sub_cameraPtr->setAutoExposureLevel(pair_infos[camInd][0].index, level + EXPOSURE_DIFF);
			this->sub_cameraPtr->setAutoExposureLevel(pair_infos[camInd][1].index, level - EXPOSURE_DIFF);
			//pair_infos[camInd].isFusionInit = false;
		}
		return 0;
	}

	int GenCameraStereo::setAutoExposureCompensation(int camInd,
		Status status, float relativeEV) {
		if (camInd == -1)
		{
			this->sub_cameraPtr->setAutoExposureCompensation(camInd, status, relativeEV);
		}
		else if (camInd < this->cameraNum / 2)
		{
			//TODO : now will affect 2 real cameras
			this->sub_cameraPtr->setAutoExposureCompensation(pair_infos[camInd][0].index, status, relativeEV);
			this->sub_cameraPtr->setAutoExposureCompensation(pair_infos[camInd][1].index, status, relativeEV);
		}
		return 0;
	}

	int GenCameraStereo::adjustBrightness(int camInd, int brightness) {
		if (camInd == -1)
		{
			this->sub_cameraPtr->adjustBrightness(camInd, brightness);
		}
		else if (camInd < this->cameraNum / 2)
		{
			//TODO : now will affect 2 real cameras
			this->sub_cameraPtr->adjustBrightness(pair_infos[camInd][0].index, brightness);
			this->sub_cameraPtr->adjustBrightness(pair_infos[camInd][1].index, brightness);
		}
		return 0;
	}

	int GenCameraStereo::setExposure(int camInd, int time) {
		if (camInd == -1)
		{
			this->sub_cameraPtr->setExposure(camInd, time);
		}
		else if (camInd < this->cameraNum / 2)
		{
			//TODO : now will affect 2 real cameras
			this->sub_cameraPtr->setExposure(pair_infos[camInd][0].index, time);
			this->sub_cameraPtr->setExposure(pair_infos[camInd][1].index, time);
		}
		return 0;
	}

	int GenCameraStereo::getBayerPattern(int camInd, GenCamBayerPattern & bayerPattern) {
		if (camInd < this->cameraNum / 2)
			this->sub_cameraPtr->getBayerPattern(pair_infos[camInd][0].index, bayerPattern);
		return 0;
	}

	int GenCameraStereo::makeSetEffective(int k) {
		this->sub_cameraPtr->makeSetEffective(k);
		return 0;
	}

	int GenCameraStereo::captureFrame(int camInd, Imagedata & img) 
	{
		//SysUtil::infoOutput(cv::format("GenCameraStereo::captureFrame camInd = %d", camInd));
		if (camInd < this->cameraNum / 2)
		{
			sub_cameraPtr->captureFrame(raw_imgs);
			int cols = sub_camInfos[pair_infos[camInd][0].index].width;
			int rows = sub_camInfos[pair_infos[camInd][0].index].height;

			for(int i = 0;i < 2;i++)
			{
				pair_infos[camInd][i].cpu_raw_img.data = reinterpret_cast<unsigned char *>(raw_imgs[pair_infos[camInd][i].index].data);
				pair_infos[camInd][i].gpu_raw_img.upload(pair_infos[camInd][i].cpu_raw_img);

				if(!_gpu_sub_camera_brightness_adjustment.empty())
				{
					cv::cuda::GpuMat tmp, tmp2;
					pair_infos[camInd][i].gpu_raw_img.convertTo(tmp, CV_32F);
					cv::cuda::multiply(tmp, this->_gpu_sub_camera_brightness_adjustment, tmp2);
					tmp2.convertTo(pair_infos[camInd][i].gpu_raw_img, CV_8U);
				}

				cv::cuda::demosaicing(pair_infos[camInd][i].gpu_raw_img, pair_infos[camInd][i].gpu_rgb_img,
					npp::bayerPatternNPP2CVRGB(static_cast<NppiBayerGridPosition>(
						static_cast<int>(camInfos[camInd].bayerPattern))));
			}

			// pair_infos[camInd][0].cpu_raw_img.data = reinterpret_cast<unsigned char *>(raw_imgs[pair_infos[camInd][0].index].data);
			// pair_infos[camInd][1].cpu_raw_img.data = reinterpret_cast<unsigned char *>(raw_imgs[pair_infos[camInd][1].index].data);

			// pair_infos[camInd][0].gpu_raw_img.upload(pair_infos[camInd][0].cpu_raw_img);
			// pair_infos[camInd][1].gpu_raw_img.upload(pair_infos[camInd][1].cpu_raw_img);

			// cv::cuda::demosaicing(pair_infos[camInd][0].gpu_raw_img, pair_infos[camInd][0].gpu_rgb_img,
			// 	npp::bayerPatternNPP2CVRGB(static_cast<NppiBayerGridPosition>(
			// 		static_cast<int>(camInfos[camInd].bayerPattern))));
			// cv::cuda::demosaicing(pair_infos[camInd][1].gpu_raw_img, pair_infos[camInd][1].gpu_rgb_img,
			// 	npp::bayerPatternNPP2CVRGB(static_cast<NppiBayerGridPosition>(
			// 		static_cast<int>(camInfos[camInd].bayerPattern))));
			//SysUtil::infoOutput("GenCameraStereo::captureFrame ready to rectify");
			if (pair_infos[camInd].inv == false)
			{
				pair_infos[camInd].sr.rectify(
					pair_infos[camInd][0].gpu_rgb_img,
					pair_infos[camInd][0].gpu_rec_img,
					pair_infos[camInd][1].gpu_rgb_img,
					pair_infos[camInd][1].gpu_rec_img);
			}
			else
			{
				pair_infos[camInd].sr.rectify(
					pair_infos[camInd][1].gpu_rgb_img,
					pair_infos[camInd][1].gpu_rec_img,
					pair_infos[camInd][0].gpu_rgb_img,
					pair_infos[camInd][0].gpu_rec_img);
			}
			//SysUtil::infoOutput("GenCameraStereo::captureFrame rectify done");
			//Twist
			for (int i = 0; i < 2; i++)
			{
				NppiSize osize;
				osize.width = sub_camInfos[pair_infos[camInd][i].index].width;
				osize.height = sub_camInfos[pair_infos[camInd][i].index].height;
				NPP_CHECK_NPP(nppiColorTwist32f_8u_C3IR(
					pair_infos[camInd][i].gpu_rec_img.data, 
					pair_infos[camInd][i].gpu_rec_img.step, 
					osize, pair_infos[camInd][i].wbTwist));
				pair_infos[camInd][i].gpu_final_img = pair_infos[camInd][i].gpu_rec_img.clone();
			}
			
			img.data = reinterpret_cast<char*>(pair_infos[camInd][0].gpu_final_img.data);
			pair_infos[camInd].isFirstPairCaptured = true;
			//SysUtil::infoOutput("camInd < this->cameraNum / 2 work well and returned");
		}
		else
		{
			StereoPair *pair = &pair_infos[camInd - cameraNum / 2];
#ifndef WIN32
			if(pair->isFirstPairCaptured == true)
			{
				pair->dUpdater.update(pair->master.gpu_final_img, pair->slave.gpu_final_img, pair->disparity_img);

// #ifdef OUTPUT_MIDIAN_RESULAT
// 				//cv::Mat dis_16;
// 				//pair->disparity_img.convertTo(dis_16, CV_16UC1);
// 				cv::imwrite(cv::format("disparity_%d.tiff",pair->dUpdater.getFrameCount()),pair->disparity_img);
// #endif
				//std::cout << pair->disparity_img <<std::endl;

				pair->_gpu_disparity_img.upload(pair->disparity_img);
				//SysUtil::infoOutput("process dis start");
				DisparityProcessor::process_disparity_with_mask(pair->_gpu_disparity_img, pair->_gpu_Ki, pair->_gpu_depth_img);
				//SysUtil::infoOutput("process dis done");
				pair->_gpu_depth_img.download(pair->depth_img);
			}
			else
#endif
			{
				cv::Mat depth_temp;
				depth_temp.create(DEPTH_MAP_HEIGHT, DEPTH_MAP_WIDTH, CV_16UC1);
				depth_temp.setTo(cv::Scalar(0));
				depth_temp.at<uint16_t>(10, 10) = 64000;
				pair->depth_img = depth_temp;
				pair->_gpu_depth_img.upload(depth_temp);
				
			}
			img.data = reinterpret_cast<char*>(pair->depth_img.data);
			img.length = DEPTH_MAP_HEIGHT * DEPTH_MAP_WIDTH * 2;
			img.maxLength = DEPTH_MAP_HEIGHT * DEPTH_MAP_WIDTH * 2;
			img.isJpegCompressd = true;
			//SysUtil::infoOutput("camInd >= this->cameraNum / 2 work well and returned");
		}
		return 0;
	}

}
