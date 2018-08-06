/**
@brief Generic Camera Driver Class
Implementation of XIMEA camera
@author Shane Yuan
@date Dec 29, 2017
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
		for (int i = 0; i < stereo_count; i++)
		{
			std::string stridx = cv::format("Stereo%d", i);
			StereoPair sp;
			sp[0] = reader.Get(stridx, "master_sn", "default_master_sn");
			sp[1] = reader.Get(stridx, "slave_sn", "default_slave_sn");
			sp.int_path = reader.Get(stridx, "int_path", "default_int_path");
			sp.ext_path = reader.Get(stridx, "ext_path", "default_ext_path");
			sp.warp_x_path = reader.Get(stridx, "warp_x_path", "default_warp_x_path");
			sp.warp_y_path = reader.Get(stridx, "warp_y_path", "default_warp_y_path");
			sp.inv = reader.GetBoolean(stridx, "inv", false);
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


		//TODO : Now only support sub XIMEA camera
		sub_model = CameraModel::XIMEA_xiC;
		//TODO : How do we input the config file path?
		config_file_path = "./StereoConfig.ini";
		//TODO : Now can only open every camera
		sub_cameraPtr = cam::createCamera(cam::CameraModel::XIMEA_xiC);
		sub_cameraPtr->init();
		sub_cameraPtr->getCamInfos(sub_camInfos);
		sub_cameraPtr->setCamBufferType(GenCamBufferType::Raw);
		sub_cameraPtr->setCaptureMode(GenCamCaptureMode::Continous, 200);
		sub_cameraPtr->setCapturePurpose(GenCamCapturePurpose::Streaming);


		load_config(this->config_file_path);
		if (pair_infos.size() == 0)
		{
			SysUtil::errorOutput("GenCameraStereo::init failed! 0 pairs detected!");
			return -1;
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
			pair_infos[i].warp_x = cv::imread(pair_infos[i].warp_x_path, cv::IMREAD_ANYDEPTH);
			pair_infos[i].warp_x.convertTo(pair_infos[i].warp_x, CV_32FC1);
			pair_infos[i].warp_y = cv::imread(pair_infos[i].warp_y_path, cv::IMREAD_ANYDEPTH);
			pair_infos[i].warp_y.convertTo(pair_infos[i].warp_y, CV_32FC1);
			pair_infos[i].gpu_warp_x.upload(pair_infos[i].warp_x);
			pair_infos[i].gpu_warp_y.upload(pair_infos[i].warp_y);

			//prepare rectify class
			pair_infos[i].sr.init(pair_infos[i].int_path, pair_infos[i].ext_path, cv::Size(sub_camInfos[idx].width, sub_camInfos[idx].height));

			//prepare fusion filter kernal
			pair_infos[i].ef.Raman_init_filter(sub_camInfos[idx].height);

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

			pair_infos[i][0].gpu_remap_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8UC3);
			pair_infos[i][1].gpu_remap_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8UC3);
			
		}
		cameraNum = pair_infos.size();
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
				delete this->raw_imgs[i].data;
			}
		}
		return 0;
	}

	int GenCameraStereo::getCamInfos(std::vector<GenCamInfo> & camInfos) {
		camInfos.resize(cameraNum);
		sub_cameraPtr->getCamInfos(sub_camInfos);
		//each pair info = master.info
		for (size_t i = 0; i < this->cameraNum; i++)
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
		return 0;
	}

	int GenCameraStereo::setFPS(int camInd, float fps, float exposureUpperLimitRatio) {
		if (camInd == -1) 
		{
			this->sub_cameraPtr->setFPS(camInd, fps, exposureUpperLimitRatio);
		}
		else 
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
		else
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
		else
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
		else
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
			for (int i = 0; i < this->cameraNum; i++)
			{
				this->sub_cameraPtr->setAutoExposureLevel(pair_infos[i][0].index, level + EXPOSURE_DIFF);
				this->sub_cameraPtr->setAutoExposureLevel(pair_infos[i][1].index, level - EXPOSURE_DIFF);
				//pair_infos[i].isFusionInit = false;
			}
		}
		else
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
		else
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
		else
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
		else
		{
			//TODO : now will affect 2 real cameras
			this->sub_cameraPtr->setExposure(pair_infos[camInd][0].index, time);
			this->sub_cameraPtr->setExposure(pair_infos[camInd][1].index, time);
		}
		return 0;
	}

	int GenCameraStereo::getBayerPattern(int camInd, GenCamBayerPattern & bayerPattern) {
		this->sub_cameraPtr->getBayerPattern(pair_infos[camInd][0].index, bayerPattern);
		return 0;
	}

	int GenCameraStereo::makeSetEffective(int k) {
		this->sub_cameraPtr->makeSetEffective(k);
		return 0;
	}

	int GenCameraStereo::captureFrame(int camInd, Imagedata & img) 
	{
		sub_cameraPtr->captureFrame(raw_imgs);
		int cols = sub_camInfos[pair_infos[camInd][0].index].width;
		int rows = sub_camInfos[pair_infos[camInd][0].index].height;
		//memcpy(pair_infos[camInd][0].cpu_raw_img.data, raw_imgs[pair_infos[camInd][0].index].data, cols * rows * sizeof(uchar));
		//memcpy(pair_infos[camInd][1].cpu_raw_img.data, raw_imgs[pair_infos[camInd][1].index].data, cols * rows * sizeof(uchar));

		pair_infos[camInd][0].cpu_raw_img.data = reinterpret_cast<unsigned char *>(raw_imgs[pair_infos[camInd][0].index].data);
		pair_infos[camInd][1].cpu_raw_img.data = reinterpret_cast<unsigned char *>(raw_imgs[pair_infos[camInd][1].index].data);

		pair_infos[camInd][0].gpu_raw_img.upload(pair_infos[camInd][0].cpu_raw_img);
		pair_infos[camInd][1].gpu_raw_img.upload(pair_infos[camInd][1].cpu_raw_img);

		cv::cuda::demosaicing(pair_infos[camInd][0].gpu_raw_img, pair_infos[camInd][0].gpu_rgb_img,
			npp::bayerPatternNPP2CVRGB(static_cast<NppiBayerGridPosition>(
				static_cast<int>(camInfos[camInd].bayerPattern))));
		cv::cuda::demosaicing(pair_infos[camInd][1].gpu_raw_img, pair_infos[camInd][1].gpu_rgb_img,
			npp::bayerPatternNPP2CVRGB(static_cast<NppiBayerGridPosition>(
				static_cast<int>(camInfos[camInd].bayerPattern))));

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

		//Twist
		for (int i = 0; i < 2; i++)
		{
			NppiSize osize;
			osize.width = sub_camInfos[pair_infos[camInd][i].index].width;
			osize.height = sub_camInfos[pair_infos[camInd][i].index].height;
			NPP_CHECK_NPP(nppiColorTwist32f_8u_C3IR(pair_infos[camInd][i].gpu_rec_img.data, pair_infos[camInd][i].gpu_rec_img.step, osize, pair_infos[camInd][i].wbTwist));
		}

		//cv::Mat tmp;
		//pair_infos[camInd][1].gpu_rec_img.download(tmp);

		cv::cuda::remap(
			pair_infos[camInd][1].gpu_rec_img, pair_infos[camInd][1].gpu_remap_img,
			pair_infos[camInd].gpu_warp_x, pair_infos[camInd].gpu_warp_y, cv::INTER_LINEAR, cv::BORDER_REFLECT); //cv::BORDER_TRANSPARENT is now not supported in cuda version


		//if (pair_infos[camInd].isFusionInit == false)
		//{
		//	cv::Mat cpu_master_tmp, cpu_slave_tmp;
		//	pair_infos[camInd][0].gpu_rec_img.download(cpu_master_tmp);
		//	pair_infos[camInd][1].gpu_remap_img.download(cpu_slave_tmp);
		//	pair_infos[camInd].ef.calcWeight(cpu_slave_tmp, cpu_master_tmp);
		//	pair_infos[camInd].isFusionInit = true;
		//}

		//pair_infos[camInd].ef.fusion(pair_infos[camInd][1].gpu_remap_img, pair_infos[camInd][0].gpu_rec_img, pair_infos[camInd].fusioned_img);
		pair_infos[camInd].ef.Raman_fusion(pair_infos[camInd][1].gpu_remap_img, pair_infos[camInd][0].gpu_rec_img, pair_infos[camInd].fusioned_img);

		//cv::Mat m1, m2, m3;
		//pair_infos[camInd][1].gpu_rec_img.download(m1);
		//pair_infos[camInd][0].gpu_rec_img.download(m2);
		//pair_infos[camInd].fusioned_img.download(m3);

		//img.data = reinterpret_cast<char*>(pair_infos[camInd][0].gpu_rec_img.data);
		img.data = reinterpret_cast<char*>(pair_infos[camInd].fusioned_img.data);

		////test only
		//cv::Mat m, s, opt;
		//int cols = sub_camInfos[pair_infos[camInd][0].index].width;
		//int rows = sub_camInfos[pair_infos[camInd][0].index].height;
		//m.create(rows, cols, CV_8U);
		//s.create(rows, cols, CV_8U);
		//opt.create(rows, cols, CV_8U);
		//memcpy(m.data, raw_imgs[pair_infos[camInd][0].index].data, cols * rows * sizeof(uchar));
		//memcpy(s.data, raw_imgs[pair_infos[camInd][1].index].data, cols * rows * sizeof(uchar));
		//cv::Rect r1(0, 0, cols / 2, rows);
		//cv::Rect r2(cols / 2, 0, cols / 2, rows);
		//cv::Mat tmp;
		//tmp.create(rows, cols / 2, CV_8U);
		//cv::Mat mask = cv::Mat::ones(rows, cols / 2, CV_8U);
		//cv::resize(m, tmp, cv::Size(cols / 2, rows));
		//tmp.copyTo(opt(r1), mask);
		//cv::resize(s, tmp, cv::Size(cols / 2, rows));
		//tmp.copyTo(opt(r2), mask);
		//memcpy(img.data, opt.data, cols * rows * sizeof(uchar));
		return 0;
	}

}
