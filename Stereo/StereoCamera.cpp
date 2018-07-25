/**
@brief Generic Camera Driver Class
Implementation of XIMEA camera
@author Shane Yuan
@date Dec 29, 2017
*/

#include "StereoCamera.h"
#include "INIReader.h"

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
			sp.master_sn = reader.Get(stridx, "master_sn", "default_master_sn");
			sp.slave_sn = reader.Get(stridx, "slave_sn", "default_slave_sn");
			sp.int_path = reader.Get(stridx, "int_path", "default_int_path");
			sp.ext_path = reader.Get(stridx, "ext_path", "default_ext_path");
			sp.inv = reader.GetBoolean(stridx, "master_sn", false);
			this->pair_infos.push_back(sp);
		}
		return 0;
	}

	int GenCameraStereo::search_camera(std::string sn, std::vector<GenCamInfo> list, GenCamInfo & info)
	{
		for (int i = 0; i < list.size(); i++)
		{
			if (list[i] == sn)
			{
				info = list[i];
				return i;
			}
		}
		return -1;
	}

	int GenCameraStereo::search_pair(std::string sn, std::vector<StereoPair> list, StereoPair & pair)
	{
		for (int i = 0; i < list.size(); i++)
		{
			if (list[i][0] == sn || list[i][1] == sn )
			{
				pair = list[i];
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
		this->sub_model = CameraModel::XIMEA_xiC;
		//TODO : How do we input the config file path?
		this->config_file_path = "./StereoConfig.ini";
		//TODO : Now can only open every camera
		this->sub_cameraPtr = cam::createCamera(cam::CameraModel::XIMEA_xiC);
		this->sub_cameraPtr->init();
		this->sub_cameraPtr->getCamInfos(this->sub_camInfos);
		this->sub_cameraPtr->setCamBufferType(GenCamBufferType::Raw);
		this->sub_cameraPtr->setCaptureMode(GenCamCaptureMode::Continous, 200);
		this->sub_cameraPtr->setCapturePurpose(GenCamCapturePurpose::Streaming);


		load_config(this->config_file_path);
		if (pair_infos.size() == 0)
		{
			SysUtil::errorOutput("GenCameraStereo::init failed! 0 pairs detected!");
			return -1;
		}
		this->raw_imgs.resize(pair_infos.size() * 2);
		for (int i = 0; i < pair_infos.size(); i++)
		{
			GenCamInfo info;
			if (search_camera(pair_infos[i][0], this->sub_camInfos, info) == -1)
			{
				SysUtil::errorOutput("GenCameraStereo::init failed! master sn not found! Searching for " + pair_infos[i][0]);
				return -1;
			}
			pair_infos[i].sr.init(pair_infos[i].int_path, pair_infos[i].ext_path, cv::Size(info.width, info.height));
			pair_infos[i].single_mix_img.create(info.height, info.width, CV_8U);
			pair_infos[i].single_master_img.create(info.height, info.width, CV_8U);
			pair_infos[i].single_slave_img.create(info.height, info.width, CV_8U);
			if (search_camera(pair_infos[i][1], this->sub_camInfos, info) == -1)
			{
				SysUtil::errorOutput("GenCameraStereo::init failed! master sn not found! Searching for " + pair_infos[i][1]);
				return -1;
			}
			size_t length = sizeof(uchar) * info.width * info.height;
			this->raw_imgs[i * 2].data = new char[length];
			this->raw_imgs[i * 2].length = length;
			this->raw_imgs[i * 2].maxLength = length;
			this->raw_imgs[i * 2 + 1].data = new char[length];
			this->raw_imgs[i * 2 + 1].length = length;
			this->raw_imgs[i * 2 + 1].maxLength = length;
		}
		this->cameraNum = pair_infos.size();
		ths.resize(this->cameraNum);
		thStatus.resize(this->cameraNum);
		this->isInit = true;
		// init image ratio vector
		imgRatios.resize(this->cameraNum);
		for (size_t i = 0; i < this->cameraNum; i++) {
			imgRatios[i] = GenCamImgRatio::Full;
		}
		return 0;
	}

	int GenCameraStereo::startCapture() {
		this->sub_cameraPtr->startCapture();
		this->isCapture = true;
		return 0;
	}

	int GenCameraStereo::stopCapture() {
		this->sub_cameraPtr->stopCapture();
		this->isCapture = false;
		return 0;
	}

	int GenCameraStereo::release() {
		if (this->isCapture == true)
			this->stopCapture();
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
		camInfos.resize(this->cameraNum);
		this->sub_cameraPtr->getCamInfos(this->sub_camInfos);
		for (size_t i = 0; i < this->cameraNum; i++) 
		{
			GenCamInfo info;
			search_camera(pair_infos[i][0], this->sub_camInfos, info);
			camInfos[i] = info;
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
			std::string sn = this->camInfos[camInd].sn;
			StereoPair sp;
			this->search_pair(sn, this->pair_infos, sp);
			GenCamInfo info;
			int idx;
			idx = this->search_camera(sp[0], this->sub_camInfos, info);
			this->sub_cameraPtr->setFPS(idx, fps, exposureUpperLimitRatio);
			idx = this->search_camera(sp[1], this->sub_camInfos, info);
			this->sub_cameraPtr->setFPS(idx, fps, exposureUpperLimitRatio);
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
			std::string sn = this->camInfos[camInd].sn;
			StereoPair sp;
			this->search_pair(sn, this->pair_infos, sp);
			GenCamInfo info;
			int idx;
			idx = this->search_camera(sp[0], this->sub_camInfos, info);
			this->sub_cameraPtr->setAutoWhiteBalance(idx);
			idx = this->search_camera(sp[1], this->sub_camInfos, info);
			this->sub_cameraPtr->setAutoWhiteBalance(idx);
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
			std::string sn = this->camInfos[camInd].sn;
			StereoPair sp;
			this->search_pair(sn, this->pair_infos, sp);
			GenCamInfo info;
			int idx;
			idx = this->search_camera(sp[0], this->sub_camInfos, info);
			this->sub_cameraPtr->setWhiteBalance(idx, redGain, greenGain, blueGain);
			idx = this->search_camera(sp[1], this->sub_camInfos, info);
			this->sub_cameraPtr->setWhiteBalance(idx, redGain, greenGain, blueGain);
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
			std::string sn = this->camInfos[camInd].sn;
			StereoPair sp;
			this->search_pair(sn, this->pair_infos, sp);
			GenCamInfo info;
			int idx;
			idx = this->search_camera(sp[0], this->sub_camInfos, info);
			this->sub_cameraPtr->setAutoExposure(idx, autoExposure);
			idx = this->search_camera(sp[1], this->sub_camInfos, info);
			this->sub_cameraPtr->setAutoExposure(idx, autoExposure);
		}
	}

	int GenCameraStereo::setAutoExposureLevel(int camInd, float level) {
		if (camInd == -1)
		{
			this->sub_cameraPtr->setAutoExposureLevel(camInd, level);
		}
		else
		{
			//TODO : now will affect 2 real cameras
			std::string sn = this->camInfos[camInd].sn;
			StereoPair sp;
			this->search_pair(sn, this->pair_infos, sp);
			GenCamInfo info;
			int idx;
			idx = this->search_camera(sp[0], this->sub_camInfos, info);
			this->sub_cameraPtr->setAutoExposureLevel(idx, level);
			idx = this->search_camera(sp[1], this->sub_camInfos, info);
			this->sub_cameraPtr->setAutoExposureLevel(idx, level);
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
			std::string sn = this->camInfos[camInd].sn;
			StereoPair sp;
			this->search_pair(sn, this->pair_infos, sp);
			GenCamInfo info;
			int idx;
			idx = this->search_camera(sp[0], this->sub_camInfos, info);
			this->sub_cameraPtr->setAutoExposureCompensation(idx, status, relativeEV);
			idx = this->search_camera(sp[1], this->sub_camInfos, info);
			this->sub_cameraPtr->setAutoExposureCompensation(idx, status, relativeEV);
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
			std::string sn = this->camInfos[camInd].sn;
			StereoPair sp;
			this->search_pair(sn, this->pair_infos, sp);
			GenCamInfo info;
			int idx;
			idx = this->search_camera(sp[0], this->sub_camInfos, info);
			this->sub_cameraPtr->adjustBrightness(idx, brightness);
			idx = this->search_camera(sp[1], this->sub_camInfos, info);
			this->sub_cameraPtr->adjustBrightness(idx, brightness);
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
			std::string sn = this->camInfos[camInd].sn;
			StereoPair sp;
			this->search_pair(sn, this->pair_infos, sp);
			GenCamInfo info;
			int idx;
			idx = this->search_camera(sp[0], this->sub_camInfos, info);
			this->sub_cameraPtr->setExposure(idx, time);
			idx = this->search_camera(sp[1], this->sub_camInfos, info);
			this->sub_cameraPtr->setExposure(idx, time);
		}
		return 0;
	}

	int GenCameraStereo::getBayerPattern(int camInd, GenCamBayerPattern & bayerPattern) {

		std::string sn = this->camInfos[camInd].sn;
		StereoPair sp;
		this->search_pair(sn, this->pair_infos, sp);
		GenCamInfo info;
		int idx;
		idx = this->search_camera(sp[0], this->sub_camInfos, info);
		this->sub_cameraPtr->getBayerPattern(idx, bayerPattern);
		return 0;
	}

	int GenCameraStereo::makeSetEffective(int k) {
		this->sub_cameraPtr->makeSetEffective(k);
		return 0;
	}

	int GenCameraStereo::captureFrame(int camInd, Imagedata & img) 
	{
		this->sub_cameraPtr->captureFrame(this->raw_imgs);
		std::string sn = this->camInfos[camInd].sn;
		StereoPair sp;
		int sp_idx;
		sp_idx = this->search_pair(sn, this->pair_infos, sp);
		GenCamInfo info;
		int idx;
		idx = this->search_camera(sp[0], this->sub_camInfos, info);
		memcpy(this->pair_infos[sp_idx].single_master_img.data, this->raw_imgs[idx].data, info.width * info.height * sizeof(unsigned char));
		idx = this->search_camera(sp[1], this->sub_camInfos, info);
		memcpy(this->pair_infos[sp_idx].single_slave_img.data, this->raw_imgs[idx].data, info.width * info.height * sizeof(unsigned char));
		int cols = this->pair_infos[sp_idx].single_master_img.cols;
		int rows = this->pair_infos[sp_idx].single_master_img.rows;
		cv::Mat roi = this->pair_infos[sp_idx].single_master_img(cv::Rect(0, 0, cols / 2, rows));

		cv::Mat mask_master(this->pair_infos[sp_idx].single_master_img.rows,
			this->pair_infos[sp_idx].single_master_img.cols,
			this->pair_infos[sp_idx].single_master_img.depth,
			cv::Scalar(0));


		// capture images
		checkXIMEAErrors(xiGetImage(hcams[camInd], 500, &xiImages[camInd]));
		// copy to opencv mat
		std::memcpy(img.data, xiImages[camInd].bp, sizeof(unsigned char) *
			xiImages[camInd].width * xiImages[camInd].height);
		return 0;
	}

}
