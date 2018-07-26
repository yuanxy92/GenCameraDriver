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
			sp[0] = reader.Get(stridx, "master_sn", "default_master_sn");
			sp[1] = reader.Get(stridx, "slave_sn", "default_slave_sn");
			sp.int_path = reader.Get(stridx, "int_path", "default_int_path");
			sp.ext_path = reader.Get(stridx, "ext_path", "default_ext_path");
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
			pair_infos[i].sr.init(pair_infos[i].int_path, pair_infos[i].ext_path, cv::Size(sub_camInfos[idx].width, sub_camInfos[idx].height));
			size_t length = sizeof(uchar) * sub_camInfos[idx].width * sub_camInfos[idx].height;
			raw_imgs[i * 2].data = new char[length];
			raw_imgs[i * 2].length = length;
			raw_imgs[i * 2].maxLength = length;
			raw_imgs[i * 2 + 1].data = new char[length];
			raw_imgs[i * 2 + 1].length = length;
			raw_imgs[i * 2 + 1].maxLength = length;

			pair_infos[i][0].cpu_raw_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8U);
			pair_infos[i][1].cpu_raw_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8U);

			pair_infos[i][0].gpu_raw_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8U);
			pair_infos[i][1].gpu_raw_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8U);

			pair_infos[i][0].gpu_rgb_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8UC3);
			pair_infos[i][1].gpu_rgb_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8UC3);

			pair_infos[i][0].gpu_rec_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8UC3);
			pair_infos[i][1].gpu_rec_img.create(sub_camInfos[idx].height, sub_camInfos[idx].width, CV_8UC3);

			//Searching Slave
			idx = search_camera(pair_infos[i][1].sn, sub_camInfos);
			if (idx == -1)
			{
				SysUtil::errorOutput("GenCameraStereo::init failed! slave sn not found! Searching for " + pair_infos[i][1].sn);
				return -1;
			}
			pair_infos[i][1] = idx;

		}
		cameraNum = pair_infos.size();
		ths.resize(cameraNum);
		thStatus.resize(cameraNum);
		isInit = true;
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
			this->sub_cameraPtr->setAutoExposureLevel(camInd, level);
		}
		else
		{
			//TODO : now will affect 2 real cameras
			this->sub_cameraPtr->setAutoExposureLevel(pair_infos[camInd][0].index, level);
			this->sub_cameraPtr->setAutoExposureLevel(pair_infos[camInd][1].index, level);
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
		memcpy(pair_infos[camInd][0].cpu_raw_img.data, raw_imgs[pair_infos[camInd][0].index].data, cols * rows * sizeof(uchar));
		memcpy(pair_infos[camInd][1].cpu_raw_img.data, raw_imgs[pair_infos[camInd][1].index].data, cols * rows * sizeof(uchar));

		pair_infos[camInd][0].gpu_raw_img.upload(pair_infos[camInd][0].cpu_raw_img);
		pair_infos[camInd][1].gpu_raw_img.upload(pair_infos[camInd][1].cpu_raw_img);

		cv::cuda::demosaicing(pair_infos[camInd][0].gpu_raw_img, pair_infos[camInd][0].gpu_rgb_img,
			npp::bayerPatternNPP2CVRGB(static_cast<NppiBayerGridPosition>(
				static_cast<int>(camInfos[camInd].bayerPattern))));
		cv::cuda::demosaicing(pair_infos[camInd][1].gpu_raw_img, pair_infos[camInd][1].gpu_rgb_img,
			npp::bayerPatternNPP2CVRGB(static_cast<NppiBayerGridPosition>(
				static_cast<int>(camInfos[camInd].bayerPattern))));

		pair_infos[camInd].sr.rectify(
			pair_infos[camInd][0].gpu_rgb_img,
			pair_infos[camInd][0].gpu_rec_img,
			pair_infos[camInd][1].gpu_rgb_img,
			pair_infos[camInd][1].gpu_rec_img);

		img.data = reinterpret_cast<char*>(pair_infos[camInd][0].gpu_rec_img.data);

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
