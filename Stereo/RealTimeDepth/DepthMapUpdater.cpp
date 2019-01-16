#include "DepthMapUpdater.h"

#ifdef OUTPUT_MEDIAN_RESULT
int DepthMapUpdater::_updaterCount = 0;
#endif


int DepthMapUpdater::init(cv::Mat& masterBackground, cv::Mat& slaveBackground, cv::Mat& depthBackground)
{
	_backMaster = masterBackground;
	_backSlave = slaveBackground;
	_backDepth = depthBackground;
	cv::resize(_backMaster, _backMaster, cv::Size(JIANING_WIDTH, JIANING_HEIGHT));
	cv::resize(_backSlave, _backSlave, cv::Size(JIANING_WIDTH, JIANING_HEIGHT));
	cv::resize(_backDepth, _backDepth, cv::Size(JIANING_WIDTH, JIANING_HEIGHT));
	_gpu_backMaster.upload(_backMaster);
	_gpu_backSlave.upload(_backSlave);
	_gpu_backDepth.upload(_backDepth);
	_frameCount = 0;

	_mog = cv::cuda::createBackgroundSubtractorMOG(70);
	_gauss = cv::cuda::createGaussianFilter(CV_8UC1, CV_8UC1, cv::Size(5, 5), 0);

#if (SELECTCHANNEL==1 | SELECTCHANNEL==2) // use Intensity or Gradient image    
	_dep = depthmap(CV_32FC1, 1, cv::IMREAD_GRAYSCALE);
#elif (SELECTCHANNEL==3) // use RGB image
	_dep = depthmap(CV_32FC3, 3, cv::IMREAD_COLOR);
#endif
	if(_backMaster.empty() || _backSlave.empty())
		std::cout<<"DepthMapUpdater::init empty mat!"<<std::endl;
	_dep.init_depth(_backMaster, _backSlave, _elem.flag);

	//_mog->apply(_gpu_backMaster, _gpu_mask, 0.01);
#ifdef OUTPUT_MEDIAN_RESULT
	char command[256];
	sprintf(command, "mkdir OUTPUT_MEDIAN_RESULT");
	system(command);
	_thisUpdaterID = _updaterCount;
	_updaterCount++;
#endif

    return 0;
}

int DepthMapUpdater::update(cv::Mat& masterMat, cv::Mat& slaveMat, cv::Mat& depthWithMask)
{
	cv::Mat _m, _s;
	cv::cuda::GpuMat _gpu_m;
	cv::resize(masterMat, _m, cv::Size(JIANING_WIDTH, JIANING_HEIGHT));
	cv::resize(slaveMat, _s, cv::Size(JIANING_WIDTH, JIANING_HEIGHT));
	_gpu_m.upload(_m);
	_mog->apply(_gpu_m, _gpu_mask, 0.01);

	_mog->getBackgroundImage(_gpu_updateBackground);
	_gpu_updateBackground.download(_updateBackground);
	_elem.init_frame(_updateBackground);

	_gauss->apply(_gpu_mask, _gpu_mask);
	_gpu_mask.download(_mask);
	std::vector<cv::Rect> result = _elem.Find_location(_mask, _m, _s);
	_depth = _dep.get_depth(_m, _s);
	cv::Mat diff_mask = _elem.refine_mask(_updateBackground, _m, _mask);
	depthWithMask = _dep.update_depth_robust(_depth, diff_mask);

	_dep.refine_depth(depthWithMask, diff_mask, result, _m, _s);
	_elem.res_out(diff_mask, depthWithMask);
	_frameCount++;
    return 0;
}

int DepthMapUpdater::update(cv::cuda::GpuMat& masterMat, cv::cuda::GpuMat& slaveMat, cv::Mat& depthWithMask)
{
	cv::Mat _m, _s;
	cv::cuda::GpuMat _gpu_m,_gpu_s;
	cv::cuda::resize(masterMat, _gpu_m, cv::Size(JIANING_WIDTH, JIANING_HEIGHT));
	cv::cuda::resize(slaveMat, _gpu_s, cv::Size(JIANING_WIDTH, JIANING_HEIGHT));
	_gpu_m.download(_m);
	_gpu_s.download(_s);
#ifdef OUTPUT_MEDIAN_RESULT
	cv::Mat _m2, _s2;
	cv::cvtColor(_m, _m2, cv::COLOR_RGB2BGR);
	cv::cvtColor(_s, _s2, cv::COLOR_RGB2BGR);
	cv::imwrite(cv::format("OUTPUT_MEDIAN_RESULT/%d_test_m_%d.jpg",_thisUpdaterID,_frameCount),_m2);
	cv::imwrite(cv::format("OUTPUT_MEDIAN_RESULT/%d_test_s_%d.jpg",_thisUpdaterID,_frameCount),_s2);
#endif
	_mog->apply(_gpu_m, _gpu_mask, 0.01);

	_mog->getBackgroundImage(_gpu_updateBackground);
	_gpu_updateBackground.download(_updateBackground);
	_elem.init_frame(_updateBackground);

	_gauss->apply(_gpu_mask, _gpu_mask);
	_gpu_mask.download(_mask);
	std::vector<cv::Rect> result = _elem.Find_location(_mask, _m, _s);
	_depth = _dep.get_depth(_m, _s);

	cv::Mat diff_mask = _elem.refine_mask(_updateBackground, _m, _mask);
	depthWithMask = _dep.update_depth_robust(_depth, diff_mask);
	_dep.refine_depth(depthWithMask, diff_mask, result, _m, _s);
	_elem.res_out(diff_mask, depthWithMask);
#ifdef OUTPUT_MEDIAN_RESULT
	//cv::imwrite(cv::format("test_mask_%d.jpg",_frameCount),_mask);
	// cv::Mat dis_16;
	// _depth.convertTo(dis_16, CV_16UC1);
	// cv::imwrite(cv::format("test_disparity_16bit_%d.png",_frameCount),dis_16);
	_dep.SavePFMFile(depthWithMask,cv::format("OUTPUT_MEDIAN_RESULT/%d_test_disparity_all_pfm_%d.pfm",_thisUpdaterID,_frameCount).c_str());
#endif
	_frameCount++;
    return 0;
}

int DepthMapUpdater::getFrameCount()
{
	return _frameCount;
}