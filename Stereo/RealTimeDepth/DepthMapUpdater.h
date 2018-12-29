/**
@brief DepthMapUpdater.h 
class for depth map update
@author zhu-ty
@date Dec 25, 2018
*/

#ifndef __DEPTH_MAP_UPDATER__
#define __DEPTH_MAP_UPDATER__

#include <opencv2/core/utility.hpp>
#include <opencv2/cudabgsegm.hpp>
#include <opencv2/cudalegacy.hpp>
#include <opencv2/video.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <sys/time.h>
#include <fstream>
#include "depthmap.h"    
#include "oflow.h"
#include "GhostElemer.h"

// #define SELECTMODE 2
// #define SELECTCHANNEL 1

#define JIANING_WIDTH 1000
#define JIANING_HEIGHT 750
//#define OUTPUT_MIDIAN_RESULAT

class DepthMapUpdater
{
public:
    DepthMapUpdater(){};
    ~DepthMapUpdater(){};
    /**
	@brief init updater with background infomation
	@param cv::Mat& masterBackground: master camera background image
	@param cv::Mat& slaveBackground: slave camera background image
	@param cv::Mat& depthBackground: initial background depth image
	@return int(0)
	*/
    int init(cv::Mat& masterBackground, cv::Mat& slaveBackground, cv::Mat& depthBackground);

    /**
	@brief update depth image with mask
	@param cv::Mat& masterMat: input master camera image
	@param cv::Mat& slaveMat: input slave camera image
	@param cv::Mat& depthWithMask: updated depth image with mask
	@return int(0)
	*/
    int update(cv::Mat& masterMat, cv::Mat& slaveMat, cv::Mat& depthWithMask);

	/**
	@brief update depth image with mask
	@param cv::cuda::GpuMat& masterMat: input master camera image
	@param cv::cuda::GpuMat& slaveMat: input slave camera image
	@param cv::cuda::GpuMat& depthWithMask: updated depth image with mask
	@return int(0)
	*/
    int update(cv::cuda::GpuMat& masterMat, cv::cuda::GpuMat& slaveMat, cv::Mat& depthWithMask);

    /**
	@brief get how many pairs have been inputted
	@return frame count
	*/
    int getFrameCount();
private:
    int _frameCount = 0;
    cv::Ptr<cv::BackgroundSubtractor> _mog;
	cv::Ptr<cv::cuda::Filter> _gauss;
    GhostElemer _elem;
	depthmap _dep;

	cv::Mat _backMaster, _backSlave, _backDepth;
	cv::cuda::GpuMat _gpu_backMaster, _gpu_backSlave, _gpu_backDepth;

	cv::Mat _mask,_depth;
	cv::cuda::GpuMat _gpu_mask;
};

#endif //__DEPTH_MAP_UPDATER__