/**
@brief class to rectify stereo images
@author Shane Yuan
@date Apr 26, 2018
*/

#ifndef __DEPTH_ESTIMATION_STEREO_RECTIFY__
#define __DEPTH_ESTIMATION_STEREO_RECTIFY__

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <thread>

#include <opencv2/opencv.hpp>

class StereoRectify {
private:
	std::string intfile;
	std::string extfile;
	cv::Mat M1;
	cv::Mat D1;
	cv::Mat M2;
	cv::Mat D2;
	cv::Mat R;
	cv::Mat T;
	cv::Mat R1;
	cv::Mat R2;
	cv::Mat P1;
	cv::Mat P2;
	cv::Mat Q;
	cv::Size imgsize;

	cv::Rect validRoi[2];
	cv::Mat rmap[2][2];
	cv::Rect rect;

	cv::cuda::GpuMat gpu_rmap[2][2];
public:

private:

public:
	StereoRectify();
	~StereoRectify();

	/**
	@brief function to init stero rectify class
	@param std::string intfile: file contain intrinsic parameter
	@param std::string extfile: file contain extrinsic parameter
	@param cv::Size imgsize: input image size
	@return int
	*/
	int init(std::string intfile, std::string extfile, cv::Size imgsize);

	/**
	@brief function to rectify images
	@param cv::Mat & leftImg: image of left view
	@param cv::Mat & rightImg: image of right view
	@return int
	*/
	int rectify(cv::Mat & leftImg, cv::Mat & rightImg);

	/**
	@brief function to rectify images
	@param cv::cuda::GpuMat & srcImg0: src image 0
	@param cv::cuda::GpuMat & dstImg0: dst image 0
	@param cv::cuda::GpuMat & srcImg1: src image 1
	@param cv::cuda::GpuMat & dstImg1: dst image 1
	@return int
	*/
	int rectify(cv::cuda::GpuMat & srcImg0, cv::cuda::GpuMat & dstImg0, cv::cuda::GpuMat & srcImg1, cv::cuda::GpuMat & dstImg1);
private:
	
};

#endif