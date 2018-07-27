/**
@brief class to rectify stereo images
@author Shane Yuan
@date Apr 26, 2018
*/

#include "StereoRectify.h"

StereoRectify::StereoRectify() {}
StereoRectify::~StereoRectify() {}

/**
@brief function to init stero rectify class
@param std::string intfile: file contain intrinsic parameter
@param std::string extfile: file contain extrinsic parameter
@param cv::Size imgsize: input image size
@return int
*/
int StereoRectify::init(std::string intfile, std::string extfile, cv::Size imgsize) {
	this->intfile = intfile;
	this->extfile = extfile;

	cv::FileStorage fs1(this->intfile, cv::FileStorage::READ);
	fs1["M1"] >> M1;
	fs1["D1"] >> D1;
	fs1["M2"] >> M2;
	fs1["D2"] >> D2;
	fs1.release();

	cv::FileStorage fs2(this->extfile, cv::FileStorage::READ);
	fs2["R"] >> R;
	fs2["T"] >> T;
	fs2["R1"] >> R1;
	fs2["R2"] >> R2;
	fs2["P1"] >> P1;
	fs2["P2"] >> P2;
	fs2["Q"] >> Q;
	fs2.release();

	this->imgsize = imgsize;

	cv::stereoRectify(M1, D1, M2, D2,
		imgsize, R, T, R1, R2, P1, P2, Q,
		cv::CALIB_ZERO_DISPARITY, 1, imgsize, &validRoi[0], &validRoi[1]);
	rect = validRoi[0] & validRoi[1];

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(M1, D1, R1, P1, imgsize, CV_32FC1, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(M2, D2, R2, P2, imgsize, CV_32FC1, rmap[1][0], rmap[1][1]);

	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			gpu_rmap[i][j].upload(rmap[i][j]);

	return 0;
}


/**
@brief function to rectify images
@param cv::Mat & leftImg: image of left view
@param cv::Mat & rightImg: image of right view
@return int
*/
int StereoRectify::rectify(cv::Mat & leftImg, cv::Mat & rightImg) {
	//TODO: change to GPU version
	cv::remap(leftImg, leftImg, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
	cv::remap(rightImg, rightImg, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
	leftImg(rect).copyTo(leftImg);
	rightImg(rect).copyTo(rightImg);
	return 0;
}

int StereoRectify::rectify(cv::cuda::GpuMat & srcImg0, cv::cuda::GpuMat & dstImg0, cv::cuda::GpuMat & srcImg1, cv::cuda::GpuMat & dstImg1)
{
	cv::cuda::remap(srcImg0, dstImg0, gpu_rmap[0][0], gpu_rmap[0][1], cv::INTER_LINEAR);
	cv::cuda::GpuMat tmp_dst0;
	dstImg0(rect).copyTo(tmp_dst0);
	cv::cuda::resize(tmp_dst0, dstImg0, cv::Size(srcImg0.cols, srcImg0.rows));

	cv::cuda::remap(srcImg1, dstImg1, gpu_rmap[1][0], gpu_rmap[1][1], cv::INTER_LINEAR);
	cv::cuda::GpuMat tmp_dst1;
	dstImg1(rect).copyTo(tmp_dst1);
	cv::cuda::resize(tmp_dst1, dstImg1, cv::Size(srcImg1.cols, srcImg1.rows));
	return 0;
}
