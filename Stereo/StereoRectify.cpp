/**
@brief class to rectify stereo images
@author Shane Yuan
@date Apr 26, 2018
*/

#include "StereoRectify.h"

#include <opencv2/core/types_c.h>

void openCV3_4_0cvStereoRectify(const CvMat* _cameraMatrix1, const CvMat* _cameraMatrix2,
	const CvMat* _distCoeffs1, const CvMat* _distCoeffs2,
	CvSize imageSize, const CvMat* matR, const CvMat* matT,
	CvMat* _R1, CvMat* _R2, CvMat* _P1, CvMat* _P2,
	CvMat* matQ, int flags, double alpha, CvSize newImgSize,
	CvRect* roi1, CvRect* roi2);

void openCV3_4_0icvGetRectangles(const CvMat* cameraMatrix, const CvMat* distCoeffs,
	const CvMat* R, const CvMat* newCameraMatrix, CvSize imgSize,
	cv::Rect_<float>& inner, cv::Rect_<float>& outer);


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

	std::cout << "int File: " <<this->intfile <<std::endl;
	cv::FileStorage fs1(this->intfile, cv::FileStorage::READ);
	fs1["M1"] >> M1;
	fs1["D1"] >> D1;
	fs1["M2"] >> M2;
	fs1["D2"] >> D2;
	fs1.release();
	std::cout << "ext File: " <<this->extfile <<std::endl;
	cv::FileStorage fs2(this->extfile, cv::FileStorage::READ);
	fs2["R"] >> R;
	fs2["T"] >> T;
	fs2["R1"] >> R1;
	fs2["R2"] >> R2;
	fs2["P1"] >> P1;
	fs2["P2"] >> P2;
	fs2["Q"] >> Q;
	fs2.release();
	std::cout << "R : " << R <<std::endl;
	std::cout << "T : " << T <<std::endl;

	this->imgsize = imgsize;

	cv::stereoRectify(M1, D1, M2, D2,
		imgsize, R, T, R1, R2, P1, P2, Q,
		cv::CALIB_ZERO_DISPARITY, 1, imgsize, &validRoi[0], &validRoi[1]);
	rect = validRoi[0] & validRoi[1];
	std::cout << "Rect : " << rect <<std::endl;
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
	std::cout << "cv::cuda::remap done" << std::endl;
	cv::cuda::GpuMat tmp_dst0;
	dstImg0(rect).copyTo(tmp_dst0);
	std::cout << "cv::cuda::GpuMat::copyTo done" << std::endl;
	std::cout << "size = " << tmp_dst0.cols << " " << tmp_dst0.rows << std::endl;
	cv::cuda::resize(tmp_dst0, dstImg0, cv::Size(srcImg0.cols, srcImg0.rows));
	std::cout << "cv::cuda::resize done" << std::endl;

	cv::cuda::remap(srcImg1, dstImg1, gpu_rmap[1][0], gpu_rmap[1][1], cv::INTER_LINEAR);
	cv::cuda::GpuMat tmp_dst1;
	dstImg1(rect).copyTo(tmp_dst1);
	cv::cuda::resize(tmp_dst1, dstImg1, cv::Size(srcImg1.cols, srcImg1.rows));
	return 0;
}

using namespace cv;

void StereoRectify::openCV3_4_0stereoRectify(
	cv::Mat _cameraMatrix1, cv::Mat _distCoeffs1,
	cv::Mat _cameraMatrix2, cv::Mat _distCoeffs2,
	cv::Size imageSize, cv::Mat _Rmat,
	cv::Mat _Tmat, cv::Mat & _Rmat1,
	cv::Mat & _Rmat2, cv::Mat & _Pmat1,
	cv::Mat & _Pmat2, cv::Mat & _Qmat,
	int flags, double alpha,
	cv::Size newImageSize, cv::Rect * validPixROI1,
	cv::Rect * validPixROI2)
{
	Mat cameraMatrix1 = _cameraMatrix1, cameraMatrix2 = _cameraMatrix2;
	Mat distCoeffs1 = _distCoeffs1, distCoeffs2 = _distCoeffs2;
	Mat Rmat = _Rmat, Tmat = _Tmat;
	CvMat c_cameraMatrix1 = cameraMatrix1;
	CvMat c_cameraMatrix2 = cameraMatrix2;
	CvMat c_distCoeffs1 = distCoeffs1;
	CvMat c_distCoeffs2 = distCoeffs2;
	CvMat c_R = Rmat, c_T = Tmat;

	int rtype = CV_64F;
	_Rmat1.create(3, 3, rtype);
	_Rmat2.create(3, 3, rtype);
	_Pmat1.create(3, 4, rtype);
	_Pmat2.create(3, 4, rtype);
	CvMat c_R1 = _Rmat1, c_R2 = _Rmat2, c_P1 = _Pmat1, c_P2 = _Pmat2;
	CvMat c_Q, *p_Q = 0;

	_Qmat.create(4, 4, rtype);
	p_Q = &(c_Q = _Qmat);

	CvMat *p_distCoeffs1 = distCoeffs1.empty() ? NULL : &c_distCoeffs1;
	CvMat *p_distCoeffs2 = distCoeffs2.empty() ? NULL : &c_distCoeffs2;
	openCV3_4_0cvStereoRectify(&c_cameraMatrix1, &c_cameraMatrix2, p_distCoeffs1, p_distCoeffs2,
		imageSize, &c_R, &c_T, &c_R1, &c_R2, &c_P1, &c_P2, p_Q, flags, alpha,
		newImageSize, (CvRect*)validPixROI1, (CvRect*)validPixROI2);
}


void openCV3_4_0cvStereoRectify(const CvMat* _cameraMatrix1, const CvMat* _cameraMatrix2,
	const CvMat* _distCoeffs1, const CvMat* _distCoeffs2,
	CvSize imageSize, const CvMat* matR, const CvMat* matT,
	CvMat* _R1, CvMat* _R2, CvMat* _P1, CvMat* _P2,
	CvMat* matQ, int flags, double alpha, CvSize newImgSize,
	CvRect* roi1, CvRect* roi2)
{
	double _om[3], _t[3] = { 0 }, _uu[3] = { 0,0,0 }, _r_r[3][3], _pp[3][4];
	double _ww[3], _wr[3][3], _z[3] = { 0,0,0 }, _ri[3][3];
	cv::Rect_<float> inner1, inner2, outer1, outer2;

	CvMat om = cvMat(3, 1, CV_64F, _om);
	CvMat t = cvMat(3, 1, CV_64F, _t);
	CvMat uu = cvMat(3, 1, CV_64F, _uu);
	CvMat r_r = cvMat(3, 3, CV_64F, _r_r);
	CvMat pp = cvMat(3, 4, CV_64F, _pp);
	CvMat ww = cvMat(3, 1, CV_64F, _ww); // temps
	CvMat wR = cvMat(3, 3, CV_64F, _wr);
	CvMat Z = cvMat(3, 1, CV_64F, _z);
	CvMat Ri = cvMat(3, 3, CV_64F, _ri);
	double nx = imageSize.width, ny = imageSize.height;
	int i, k;

	if (matR->rows == 3 && matR->cols == 3)
		cvRodrigues2(matR, &om);          // get vector rotation
	else
		cvConvert(matR, &om); // it's already a rotation vector
	cvConvertScale(&om, &om, -0.5); // get average rotation
	cvRodrigues2(&om, &r_r);        // rotate cameras to same orientation by averaging
	cvMatMul(&r_r, matT, &t);

	int idx = fabs(_t[0]) > fabs(_t[1]) ? 0 : 1;
	double c = _t[idx], nt = cvNorm(&t, 0, CV_L2);
	_uu[idx] = c > 0 ? 1 : -1;

	// calculate global Z rotation
	cvCrossProduct(&t, &uu, &ww);
	double nw = cvNorm(&ww, 0, CV_L2);
	if (nw > 0.0)
		cvConvertScale(&ww, &ww, acos(fabs(c) / nt) / nw);
	cvRodrigues2(&ww, &wR);

	// apply to both views
	cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, CV_GEMM_B_T);
	cvConvert(&Ri, _R1);
	cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, 0);
	cvConvert(&Ri, _R2);
	cvMatMul(&Ri, matT, &t);

	// calculate projection/camera matrices
	// these contain the relevant rectified image internal params (fx, fy=fx, cx, cy)
	double fc_new = DBL_MAX;
	CvPoint2D64f cc_new[2] = { { 0,0 },{ 0,0 } };

	for (k = 0; k < 2; k++) {
		const CvMat* A = k == 0 ? _cameraMatrix1 : _cameraMatrix2;
		const CvMat* Dk = k == 0 ? _distCoeffs1 : _distCoeffs2;
		double dk1 = Dk && Dk->data.ptr ? cvmGet(Dk, 0, 0) : 0;
		double fc = cvmGet(A, idx ^ 1, idx ^ 1);
		if (dk1 < 0) {
			fc *= 1 + dk1*(nx*nx + ny*ny) / (4 * fc*fc);
		}
		fc_new = MIN(fc_new, fc);
	}

	for (k = 0; k < 2; k++)
	{
		const CvMat* A = k == 0 ? _cameraMatrix1 : _cameraMatrix2;
		const CvMat* Dk = k == 0 ? _distCoeffs1 : _distCoeffs2;
		CvPoint2D32f _pts[4];
		CvPoint3D32f _pts_3[4];
		CvMat pts = cvMat(1, 4, CV_32FC2, _pts);
		CvMat pts_3 = cvMat(1, 4, CV_32FC3, _pts_3);

		for (i = 0; i < 4; i++)
		{
			int j = (i<2) ? 0 : 1;
			_pts[i].x = (float)((i % 2)*(nx));
			_pts[i].y = (float)(j*(ny));
		}
		cvUndistortPoints(&pts, &pts, A, Dk, 0, 0);
		cvConvertPointsHomogeneous(&pts, &pts_3);

		//Change camera matrix to have cc=[0,0] and fc = fc_new
		double _a_tmp[3][3];
		CvMat A_tmp = cvMat(3, 3, CV_64F, _a_tmp);
		_a_tmp[0][0] = fc_new;
		_a_tmp[1][1] = fc_new;
		_a_tmp[0][2] = 0.0;
		_a_tmp[1][2] = 0.0;
		cvProjectPoints2(&pts_3, k == 0 ? _R1 : _R2, &Z, &A_tmp, 0, &pts);
		CvScalar avg = cvAvg(&pts);
		cc_new[k].x = (nx) / 2 - avg.val[0];
		cc_new[k].y = (ny) / 2 - avg.val[1];
	}

	// vertical focal length must be the same for both images to keep the epipolar constraint
	// (for horizontal epipolar lines -- TBD: check for vertical epipolar lines)
	// use fy for fx also, for simplicity

	// For simplicity, set the principal points for both cameras to be the average
	// of the two principal points (either one of or both x- and y- coordinates)
	if (flags & CALIB_ZERO_DISPARITY)
	{
		cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x)*0.5;
		cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;
	}
	else if (idx == 0) // horizontal stereo
		cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;
	else // vertical stereo
		cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x)*0.5;

	cvZero(&pp);
	_pp[0][0] = _pp[1][1] = fc_new;
	_pp[0][2] = cc_new[0].x;
	_pp[1][2] = cc_new[0].y;
	_pp[2][2] = 1;
	cvConvert(&pp, _P1);

	_pp[0][2] = cc_new[1].x;
	_pp[1][2] = cc_new[1].y;
	_pp[idx][3] = _t[idx] * fc_new; // baseline * focal length
	cvConvert(&pp, _P2);

	alpha = MIN(alpha, 1.);

	openCV3_4_0icvGetRectangles(_cameraMatrix1, _distCoeffs1, _R1, _P1, imageSize, inner1, outer1);
	openCV3_4_0icvGetRectangles(_cameraMatrix2, _distCoeffs2, _R2, _P2, imageSize, inner2, outer2);

	{
		newImgSize = newImgSize.width*newImgSize.height != 0 ? newImgSize : imageSize;
		double cx1_0 = cc_new[0].x;
		double cy1_0 = cc_new[0].y;
		double cx2_0 = cc_new[1].x;
		double cy2_0 = cc_new[1].y;
		double cx1 = newImgSize.width*cx1_0 / imageSize.width;
		double cy1 = newImgSize.height*cy1_0 / imageSize.height;
		double cx2 = newImgSize.width*cx2_0 / imageSize.width;
		double cy2 = newImgSize.height*cy2_0 / imageSize.height;
		double s = 1.;

		if (alpha >= 0)
		{
			double s0 = std::max(std::max(std::max((double)cx1 / (cx1_0 - inner1.x), (double)cy1 / (cy1_0 - inner1.y)),
				(double)(newImgSize.width - cx1) / (inner1.x + inner1.width - cx1_0)),
				(double)(newImgSize.height - cy1) / (inner1.y + inner1.height - cy1_0));
			s0 = std::max(std::max(std::max(std::max((double)cx2 / (cx2_0 - inner2.x), (double)cy2 / (cy2_0 - inner2.y)),
				(double)(newImgSize.width - cx2) / (inner2.x + inner2.width - cx2_0)),
				(double)(newImgSize.height - cy2) / (inner2.y + inner2.height - cy2_0)),
				s0);

			double s1 = std::min(std::min(std::min((double)cx1 / (cx1_0 - outer1.x), (double)cy1 / (cy1_0 - outer1.y)),
				(double)(newImgSize.width - cx1) / (outer1.x + outer1.width - cx1_0)),
				(double)(newImgSize.height - cy1) / (outer1.y + outer1.height - cy1_0));
			s1 = std::min(std::min(std::min(std::min((double)cx2 / (cx2_0 - outer2.x), (double)cy2 / (cy2_0 - outer2.y)),
				(double)(newImgSize.width - cx2) / (outer2.x + outer2.width - cx2_0)),
				(double)(newImgSize.height - cy2) / (outer2.y + outer2.height - cy2_0)),
				s1);

			s = s0*(1 - alpha) + s1*alpha;
		}

		fc_new *= s;
		cc_new[0] = cvPoint2D64f(cx1, cy1);
		cc_new[1] = cvPoint2D64f(cx2, cy2);

		cvmSet(_P1, 0, 0, fc_new);
		cvmSet(_P1, 1, 1, fc_new);
		cvmSet(_P1, 0, 2, cx1);
		cvmSet(_P1, 1, 2, cy1);

		cvmSet(_P2, 0, 0, fc_new);
		cvmSet(_P2, 1, 1, fc_new);
		cvmSet(_P2, 0, 2, cx2);
		cvmSet(_P2, 1, 2, cy2);
		cvmSet(_P2, idx, 3, s*cvmGet(_P2, idx, 3));

		if (roi1)
		{
			*roi1 = cv::Rect(cvCeil((inner1.x - cx1_0)*s + cx1),
				cvCeil((inner1.y - cy1_0)*s + cy1),
				cvFloor(inner1.width*s), cvFloor(inner1.height*s))
				& cv::Rect(0, 0, newImgSize.width, newImgSize.height);
		}

		if (roi2)
		{
			*roi2 = cv::Rect(cvCeil((inner2.x - cx2_0)*s + cx2),
				cvCeil((inner2.y - cy2_0)*s + cy2),
				cvFloor(inner2.width*s), cvFloor(inner2.height*s))
				& cv::Rect(0, 0, newImgSize.width, newImgSize.height);
		}
	}

	if (matQ)
	{
		double q[] =
		{
			1, 0, 0, -cc_new[0].x,
			0, 1, 0, -cc_new[0].y,
			0, 0, 0, fc_new,
			0, 0, -1. / _t[idx],
			(idx == 0 ? cc_new[0].x - cc_new[1].x : cc_new[0].y - cc_new[1].y) / _t[idx]
		};
		CvMat Q = cvMat(4, 4, CV_64F, q);
		cvConvert(&Q, matQ);
	}
}

void openCV3_4_0icvGetRectangles(const CvMat* cameraMatrix, const CvMat* distCoeffs,
	const CvMat* R, const CvMat* newCameraMatrix, CvSize imgSize,
	cv::Rect_<float>& inner, cv::Rect_<float>& outer)
{
	const int N = 9;
	int x, y, k;
	cv::Ptr<CvMat> _pts(cvCreateMat(1, N*N, CV_32FC2));
	CvPoint2D32f* pts = (CvPoint2D32f*)(_pts->data.ptr);

	for (y = k = 0; y < N; y++)
		for (x = 0; x < N; x++)
			pts[k++] = cvPoint2D32f((float)x*imgSize.width / (N - 1),
			(float)y*imgSize.height / (N - 1));

	cvUndistortPoints(_pts, _pts, cameraMatrix, distCoeffs, R, newCameraMatrix);

	float iX0 = -FLT_MAX, iX1 = FLT_MAX, iY0 = -FLT_MAX, iY1 = FLT_MAX;
	float oX0 = FLT_MAX, oX1 = -FLT_MAX, oY0 = FLT_MAX, oY1 = -FLT_MAX;
	// find the inscribed rectangle.
	// the code will likely not work with extreme rotation matrices (R) (>45%)
	for (y = k = 0; y < N; y++)
		for (x = 0; x < N; x++)
		{
			CvPoint2D32f p = pts[k++];
			oX0 = MIN(oX0, p.x);
			oX1 = MAX(oX1, p.x);
			oY0 = MIN(oY0, p.y);
			oY1 = MAX(oY1, p.y);

			if (x == 0)
				iX0 = MAX(iX0, p.x);
			if (x == N - 1)
				iX1 = MIN(iX1, p.x);
			if (y == 0)
				iY0 = MAX(iY0, p.y);
			if (y == N - 1)
				iY1 = MIN(iY1, p.y);
		}
	inner = cv::Rect_<float>(iX0, iY0, iX1 - iX0, iY1 - iY0);
	outer = cv::Rect_<float>(oX0, oY0, oX1 - oX0, oY1 - oY0);
}