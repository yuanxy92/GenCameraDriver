/**
@brief depthmap.h 
class for depth map update
@author zhang-jn
@date Dec 26, 2018
*/
#ifndef _DEPTHMAP_H_
#define _DEPTHMAP_H_
#define disp_danger 55//the confindence of the disparity is low
#define largest_diff 0
#define update_thresh 0 //do not update, if the max disparity is smalller 

#include <opencv2/opencv.hpp>
#include "oflow.h"


class depthmap
{
public:
    //parameter for DIS
    int lv_f, lv_l, maxiter, miniter, patchsz, patnorm, costfct, tv_innerit, tv_solverit, verbosity;
    float mindprate, mindrrate, minimgerr, poverl, tv_alpha, tv_gamma, tv_delta, tv_sor;
    bool usefbcon, usetvref;
    char* outfile;
    int rpyrtype,nochannels,incoltype;
	depthmap(int rpyrtype = CV_32FC1, int nochannels = 1, int incoltype = cv::IMREAD_GRAYSCALE);
    /**
    @brief get the depthmap
    @param cv::Mat input frame left(up)
    @param cv::Mat input2 frame right(down)
    @return cv::Mat CV_32FC1 depthmap 
    */
    cv::Mat get_depth(cv::Mat input1,cv::Mat input2);
    //cv::Mat update_depth(cv::Mat& bg_depth,std::vector<cv::Rect> result,cv::Mat& frame,cv::Mat& frame2);
    cv::Mat update_depth_robust(cv::Mat& depth_map,cv::Mat mask); 
    cv::Mat init_depth(cv::Mat init1,cv::Mat init2,int &flag);
    void SavePFMFile(cv::Mat& img, const char* filename);
    void SaveFlowFile(cv::Mat& img, const char* filename);
    //void update_mat(Mat input1,Mat input2);

    /**
	@brief refine the depth value, especailly correct the wrong value. here we suppose for the same pixel the right coordinate is smaller
    @brief if the disparity is not very small and the max-mean is not very large, do not refine
    @brief if the disparity is too small, delete the mask and the masked depth
	@param cv::Mat& mask_depth to get the search area
	@param cv::Mat& mask  mask
    @param vector<Rect> rectangle
	@param cv::Mat& frame image left
    @param cv::Mat& frame2 image right
	@return refined mask_depth
	*/
    void refine_depth(cv::Mat& mask_depth,cv::Mat& mask,cv::Mat& frame,cv::Mat& frame2);
    cv::Mat rotate_(cv::Mat m);
    cv::Mat rotate_back(cv::Mat m);
private:
    void ConstructImgPyramide(const cv::Mat & img_ao_fmat, cv::Mat * img_ao_fmat_pyr, cv::Mat * img_ao_dx_fmat_pyr, cv::Mat * img_ao_dy_fmat_pyr, const float ** img_ao_pyr, const float ** img_ao_dx_pyr, const float ** img_ao_dy_pyr, const int lv_f, const int lv_l, const int rpyrtype, const bool getgrad, const int imgpadding, const int padw, const int padh);
    int AutoFirstScaleSelect(int imgwidth, int fratio, int patchsize);
    
    /**
    @brief match the object in another image, to get the disparity standard
	@param int x_forward = larger width - smaller width
    @param cv::Mat& temp 
    @param cv::Mat& temp_area search area
	@return int stantdard dispartiy
	*/
    int pattern_match(int x_forward,int flag,cv::Mat temp,cv::Mat temp_area);
    
};

#endif
