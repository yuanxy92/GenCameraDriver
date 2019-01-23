#ifndef _GHOSTELEMER_H_
#define _GHOSTELEMER_H_
#define Area 500

#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui.hpp"

class GhostElemer
{
public:
    float thresh;
    size_t size;
    int num;
    float time1;
    float time2;
    int flag;
    std::vector< std::vector<cv::Point> > record;
    std::vector<int> table;
    GhostElemer(float thr = 4,size_t sz = 5, int num_ = 4, float t1 = 1.1,float t2 = 1.2,int f = 1):thresh(thr),size(sz),num(num_),time1(t1),time2(t2),flag(f){}
    void ghost_dele(std::vector<cv::Rect> &res_c);
    void ghost_elem_update(std::vector<cv::Rect> &res_c);
    /**
    @brief get the location(rectangle) of the target
    @param cv::Mat&img foreground image
    @return std::vector<cv::Rect> vector of the result rect, also the rectangle mask is getted (img)
    */
    void Find_location(cv::Mat& img,cv::Mat& frame,cv::Mat& frame2);
    std::vector<cv::Mat> Mat_res(std::vector<cv::Rect> res_c,cv::Mat frame,cv::Mat frame2);
    /**
    @brief refine the mask, instead of rectangle, is the shape of the target
    @param cv::Mat frame_init the background
    @param cv::Mat frame current frame
    @param cv::Mat mask;
    @return cv::Mat refined mask
    */
    cv::Mat refine_mask(cv::Mat frame_init,cv::Mat frame,cv::Mat mask);
    void rotate_(cv::Mat& m);
    void rotate_back(cv::Mat& m);
    void init_frame(cv::Mat &img);
    void res_out(cv::Mat& mask,cv::Mat& depth_mask);
    
private:
    bool ghost_range(cv::Point a, cv::Point b);
    void ghost_locate();
    cv::Point get_center(cv::Rect r);
    cv::Rect Large_res(cv::Rect r);
    bool Rect_Intersect(cv::Rect r1, cv::Rect r2);
    cv::Rect Rect_Join(cv::Rect r1, cv::Rect r2);
    std::vector<cv::Point> get_all_center(std::vector<cv::Rect> res_c);

};
#endif
