#include "opencv2/core/utility.hpp"
#include "opencv2/cudabgsegm.hpp"
#include "opencv2/cudalegacy.hpp"
#include "opencv2/video.hpp"
#include "opencv2/cudawarping.hpp"
#include "opencv2/cudaarithm.hpp"
#include "opencv2/cudafilters.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <string>
#include <sys/time.h>
#include <fstream>
#include "depthmap.h"    
#include "oflow.h"
#include "GhostElemer.h"

#define rotate_req 0

using namespace std;
using namespace cv;
using namespace cv::cuda;
//input : ./run_DE_INT test2.avi test2b.avi
//output:   vector<Rect> result, Mat depth_map
int main(int argc, char** argv)
{
	char *file = argv[1];//video input 1
	char *file2 = argv[2];//video input 2
	vector<Rect> result;
	bool useCamera = false;
	VideoCapture cap;
	VideoCapture cap2;
	if (useCamera)
		cap.open(0);
	else
		{
			cap.open(file);
			cap2.open(file2);
		}
	if (!cap.isOpened() || !cap2.isOpened())
	{
		cerr << "can not open camera or video file" << endl;
		return -1;
	}
	Mat frame;
	Mat frame2;
	Mat frame_;
	cap >> frame;
	cap2 >> frame2; 
	
	//cv::resize(frame,frame_,cv::Size(), .5 , .5 );
	cv::resize(frame, frame, cv::Size(), .25, .25);
	cv::resize(frame2,frame2,cv::Size(), .25, .25);
	
	
	Ptr<BackgroundSubtractor> mog = cuda::createBackgroundSubtractorMOG(70);
	GpuMat d_fgmask;
	GpuMat d_bgimg;
	Ptr<cv::cuda::Filter> gauss = cv::cuda::createGaussianFilter(CV_8UC1, CV_8UC1, Size(5,5), 0);
    Mat fgmask;
    Mat bg_depth;
	Mat bg;
    Mat depth_map;
	Mat depth_mask;
	GhostElemer elem;
    //------------------------------------------------------------parpare for DIS-----------------------------------------------------------
    int rpyrtype, nochannels, incoltype;
         
    incoltype = IMREAD_GRAYSCALE;      
    rpyrtype = CV_32FC1;
    nochannels = 1;
    
  // *** Parse rest of parameters, See oflow.h for definitions.
    if(frame.empty()||frame2.empty())
    {
        cout<<"the first frame is empty!"<<endl;
        return -1;
    }
    depthmap dep(rpyrtype,nochannels,incoltype);
    
	Mat init1 = frame.clone();
	Mat init2 = frame2.clone();
	Mat depth_init = dep.init_depth(init1,init2,elem.flag);
	
	GpuMat d_frame;
	int count = 0;
	Mat diff_mask;
	for (;;)
	{
		cap >> frame;
		cap2 >> frame2;
		if (frame.empty())
			break;
		
		//cv::resize(frame,frame_,cv::Size(),.5,.5);
		//cv::resize(frame, frame, cv::Size(), .25, .25);
		//cv::resize(frame2,frame2,cv::Size(), .25, .25);
		
		//d_frame.upload(frame_);
		d_frame.upload(frame);
		int64 start = cv::getTickCount();
		//update the model
		mog->apply(d_frame, d_fgmask, 0.01);
		mog->getBackgroundImage(d_bgimg);
		gauss->apply(d_fgmask, d_fgmask);
		d_fgmask.download(fgmask);
		imshow("fgmask",fgmask);
		d_bgimg.download(bg);//get the background
		elem.init_frame(bg);//rotate the background
		//cv::resize(fgmask,fgmask,cv::Size(),.5,.5);
		//cv:resize(bg,bg,cv::Size(),.5,.5);
		result = elem.Find_location(fgmask,frame,frame2);//get vector<Rect> and mask
		imwrite("./frame1/frame1_" + std::to_string(count) + ".png",frame);
		imwrite("./frame2/frame2_" + std::to_string(count) + ".png",frame2);
	
		depth_map = dep.get_depth(frame,frame2);
		char buffer1[80];
		sprintf(buffer1,"./origin_dep/dep_%d.pfm",count);
		dep.SavePFMFile(depth_map,buffer1);
		//refine the mask
		
		diff_mask = elem.refine_mask(bg,frame,fgmask);
		depth_mask = dep.update_depth_robust(depth_map,diff_mask);
		dep.refine_depth(depth_mask,diff_mask,result,frame,frame2);//refine the depth and mask,abs here
		elem.res_out(diff_mask,depth_mask);
		imshow("mask",diff_mask);
		
		double fps = cv::getTickFrequency() / (cv::getTickCount() - start);
		std::cout << "time : " << 1000/fps << std::endl;
		cout<<"depth update done!"<<endl;
		result.clear();
		count = count + 1;
		
		imwrite("./mask/mask_"+std::to_string(count)+".png",diff_mask);
		char buffer[80];
		sprintf(buffer,"./depth/dep_%d.pfm",count);
		dep.SavePFMFile(depth_mask,buffer);
		int key = waitKey(10);
		if (key == 27)
			break;
	}
	return 0;
}

