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
	cap >> frame;
	cap2 >> frame2; 
	//cv::resize(frame, frame, cv::Size(), .25, .25);
	//cv::resize(frame2,frame2,cv::Size(), .25, .25);

	
	Ptr<BackgroundSubtractor> mog = cuda::createBackgroundSubtractorMOG(70);
	GpuMat d_fgmask;
	Ptr<cv::cuda::Filter> gauss = cv::cuda::createGaussianFilter(CV_8UC1, CV_8UC1, Size(5,5), 0);
    Mat fgmask;
    Mat bg_depth;
    Mat depth_map;
	Mat depth_mask;
	GhostElemer elem;
    //------------------------------------------------------------parpare for DIS-----------------------------------------------------------
    int rpyrtype, nochannels, incoltype;
    //different version
    //#if (SELECTCHANNEL==1 | SELECTCHANNEL==2) // use Intensity or Gradient image      
    incoltype = IMREAD_GRAYSCALE;      
    rpyrtype = CV_32FC1;
    nochannels = 1;
    // #elif (SELECTCHANNEL==3) // use RGB image
    // incoltype = IMREAD_COLOR;
    // rpyrtype = CV_32FC3;
    // nochannels = 3;      
    // #endif
  // *** Parse rest of parameters, See oflow.h for definitions.
    if(frame.empty()||frame2.empty())
    {
        cout<<"the first frame is empty!"<<endl;
        return -1;
    }
    depthmap dep(rpyrtype,nochannels,incoltype);
    
	GpuMat d_frame(frame);
	//bg_depth = dep.init_depth(frame,frame2);//should use the output of A stage,just test
	Mat init1 = frame.clone();
	Mat init2 = frame2.clone();
	Mat depth_init = dep.init_depth(init1,init2,elem.flag);
	Mat frame_init = elem.init_frame(frame);
	mog->apply(d_frame, d_fgmask, 0.01);
	int count = 0;
	Mat diff_mask;
	for (;;)
	{
		cap >> frame;
		cap2 >> frame2;
		if (frame.empty())
			break;
		//cv::resize(frame, frame, cv::Size(), .25, .25);
		//cv::resize(frame2,frame2,cv::Size(), .25, .25);
		//frame_o = frame.clone();
		d_frame.upload(frame);
		int64 start = cv::getTickCount();
		//update the model
		mog->apply(d_frame, d_fgmask, 0.01);
		gauss->apply(d_fgmask, d_fgmask);
		d_fgmask.download(fgmask);
		result = elem.Find_location(fgmask,frame,frame2);//get vector<Rect> and mask
		
		depth_map = dep.get_depth(frame,frame2);
		//refine the mask
		diff_mask = elem.refine_mask(frame_init,frame,fgmask);
		depth_mask = dep.update_depth_robust(depth_map,diff_mask);
		dep.refine_depth(depth_mask,diff_mask,result,frame,frame2);//refine the depth and mask,abs here
		elem.res_out(diff_mask,depth_mask);
		//elem.res_out(fgmask,depth_map);
		
		double fps = cv::getTickFrequency() / (cv::getTickCount() - start);
		std::cout << "time : " << 1000/fps << std::endl;
		cout<<"depth update done!"<<endl;
		result.clear();
		count = count + 1;
		if(count == 15)
		{
			imwrite("mask_15.png",diff_mask);
			dep.SavePFMFile(depth_mask,"depth_mask_15.pfm");
		}
		int key = waitKey(10);
		if (key == 27)
			break;
	}
	return 0;
}

