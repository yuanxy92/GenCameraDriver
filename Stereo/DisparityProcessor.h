/**
@brief DisparityProcessor.h
Disparity map processor, calculate depth from disparity
@author zhu-ty
@date Dec 28, 2018
*/

#ifndef __GENERIC_CAMERA_DRIVER_DISPARITY_PROCESSOR_H__
#define __GENERIC_CAMERA_DRIVER_DISPARITY_PROCESSOR_H__

#include <cuda_runtime.h>

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <surface_functions.h>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <algorithm>
#include <ctime>
#define MAX_DEPTH_VALUE 50000
//#define MIN_DISPARITY_VALUE 1

class DisparityProcessor
{
public:
	static int process_disparity_with_mask(cv::cuda::GpuMat &disparity, cv::cuda::GpuMat &Ki, cv::cuda::GpuMat &depth);
};

#endif //__GENERIC_CAMERA_DRIVER_DISPARITY_PROCESSOR_H__
