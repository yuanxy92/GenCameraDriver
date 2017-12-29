/**
@brief cuda header file of camera utility class
@author: Shane Yuan
@date: Sep 1, 2017
*/

#ifndef __NPPJPEG_CODER_KERNEL_H__
#define __NPPJPEG_CODER_KERNEL_H__

// basic 
#include <iostream>
#include <cstdlib>
#include <fstream>

// C++ 11 parallel 
#include <thread>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>

// cuda
#ifdef _WIN32
#include <windows.h>
#endif
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <device_launch_parameters.h>
#include <surface_functions.h>

namespace NPPJpegCoderKernel {
	
	/*************************************************************************/
	/*                     GPU Kernel for NPP Jpeg Coder                     */
	/*************************************************************************/
	/**
	@brief cuda clip value function
	@return unsigned char: return value
	*/
	__device__ unsigned char clip_value(float x, unsigned char min_val,
		unsigned char  max_val);

	/**
	@brief GPUkernel function for change bayerRG image to npp data structure
	YUV411 setting
	@param unsigned char* bayer_img: input bayerRG image
	@param unsigned char* Y: output luminance channel Y of image
	@param unsigned char* U: output color channel U of image
	@param unsigned char* V: output color channel U of image
	@param int width: input image width
	@param int height: input image height
	@param int y_pitch: input pitch for luminance channel Y of image
	@param int chroma_pitch: input pitch for luminance channel U of image
	@param int chroma_pitch: input pitch for luminance channel V of image
	*/
	__global__ void kernel_bayerRG2pitchYUV(unsigned char* bayer_img,
		unsigned char *Y, unsigned char *U, unsigned char *V, int width,
		int height, int lumin_pitch, int chroma_pitch_U, int chroma_pitch_V);

	__global__ void kernel_test(unsigned char* data, int step, int width, int height, cv::cuda::PtrStep<uchar3> img);


	/*************************************************************************/
	/*                   GPU function for NPP Jpeg Coder                     */
	/*************************************************************************/
	/**
	@brief function for change bayerRG image to npp data structure
	YUV411 setting
	@param unsigned cv::Mat bayer_img: input bayerRG image
	@param unsigned char* Y: output luminance channel Y of image
	@param unsigned char* U: output color channel U of image
	@param unsigned char* V: output color channel U of image
	@param int y_pitch: input pitch for luminance channel Y of image
	@param int chroma_pitch_U: input pitch for luminance channel U of image
	@param int chroma_pitch_V: input pitch for luminance channel V of image
	*/
	int bayerRG2patchYUV(cv::Mat bayerImg, unsigned char* Y,
		unsigned char* U, unsigned char* V, int lumin_pitch, 
		int chroma_pitch_U, int chroma_pitch_V);
	
	/*************************************************************************/
	/*               Host utility function for NPP Jpeg Coder                */
	/*************************************************************************/
	/**
	@brief transfer bgr color image int bayerRG image
	@param cv::Mat input: input bgr color image
	@return cv::Mat bayerRG image
	*/
	cv::Mat bgr2bayerRG(cv::Mat input);

	int test(unsigned char* data, int step, int width, int height, cv::Mat & img);
}

#endif