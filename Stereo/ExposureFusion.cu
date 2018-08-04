/**
@brief Exposure Fusion CUDA Kernal File
	only two image is support now
@author zhu-ty
@date Aug 4, 2018
*/

#define TILE_X 16
#define TILE_Y 16

//filter ref:https://github.com/aashikgowda/Bilateral-Filter-CUDA/blob/master/kernel.cu

#include "ExposureFusion.h"

// cuda
// CUDA includes
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
#include <opencv2/opencv.hpp>

__constant__ float cGaussian[64];

void updateGaussian(int r, float sd)
{
	float fGaussian[64];
	for (int i = 0; i < 2 * r + 1; i++)
	{
		float x = i - r;
		fGaussian[i] = expf(-(x*x) / (2 * sd*sd));
	}
	cudaMemcpyToSymbol(cGaussian, fGaussian, sizeof(float)*(2 * r + 1));
}

// Gaussian function for range difference
__device__ inline float gaussian(float x, float sigma)
{
	return __expf(-(powf(x, 2)) / (2 * powf(sigma, 2)));
}

__global__ void cuda_fusionRaman(cv::cuda::PtrStep<uchar3> input1, cv::cuda::PtrStep<uchar3> input2,
	cv::cuda::PtrStep<uchar3> input1_XYZ, cv::cuda::PtrStep<uchar3> input2_XYZ,
	cv::cuda::PtrStep<uchar3> output, int width, int height,
	float C, int r, float sI, float sS)
{
	// Initialize global Tile indices along x,y and xy
	int txIndex = __mul24(blockIdx.x, TILE_X) + threadIdx.x;
	int tyIndex = __mul24(blockIdx.y, TILE_Y) + threadIdx.y;

	// If within image size
	if ((txIndex < width) && (tyIndex < height))
	{
		uchar3 ip1 = input1(tyIndex, txIndex);
		uchar3 ip2 = input2(tyIndex, txIndex);
		//unsigned char lum_ip1 = input1_XYZ(tyIndex, txIndex).y;
		//unsigned char lum_ip2 = input2_XYZ(tyIndex, txIndex).y;
		//float lum = 0.2126f * ip1.x + 0.7152 * ip1.y + 0.0722 * ip1.z; //0~255
		float weight1 = 0.0f;
		float weight2 = 0.0f;
		float total = 0.0f;

		{
			float iFiltered = 0;
			float wP = 0;
			float centrePx = input1_XYZ(tyIndex, txIndex).y;
			for (int dy = -r; dy <= r; dy++)
			{
				for (int dx = -r; dx <= r; dx++)
				{
					float currPx = (txIndex + dx < width && txIndex + dx >= 0 && tyIndex + dy < height && tyIndex + dy >= 0) ?
						(input1_XYZ(tyIndex + dy, txIndex + dx).y) : (0.0f);

					float w = (cGaussian[dy + r] * cGaussian[dx + r]) * gaussian(centrePx - currPx, sI);
					iFiltered += w * currPx;
					wP += w;
				}
			}
			weight1 = fabs(iFiltered / wP - centrePx) / 255.0f + C;
		}
		{
			float iFiltered = 0;
			float wP = 0;
			float centrePx = input2_XYZ(tyIndex, txIndex).y;
			for (int dy = -r; dy <= r; dy++)
			{
				for (int dx = -r; dx <= r; dx++)
				{
					float currPx = (txIndex + dx < width && txIndex + dx >= 0 && tyIndex + dy < height && tyIndex + dy >= 0) ?
						(input2_XYZ(tyIndex + dy, txIndex + dx).y) : (0.0f);

					float w = (cGaussian[dy + r] * cGaussian[dx + r]) * gaussian(centrePx - currPx, sI);
					iFiltered += w * currPx;
					wP += w;
				}
			}
			weight2 = fabs(iFiltered / wP - centrePx) / 255.0f + C;
		}
		total = weight1 + weight2;

		unsigned char ansx = (unsigned char)((weight1 * ip1.x + weight2 * ip2.x) / total);
		unsigned char ansy = (unsigned char)((weight1 * ip1.y + weight2 * ip2.y) / total);
		unsigned char ansz = (unsigned char)((weight1 * ip1.z + weight2 * ip2.z) / total);

		//printf("FromCudaKernel::ansxyz = %d %d %d\n", ansx,ansy,ansz);

		output(tyIndex, txIndex) = make_uchar3(ansx, ansy, ansz);

	}
}

int ExposureFusion::Raman_fusion(cv::cuda::GpuMat & img1, cv::cuda::GpuMat & img2, cv::cuda::GpuMat & fusion)
{
	int height = img1.rows;
	int width = img1.cols;
	fusion.create(height, width, CV_8UC3);

	float C = 70.0 / 255.0;
	float K1 = 1.0;
	float K2 = 1.0 / 10.0;

	//TODO:should search the img. ref:https://docs.opencv.org/3.4/d5/de6/group__cudaarithm__reduce.html#ga5cacbc2a2323c4eaa81e7390c5d9f530
	float imageStackMax = 255.0;
	float imageStackMin = 0.0;

	float sigma_r = K2 * (imageStackMax - imageStackMin);
	float sigma_s = MIN(height, width);

	cv::cuda::GpuMat XYZ1, XYZ2;
	cv::cuda::cvtColor(img1, XYZ1, cv::COLOR_RGB2XYZ);
	cv::cuda::cvtColor(img2, XYZ2, cv::COLOR_RGB2XYZ);

	dim3 block(TILE_X, TILE_Y);
	dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
	cuda_fusionRaman << <grid, block >> > (img1, img2, XYZ1, XYZ2, fusion, width, height, C, radius, sigma_r, sigma_s);

	return 0;
}


int ExposureFusion::Raman_init_filter(int height, int radius)
{
	this->radius = radius;
	updateGaussian(radius, height);
	return 0;
}