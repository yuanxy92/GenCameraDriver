/**
@brief Generic Camera Driver Class
Implementation of Stereo Camera Cudapart
@author zhu-ty
@date Dec 28, 2018
*/

#include "DisparityProcessor.h"
#include "Exceptions.h"
#include "helper_cuda.h"

__global__ void calculate_depth(
	cv::cuda::PtrStep<float> disparity,
	cv::cuda::PtrStep<float> Ki,
	cv::cuda::PtrStep<ushort> depth,
	int width, int height)
{
	unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
	unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

	if (x < width && y < height) 
	{
		float dis = disparity.ptr(y)[x];
		float A = Ki.ptr(0)[0];
		float B = Ki.ptr(0)[1];
		float C = Ki.ptr(0)[2];
		float D = Ki.ptr(0)[3];
		float E = Ki.ptr(0)[4];
		if(dis < 1) //very close to 0
			depth.ptr(y)[x] = 0;
		else
		{
			float divdis = 1.0f / dis;
			float3 vec;
			vec.x = A * divdis * x + B * divdis;
			vec.y = C * divdis * y + D * divdis;
			vec.z = divdis;
			float norm = sqrt(vec.x * vec.x  + vec.y * vec.y + vec.z * vec.z);
			float dep = norm * E;
			if(dep > MAX_DEPTH_VALUE)
				depth.ptr(y)[x] = MAX_DEPTH_VALUE;
			else
				depth.ptr(y)[x] = 	__float2uint_rd(dep);
		}
	}
}

int DisparityProcessor::process_disparity_with_mask(cv::cuda::GpuMat &disparity, cv::cuda::GpuMat &Ki, cv::cuda::GpuMat &depth)
{
	depth.create(disparity.size(), CV_16UC1);
	dim3 dimBlock(32, 32);
	dim3 dimGrid((disparity.cols + dimBlock.x - 1) / dimBlock.x,
			    (disparity.rows + dimBlock.y - 1) / dimBlock.y);
	calculate_depth << <dimGrid, dimBlock>> >(disparity, Ki, depth,disparity.cols, disparity.rows);
	checkCudaErrors(cudaDeviceSynchronize());
	return 0;
}
