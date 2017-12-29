/**
@brief cuda source file of camera utility class
@author: Shane Yuan
@date: Sep 1, 2017
*/

#include "NPPJpegCoderKernel.h"

/*************************************************************************/
/*                     GPU Kernel for NPP Jpeg Coder                     */
/*************************************************************************/
/**
@brief cuda clip value function
@return unsigned char: return value
*/
__device__ unsigned char NPPJpegCoderKernel::clip_value(float x,
	unsigned char min_val, unsigned char  max_val) {
	if (x>max_val) {
		return max_val;
	}
	else if (x<min_val) {
		return min_val;
	}
	else {
		return (uchar)x;
	}
}

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
__global__ void NPPJpegCoderKernel::kernel_bayerRG2pitchYUV(unsigned char* bayer_img,
	unsigned char *Y, unsigned char *U, unsigned char *V, int width,
	int height, int lumin_pitch, int chroma_pitch_U, int chroma_pitch_V) {

	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;

	// debayer
	if (x < width / 2 && y < height / 2) {
		float r = (float)bayer_img[(2 * y) * width + 2 * x];
		float g1 = bayer_img[(2 * y) * width + 2 * x + 1];
		float g2 = bayer_img[(2 * y + 1) * width + 2 * x];
		float b = bayer_img[(2 * y + 1) * width + 2 * x + 1];
		float g = (g1 + g2) / 2.0f;

		// calculate Y
		float Y00 = clip_value(0.299 * r + 0.587 * g1 + 0.114 * b, 0, 255);
		float Y01 = clip_value(0.299 * r + 0.587 * g1 + 0.114 * b, 0, 255);
		float Y10 = clip_value(0.299 * r + 0.587 * g2 + 0.114 * b, 0, 255);
		float Y11 = clip_value(0.299 * r + 0.587 * g2 + 0.114 * b, 0, 255);

		// calculate UV
		float U_val = clip_value(-0.147 * r - 0.289 * g + 0.436 * b + 128, 0, 255);
		float V_val = clip_value(0.615 * r - 0.515 * g - 0.100 * b + 128, 0, 255);

		Y[(2 * y) * lumin_pitch + 2 * x] = Y00;
		Y[(2 * y) * lumin_pitch + 2 * x + 1] = Y01;
		Y[(2 * y + 1) * lumin_pitch + 2 * x] = Y10;
		Y[(2 * y + 1) * lumin_pitch + 2 * x + 1] = Y11;
		U[y * chroma_pitch_U + x] = U_val;
		V[y * chroma_pitch_V + x] = V_val;
	}
}

__global__ void NPPJpegCoderKernel::kernel_test(unsigned char* data, int step, int width, int height,
	cv::cuda::PtrStep<uchar3> img) {
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;

	if (x < width && y < height) {
		uchar r = data[y * step + 3 * x + 0];
		uchar g = data[y * step + 3 * x + 1];
		uchar b = data[y * step + 3 * x + 2];
		img.ptr(y)[x] = make_uchar3(b, g, r);
	}
}

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
int NPPJpegCoderKernel::bayerRG2patchYUV(cv::Mat bayerImg, unsigned char* Y, unsigned char* U,
	unsigned char* V, int lumin_pitch, int chroma_pitch_U, int chroma_pitch_V) {
	int width = bayerImg.cols;
	int height = bayerImg.rows;
	unsigned char* bayer_img_d;
	cudaMalloc(&bayer_img_d, sizeof(unsigned char) * width * height);
	cudaMemcpy(bayer_img_d, bayerImg.data, sizeof(unsigned char) * width * height, cudaMemcpyHostToDevice);

	dim3 dimBlock(32, 32);
	dim3 dimGrid((width / 2 + dimBlock.x - 1) / dimBlock.x, (height / 2 + dimBlock.y - 1) / dimBlock.y);
	kernel_bayerRG2pitchYUV << <dimGrid, dimBlock >> >(bayer_img_d, Y, U, V, width, height,
		lumin_pitch, chroma_pitch_U, chroma_pitch_V);
	
	cudaFree(bayer_img_d);
	return 0;
}


/*************************************************************************/
/*               Host utility function for NPP Jpeg Coder                */
/*************************************************************************/
/**
@brief transfer bgr color image int bayerRG image
@param cv::Mat input: input bgr color image
@return cv::Mat bayerRG image
*/
cv::Mat NPPJpegCoderKernel::bgr2bayerRG(cv::Mat input) {
	cv::Mat bayerRGImg(input.rows, input.cols, CV_8U);
	int height = input.rows;
	int width = input.cols;
	for (size_t i = 0; i < height / 2; i++) {
		for (size_t j = 0; j < width / 2; j++) {
			bayerRGImg.at<uchar>(2 * i, 2 * j) = input.at<cv::Vec3b>
				(2 * i, 2 * j).val[2];
			bayerRGImg.at<uchar>(2 * i, 2 * j + 1) = input.at<cv::Vec3b>
				(2 * i, 2 * j + 1).val[1];
			bayerRGImg.at<uchar>(2 * i + 1, 2 * j) = input.at<cv::Vec3b>
				(2 * i + 1, 2 * j).val[1];
			bayerRGImg.at<uchar>(2 * i + 1, 2 * j + 1) = input.at<cv::Vec3b>
				(2 * i + 1, 2 * j + 1).val[0];
		}
	}
	return bayerRGImg;
}

int NPPJpegCoderKernel::test(unsigned char* data, int step, int width, int height, cv::Mat & img) {
	cv::cuda::GpuMat img_d(height, width, CV_8UC3);

	dim3 dimBlock(32, 32);
	dim3 dimGrid((width + dimBlock.x - 1) / dimBlock.x, (height + dimBlock.y - 1) / dimBlock.y);
	kernel_test << <dimGrid, dimBlock >> >(data, step, width, height, img_d);

	img_d.download(img);

	return 0;
}