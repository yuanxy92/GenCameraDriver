/**
@brief CUDA real time exposure fusion class 
	only two image is support now
@author ShaneYuan
@date Jul 18, 2018
*/

#ifndef __CUDA_EXPOSURE_FUSION_H__
#define __CUDA_EXPOSURE_FUSION_H__

#include <cstdlib>
#include <cstdio>
#include <memory>
#include <thread>

#include <opencv2/opencv.hpp>


// float contrast_weight = 1.0f, float saturation_weight = 1.0f, float exposure_weight = 0.0f);
class MergeMertensImpl
{
public:
	MergeMertensImpl(float _wcon, float _wsat, float _wexp) :
		name("MergeMertens"),
		wcon(_wcon),
		wsat(_wsat),
		wexp(_wexp)
	{
	}

	void process(cv::InputArrayOfArrays src, cv::OutputArrayOfArrays dst, cv::InputArray, cv::InputArray) {
		process(src, dst);
	}

	void process(cv::InputArrayOfArrays src, cv::OutputArray dst)
	{

		std::vector<cv::Mat> images;
		src.getMatVector(images);

		int channels = images[0].channels();
		CV_Assert(channels == 1 || channels == 3);
		cv::Size size = images[0].size();
		int CV_32FCC = CV_MAKETYPE(CV_32F, channels);

		std::vector<cv::Mat> weights(images.size());
		cv::Mat weight_sum = cv::Mat::zeros(size, CV_32F);

		for (size_t i = 0; i < images.size(); i++) {
			cv::Mat img, gray, contrast, saturation, wellexp;
			std::vector<cv::Mat> splitted(channels);

			images[i].convertTo(img, CV_32F, 1.0f / 255.0f);
			if (channels == 3) {
				cvtColor(img, gray, cv::COLOR_RGB2GRAY);
			}
			else {
				img.copyTo(gray);
			}
			split(img, splitted);

			Laplacian(gray, contrast, CV_32F);
			contrast = abs(contrast);

			cv::Mat mean = cv::Mat::zeros(size, CV_32F);
			for (int c = 0; c < channels; c++) {
				mean += splitted[c];
			}
			mean /= channels;

			saturation = cv::Mat::zeros(size, CV_32F);
			for (int c = 0; c < channels; c++) {
				cv::Mat deviation = splitted[c] - mean;
				cv::pow(deviation, 2.0f, deviation);
				saturation += deviation;
			}
			sqrt(saturation, saturation);

			wellexp = cv::Mat::ones(size, CV_32F);
			for (int c = 0; c < channels; c++) {
				cv::Mat expo = splitted[c] - 0.5f;
				cv::pow(expo, 2.0f, expo);
				expo = -expo / 0.08f;
				cv::exp(expo, expo);
				wellexp = wellexp.mul(expo);
			}

			cv::pow(contrast, wcon, contrast);
			cv::pow(saturation, wsat, saturation);
			cv::pow(wellexp, wexp, wellexp);

			weights[i] = contrast;
			if (channels == 3) {
				weights[i] = weights[i].mul(saturation);
			}
			weights[i] = weights[i].mul(wellexp) + 1e-12f;
			weight_sum += weights[i];
		}
		int maxlevel = static_cast<int>(logf(static_cast<float>(std::min<int>(size.width, size.height))) / logf(2.0f));
		std::vector<cv::Mat> res_pyr(maxlevel + 1);

		for (size_t i = 0; i < images.size(); i++) {
			weights[i] /= weight_sum;
			cv::Mat img;
			images[i].convertTo(img, CV_32F, 1.0f / 255.0f);

			std::vector<cv::Mat> img_pyr, weight_pyr;
			buildPyramid(img, img_pyr, maxlevel);
			buildPyramid(weights[i], weight_pyr, maxlevel);

			for (int lvl = 0; lvl < maxlevel; lvl++) {
				cv::Mat up;
				pyrUp(img_pyr[lvl + 1], up, img_pyr[lvl].size());
				img_pyr[lvl] -= up;
			}
			for (int lvl = 0; lvl <= maxlevel; lvl++) {
				std::vector<cv::Mat> splitted(channels);
				split(img_pyr[lvl], splitted);
				for (int c = 0; c < channels; c++) {
					splitted[c] = splitted[c].mul(weight_pyr[lvl]);
				}
				merge(splitted, img_pyr[lvl]);
				if (res_pyr[lvl].empty()) {
					res_pyr[lvl] = img_pyr[lvl];
				}
				else {
					res_pyr[lvl] += img_pyr[lvl];
				}
			}
		}
		for (int lvl = maxlevel; lvl > 0; lvl--) {
			cv::Mat up;
			pyrUp(res_pyr[lvl], up, res_pyr[lvl - 1].size());
			res_pyr[lvl - 1] += up;
		}
		dst.create(size, CV_32FCC);
		res_pyr[0].copyTo(dst.getMat());
	}

	float getContrastWeight() const { return wcon; }
	void setContrastWeight(float val) { wcon = val; }

	float getSaturationWeight() const { return wsat; }
	void setSaturationWeight(float val) { wsat = val; }

	float getExposureWeight() const { return wexp; }
	void setExposureWeight(float val) { wexp = val; }

protected:
	std::string name;
	float wcon, wsat, wexp;
};

class PyramidCUDA {
private:

public:

private:

public:
	PyramidCUDA();
	~PyramidCUDA();

	static int buildPyramidLaplacian(cv::cuda::GpuMat img, 
		std::vector<cv::cuda::GpuMat> & pyrImgs,
		int levels);
};


class ExposureFusion {
private:
	float wcon; 
	float wsat;
	float wexp;

	int imgNum;
	int layerNum;
	std::vector<cv::Mat> weights;
	std::vector<std::vector<cv::cuda::GpuMat>> weightsPyr;
	std::vector<cv::cuda::GpuMat> imgPyr;
	std::vector<cv::cuda::GpuMat> images;
	std::vector<std::vector<cv::cuda::GpuMat>> pyrImgs;
	std::vector<cv::cuda::GpuMat> resPyr;

	int original_width;
	int original_height;
	const int resized_width = 4096;
	const int resized_height = 4096;

	int radius;

public:

private:

public:
	ExposureFusion();
	~ExposureFusion();

	/**
	@brief calculate weight mat to blend hdr result
	@return int
	*/
	int calcWeight(cv::Mat dark, cv::Mat light);

	/**
	@brief fusion two images
	@return int
	*/
	int fusion(cv::cuda::GpuMat dark, cv::cuda::GpuMat light, 
		cv::cuda::GpuMat & fusion);

	int Raman_fusion(cv::cuda::GpuMat &img1, cv::cuda::GpuMat &img2,
		cv::cuda::GpuMat & fusion);

	int Raman_init_filter(int height, int radius = 1);

};

#endif // !__CUDA_EXPOSURE_FUSION_H__
