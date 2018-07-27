/**
@brief CUDA real time exposure fusion class
@author ShaneYuan
@date Jul 18, 2018
*/

#include "ExposureFusion.h"

PyramidCUDA::PyramidCUDA() {};
PyramidCUDA::~PyramidCUDA() {};

int PyramidCUDA::buildPyramidLaplacian(cv::cuda::GpuMat img,
	std::vector<cv::cuda::GpuMat> & pyrImgs, int levels) {
	img.convertTo(pyrImgs[0], CV_32F);
	for (int i = 0; i < levels; ++i) {
		cv::cuda::pyrDown(pyrImgs[i], pyrImgs[i + 1]);
	}
	cv::cuda::GpuMat tmp, tmp2;
	for (int i = 0; i < levels; ++i) {
		cv::cuda::pyrUp(pyrImgs[i + 1], tmp);
		cv::cuda::subtract(pyrImgs[i], tmp, pyrImgs[i]);
	}
	return 0;
}

ExposureFusion::ExposureFusion() {
	wcon = 1.0f;
	wsat = 1.0f;
	wexp = 0.0f;
}
ExposureFusion::~ExposureFusion() {}

/**
@brief calculate weight mat to blend hdr result
@return int
*/
int ExposureFusion::calcWeight(cv::Mat dark, cv::Mat light) {

	// resize
	cv::Mat dark_resized, light_resized;
	this->original_width = dark.cols;
	this->original_height = dark.rows;
	cv::resize(dark, dark_resized, cv::Size(resized_width, resized_height));
	cv::resize(light, light_resized, cv::Size(resized_width, resized_height));
	// prepare variables
	imgNum = 2;
	layerNum = 11;
	weights.resize(imgNum);
	weightsPyr.resize(imgNum);
	for (size_t i = 0; i < imgNum; i++) {
		weightsPyr[i].resize(layerNum);
	}
	std::vector<cv::Mat> images(imgNum);
	images[0] = dark_resized;
	images[1] = light_resized;
	cv::Size size = images[0].size();
	cv::Mat weight_sum(size, CV_32F);
	weight_sum.setTo(0);

	// calcualte weights
	int channels = 3;
	for (size_t i = 0; i < images.size(); i++) {
		cv::Mat img, gray, contrast, saturation, wellexp;
		std::vector<cv::Mat> splitted(channels);

		images[i].convertTo(img, CV_32F, 1.0f / 255.0f);
		if (channels == 3) {
			cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
		}
		else {
			img.copyTo(gray);
		}
		cv::split(img, splitted);

		cv::Laplacian(gray, contrast, CV_32F);
		contrast = cv::abs(contrast);

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
		cv::sqrt(saturation, saturation);

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
	for (size_t i = 0; i < images.size(); i++) {
		weights[i] /= weight_sum;
	}

	// build weight pyramid
	for (size_t i = 0; i < images.size(); i++) {
		std::vector<cv::Mat> weight_pyr;
		buildPyramid(weights[i], weight_pyr, layerNum - 1);
		for (size_t j = 0; j < weight_pyr.size(); j++) {
			weightsPyr[i][j].upload(weight_pyr[j]);
			cv::cuda::cvtColor(weightsPyr[i][j], weightsPyr[i][j], cv::COLOR_GRAY2BGR);
		}
	}

	this->images.resize(imgNum);
	this->pyrImgs.resize(imgNum);
	for (size_t i = 0; i < imgNum; i++) {
		this->pyrImgs[i].resize(layerNum);
	}

	return 0;
}

/**
@brief fusion two images
@return int
*/
int ExposureFusion::fusion(cv::cuda::GpuMat dark, cv::cuda::GpuMat light,
	cv::cuda::GpuMat & fusion) {

	cv::cuda::GpuMat dark_resized, light_resized;

	cv::cuda::resize(dark, dark_resized, cv::Size(resized_width, resized_height));
	cv::cuda::resize(light, light_resized, cv::Size(resized_width, resized_height));

	images[0] = dark_resized;
	images[1] = light_resized;
	PyramidCUDA::buildPyramidLaplacian(images[0], pyrImgs[0], layerNum - 1);
	PyramidCUDA::buildPyramidLaplacian(images[1], pyrImgs[1], layerNum - 1);

	if (resPyr.size() == 0) {
		resPyr.resize(pyrImgs[0].size());
		for (size_t j = 0; j < layerNum; j++) {
			resPyr[j].create(pyrImgs[0][j].size(), CV_32FC3);
		}
	}
	for (size_t j = 0; j < layerNum; j++) {
		resPyr[j].setTo(cv::Scalar(0, 0, 0));
	}

	for (size_t i = 0; i < images.size(); i++) {
		for (int lvl = 0; lvl < layerNum; lvl++) {
			cv::cuda::multiply(pyrImgs[i][lvl], weightsPyr[i][lvl], pyrImgs[i][lvl]);
			cv::cuda::add(resPyr[lvl], pyrImgs[i][lvl], resPyr[lvl]);
		}
	}
	for (int lvl = layerNum - 1; lvl > 0; lvl--) {
		cv::cuda::GpuMat up;
		cv::cuda::pyrUp(resPyr[lvl], up);
		cv::cuda::add(resPyr[lvl - 1], up, resPyr[lvl - 1]);
	}


	//fusion = resPyr[0];
	fusion.create(this->original_height, this->original_width, CV_8UC3);
	cv::cuda::GpuMat resized_fusion;
	cv::cuda::resize(resPyr[0], resized_fusion, cv::Size(this->original_width, this->original_height));
	resized_fusion.convertTo(fusion, CV_8UC3);
	return 0;
}