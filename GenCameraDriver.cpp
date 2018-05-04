/**
@brief Generic Camera Driver Class
@author Shane Yuan
@date Dec 29, 2017
*/

#include "GenCameraDriver.h"
#include <time.h>
#include <algorithm>
#include <functional>   // std::minus
#include <numeric>      // std::accumulate

// include NPPJpegCoder
#include "NPPJpegCoder.h"

namespace cam {

	GenCamera::GenCamera() : isInit(false), isCapture(false),
		isVerbose(false), bufferType(GenCamBufferType::Raw),
		camPurpose(GenCamCapturePurpose::Streaming),
		JPEGQuality(75), sizeRatio(0.12) {}
	GenCamera::~GenCamera() {}

	/**
	@brief set verbose
	@param bool isVerbose: true, verbose mode, output many infomations
	for debugging
	@return int
	*/
	int GenCamera::setVerbose(bool isVerbose) {
		this->isVerbose = isVerbose;
		return 0;
	}

	/**
	@brief set buffer type
	@param GenCamBufferType type: buffer type
	@return int
	*/
	int GenCamera::setCamBufferType(GenCamBufferType type) {
		this->bufferType = type;
		return 0;
	}

	/**
	@brief set jpeg compression quality
	@param int quality: JPEG compression quality (1 - 100)
	@param float sizeRatio: expected compression ratio used for
	pre-malloc memory
	@return int
	*/
	int GenCamera::setJPEGQuality(int quality, float sizeRatio) {
		this->JPEGQuality = quality;
		this->sizeRatio = sizeRatio;
		return 0;
	}

	/**
	@brief set capture purpose
	@param GenCamCapturePurpose camPurpose: purpose, for streaming or recording
	@return int
	*/
	int GenCamera::setCapturePurpose(GenCamCapturePurpose camPurpose) {
		this->camPurpose = camPurpose;
		return 0;
	}

	/*************************************************************/
	/*        function to save capture images to files           */
	/*************************************************************/
	/**
	@brief save captured images to dir
	@param std::string dir: input dir to save images
	@return int
	*/
	int GenCamera::saveImages(std::string dir) {
		if (this->bufferType == GenCamBufferType::JPEG) {
			std::vector<cam::GenCamInfo> _camInfos;
			this->getCamInfos(_camInfos);
			SysUtil::mkdir(dir);
			for (size_t i = 0; i < this->cameraNum; i++) {
				// init npp jpeg coder
				npp::NPPJpegCoder coder;
				coder.init(camInfos[i].width, camInfos[i].height, JPEGQuality);
				cv::cuda::GpuMat img_d(camInfos[i].height, camInfos[i].width, CV_8UC3);
				cv::Mat img(camInfos[i].height, camInfos[i].width, CV_8UC3);
				for (size_t j = 0; j < this->bufferSize; j++) {
					char outname[256];
					coder.decode(reinterpret_cast<uchar*>(this->bufferImgs[j][i].data),
						this->bufferImgs[j][i].length,
						img_d, 0);
					img_d.download(img);
					sprintf(outname, "%s/%s_%02d_%05d.jpg", dir.c_str(), _camInfos[i].sn.c_str(), i, j);
					cv::imwrite(outname, img);
				}
			}
		}
		else {
			std::vector<cam::GenCamInfo> _camInfos;
			this->getCamInfos(_camInfos);
			SysUtil::mkdir(dir);
			for (size_t i = 0; i < this->cameraNum; i++) {
				for (size_t j = 0; j < this->bufferSize; j++) {
					cv::cuda::GpuMat bayer_img_d(camInfos[i].height, camInfos[i].width, CV_8U);
					cv::cuda::GpuMat rgb_img_mat_d(camInfos[i].height, camInfos[i].width, CV_8UC3);
					cv::Mat bayer_img(camInfos[i].height, camInfos[i].width, CV_8U, 
						reinterpret_cast<uchar*>(this->bufferImgs[j][i].data));
					cv::Mat rgb_img_mat;
					bayer_img_d.upload(bayer_img);
					cv::cuda::demosaicing(bayer_img_d, rgb_img_mat_d, 
						npp::bayerPatternNPP2CVBGR(static_cast<NppiBayerGridPosition>(
							_camInfos[i].bayerPattern)), -1);
					rgb_img_mat_d.download(rgb_img_mat);
					char outname[256];
					sprintf(outname, "%s/%s_%02d_%05d.png", dir.c_str(), _camInfos[i].sn.c_str(), i, j);
					cv::imwrite(outname, rgb_img_mat);
				}
			}
		}
		return 0;
	}

	/**
	@brief save captured videos to dir
	@param std::string dir: input dir to save videos
	@return int
	*/
	int GenCamera::saveVideos(std::string dir) {
		if (this->bufferType == GenCamBufferType::JPEG) {
			SysUtil::mkdir(dir);
			for (size_t i = 0; i < this->cameraNum; i++) {
				// init npp jpeg coder
				npp::NPPJpegCoder coder;
				coder.init(camInfos[i].width, camInfos[i].height, JPEGQuality);
				// init video parameter
				std::string videoname = cv::format("%s/cam_%02d.avi", dir.c_str(), i);
				cv::VideoWriter writer(videoname, cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 
					camInfos[i].fps, cv::Size(camInfos[i].width, camInfos[i].height), true);
				cv::cuda::GpuMat img_d(camInfos[i].height, camInfos[i].width, CV_8UC3);
				cv::Mat img(camInfos[i].height, camInfos[i].width, CV_8UC3);
				for (size_t j = 0; j < this->bufferSize; j++) {
					coder.decode(reinterpret_cast<uchar*>(this->bufferImgs[j][i].data), 
						this->bufferImgs[j][i].length,
						img_d, 0);
					img_d.download(img);
					writer << img;
					//char outname[256];
					//sprintf(outname, "%s/%02d_%05d.jpg", dir.c_str(), i, j);
					//cv::imwrite(outname, img);
				}
				writer.release();
				// release npp jpeg coder
				coder.release();
			}
		}
		else {
			std::vector<cam::GenCamInfo> _camInfos;
			this->getCamInfos(_camInfos);
			SysUtil::mkdir(dir);
			for (size_t i = 0; i < this->cameraNum; i++) {
				// init video parameter
				std::string videoname = cv::format("%s/cam_%02d.avi", dir.c_str(), i);
				cv::VideoWriter writer(videoname, cv::VideoWriter::fourcc('D', 'I', 'V', 'X'),
					camInfos[i].fps, cv::Size(camInfos[i].width, camInfos[i].height), true);
				for (size_t j = 0; j < this->bufferSize; j++) {
					cv::cuda::GpuMat bayer_img_d(camInfos[i].height, camInfos[i].width, CV_8U);
					cv::cuda::GpuMat rgb_img_mat_d(camInfos[i].height, camInfos[i].width, CV_8UC3);
					cv::Mat bayer_img(camInfos[i].height, camInfos[i].width, CV_8U,
						reinterpret_cast<uchar*>(this->bufferImgs[j][i].data));
					cv::Mat rgb_img_mat;
					bayer_img_d.upload(bayer_img);
					cv::cuda::demosaicing(bayer_img_d, rgb_img_mat_d,
						npp::bayerPatternNPP2CVBGR(static_cast<NppiBayerGridPosition>(
							_camInfos[i].bayerPattern)), -1);
					rgb_img_mat_d.download(rgb_img_mat);
					writer << rgb_img_mat;
				}
				writer.release();
			}
		}
		return 0;
	}

	/*************************************************************/
	/*    function to set mapping vector of capture function     */
	/*                and function to capture images             */
	/*         old function will be deprecated in the future     */
	/*************************************************************/
	/**
	@brief set mapping vector of capture function
	@param std::vector<size_t> mappingVector: input mapping vector
	@return int
	*/
	int GenCamera::setMappingVector(std::vector<size_t> mappingVector) {
		this->mappingVector = mappingVector;
		return 0;
	}

	/**
	@brief capture one frame
	@param std::vector<Imagedata> & imgs: output captured images
	if in single mode, memory of image mats should be malloced
	before using this function
	@return int
	*/
	int GenCamera::captureFrame(std::vector<Imagedata> & imgs) {
		if (captureMode == GenCamCaptureMode::Continous ||
			captureMode == GenCamCaptureMode::ContinousTrigger) {
			// get images from buffer
			for (size_t camInd = 0; camInd < this->cameraNum; camInd++) {
				int index = (thBufferInds[camInd] - 1 + bufferSize) % bufferSize;
				imgs[camInd] = bufferImgs[index][camInd];
			}
			// increase buffer indices for file camera
			if (this->camModel == cam::CameraModel::File) {
				for (size_t camInd = 0; camInd < this->cameraNum; camInd++) {
					thBufferInds[camInd] = (thBufferInds[camInd] + 1) % bufferSize;
				}
			}
		}
		else if (captureMode == GenCamCaptureMode::Single ||
			captureMode == GenCamCaptureMode::SingleTrigger) {
			SysUtil::errorOutput("Single mode is not implemented yet !");
			exit(-1);
		}
		return 0;
	}

	/**
	@brief capture one frame with Mapping
	@param std::vector<Imagedata> & imgs: output captured images
	if in single mode, memory of image mats should be malloced
	before using this function
	@return int
	*/
	int GenCamera::captureFrameWithMapping(std::vector<Imagedata> & imgs) {
		size_t camInd;
		if (captureMode == GenCamCaptureMode::Continous ||
			captureMode == GenCamCaptureMode::ContinousTrigger) {
			// get images from buffer
			for (size_t i = 0; i < this->cameraNum; i++) {
				camInd = mappingVector[i];
				int index = (thBufferInds[camInd] - 1 + bufferSize) % bufferSize;
				imgs[i] = bufferImgs[index][camInd];
			}

		}
		else if (captureMode == GenCamCaptureMode::Single ||
			captureMode == GenCamCaptureMode::SingleTrigger) {
			SysUtil::errorOutput("Single mode is not implemented yet !");
			exit(-1);
		}
		return 0;
	}

	/**
	@brief get camera infos list with mapping
	@param std::vector<cam::GenCamInfo> & camInfos: output camera info list
	@return int
	*/
	int GenCamera::getCameraInfoListsWithMapping(std::vector<cam::GenCamInfo> & camInfos) {
		size_t camInd;
		// get camera infos without mapping
		std::vector<cam::GenCamInfo> camInfosWoMapping;
		this->getCamInfos(camInfosWoMapping);
		camInfos.clear();
		camInfos.resize(this->cameraNum);
		for (size_t i = 0; i < this->cameraNum; i++) {
			camInd = mappingVector[i];
			camInfos[i] = camInfosWoMapping[camInd];
		}
		return 0;
	}

	/*************************************************************/
	/*                function to capture images                 */
	/*************************************************************/
	/**
	@brief capture one frame
	@param std::vector<Imagedata> & refImgs: output reference images
	@param std::vector<Imagedata> & localImgs: output localview images
	@param std::vector<int> refInds: input reference indices
	@param std::vector<int> localInds: input local indices
	@return int
	*/
	int GenCamera::captureFrame(std::vector<Imagedata> & refImgs,
		std::vector<Imagedata> & localImgs,
		std::vector<int> refInds,
		std::vector<int> localInds) {
		size_t camInd;
		if (captureMode == GenCamCaptureMode::Continous ||
			captureMode == GenCamCaptureMode::ContinousTrigger) {
			// get refernce images from buffer
			for (size_t i = 0; i < refInds.size(); i++) {
				camInd = refInds[i];
				int index = (thBufferInds[camInd] - 1 + bufferSize) % bufferSize;
				refImgs[i] = bufferImgs[index][camInd];
			}
			// get local images from buffer
			for (size_t i = 0; i < localInds.size(); i++) {
				camInd = localInds[i];
				int index = (thBufferInds[camInd] - 1 + bufferSize) % bufferSize;
				localImgs[i] = bufferImgs[index][camInd];
			}
			// increase buffer indices for file camera
			if (this->camModel == cam::CameraModel::File) {
				for (size_t camInd = 0; camInd < this->cameraNum; camInd++) {
					thBufferInds[camInd] = (thBufferInds[camInd] + 1) % bufferSize;
				}
			}
		}
		else if (captureMode == GenCamCaptureMode::Single ||
			captureMode == GenCamCaptureMode::SingleTrigger) {
			SysUtil::errorOutput("Single mode is not implemented yet !");
			exit(-1);
		}
		return 0;
	}
}



