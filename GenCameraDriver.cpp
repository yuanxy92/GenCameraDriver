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
#include "SKEncoder.h"

namespace cam {

	GenCamera::GenCamera() : isInit(false), isCapture(false),
		isVerbose(false), bufferType(GenCamBufferType::Raw),
		camPurpose(GenCamCapturePurpose::Streaming),
		JPEGQuality(75), sizeRatio(0.12), isStartRecord(true){}
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
				if (camInfos[i].sn.substr(0, 4) == "RAW_")
				{
					cv::Mat img(camInfos[i].height, camInfos[i].width, CV_16UC1);
					for (size_t j = 0; j < this->bufferSize; j++) {
						char outname[1024];
						img.data = reinterpret_cast<uchar*>(this->bufferImgs[j][i].data);
						for(int k = 0;k < 99999;k++)
						{
							sprintf(outname, "%s/%s_%02d_%05d.png", dir.c_str(), _camInfos[i].sn.c_str(), i, j + k);
							if(!SysUtil::existFile(outname))
								break;
							SysUtil::warningOutput("File Exist! FileName = " + std::string(outname));
						}
						cv::imwrite(outname, img);
					}
				}
				else
				{
					// init npp jpeg coder
					std::vector<npp::NPPJpegCoder> coder(4);
					for (size_t k = 0; k < 4; k++) {
						cv::Size size = cam::GenCamera::makeDoubleSize(cv::Size(camInfos[i].width, camInfos[i].height),
							static_cast<cam::GenCamImgRatio>(k));
						coder[k].init(size.width, size.height, JPEGQuality);
					}
					cv::Mat img(camInfos[i].height, camInfos[i].width, CV_8UC3);
					for (size_t j = 0; j < this->bufferSize; j++) {
						char outname[1024];
						int ratioInd = static_cast<int>(this->bufferImgs[j][i].ratio);
						cv::Size size = cam::GenCamera::makeDoubleSize(cv::Size(camInfos[i].width, camInfos[i].height),
							static_cast<cam::GenCamImgRatio>(ratioInd));
						cv::cuda::GpuMat img_d(size, CV_8UC3);
						coder[ratioInd].decode(reinterpret_cast<uchar*>(this->bufferImgs[j][i].data),
							this->bufferImgs[j][i].length,
							img_d, 0);
						img_d.download(img);
						for(int k = 0;k < 99999;k++)
						{
							sprintf(outname, "%s/%s_%02d_%05d.jpg", dir.c_str(), _camInfos[i].sn.c_str(), i, j + k);
							if(!SysUtil::existFile(outname))
								break;
							SysUtil::warningOutput("File Exist! FileName = " + std::string(outname));
						}
						cv::imwrite(outname, img);
					}
					// release npp jpeg coder
					for (size_t k = 0; k < 4; k++) {
						coder[k].release();
					}
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
			for (size_t i = 0; i < this->cameraNum; i++) 
			{
				if (camInfos[i].sn.substr(0, 4) == "RAW_")
				{
					std::string sub_dir = dir + cv::format("./%s/", camInfos[i].sn.c_str());
					SysUtil::mkdir(sub_dir);
					cv::Mat img(camInfos[i].height, camInfos[i].width, CV_16UC1);
					for (size_t j = 0; j < this->bufferSize; j++) {
						char outname[256];
						img.data = reinterpret_cast<uchar*>(this->bufferImgs[j][i].data);
						sprintf(outname, "%s/%s_%02d_%05d.png", sub_dir.c_str(), camInfos[i].sn.c_str(), i, j);
						cv::imwrite(outname, img);
					}
					continue;
				}
				// init npp jpeg coder
				std::vector<npp::NPPJpegCoder> coder(4);
				for (size_t k = 0; k < 4; k++) {
					cv::Size size = cam::GenCamera::makeDoubleSize(cv::Size(camInfos[i].width, camInfos[i].height),
						static_cast<cam::GenCamImgRatio>(k));
					coder[k].init(size.width, size.height, JPEGQuality);
				}
				// init video parameter
				std::string videoname = cv::format("%s/cam_%02d.avi", dir.c_str(), i);
				cv::VideoWriter writer(videoname, cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 
					camInfos[i].fps, cv::Size(camInfos[i].width, camInfos[i].height), true);
				cv::cuda::GpuMat img_d(camInfos[i].height, camInfos[i].width, CV_8UC3);
				cv::Mat img(camInfos[i].height, camInfos[i].width, CV_8UC3);
				for (size_t j = 0; j < this->bufferSize; j++) {
					int ratioInd = static_cast<int>(this->bufferImgs[j][i].ratio);
					cv::Size size = cam::GenCamera::makeDoubleSize(cv::Size(camInfos[i].width, camInfos[i].height),
						static_cast<cam::GenCamImgRatio>(ratioInd));
					cv::cuda::GpuMat img_d_temp(size, CV_8UC3);
					coder[ratioInd].decode(reinterpret_cast<uchar*>(this->bufferImgs[j][i].data), 
						this->bufferImgs[j][i].length,
						img_d_temp, 0);
					cv::cuda::resize(img_d_temp, img_d, img_d.size(), cv::INTER_LINEAR);
					img_d.download(img);
					writer << img;
					//char outname[256];
					//sprintf(outname, "%s/%02d_%05d.jpg", dir.c_str(), i, j);
					//cv::imwrite(outname, img);
				}
				writer.release();
				// release npp jpeg coder
				for (size_t k = 0; k < 4; k++) {
					coder[k].release();
				}
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
	int GenCamera::saveVideosGpu(std::string dir)
	{
		if (this->bufferType != GenCamBufferType::JPEG)
		{
			cam::SysUtil::errorOutput("GenCamera::saveVideosGpu now only support Jpeg Type");
			return -1;
		}
		SysUtil::mkdir(dir);
		//SysUtil::errorOutput(dir);
		//getchar();
		for (size_t i = 0; i < this->cameraNum; i++) 
		{
			if (camInfos[i].sn.substr(0, 4) == "RAW_")
			{
				std::string sub_dir = dir + cv::format("/%s/", camInfos[i].sn.c_str());
				SysUtil::mkdir(sub_dir);
				cv::Mat img(camInfos[i].height, camInfos[i].width, CV_16UC1);
				for (size_t j = 0; j < this->bufferSize; j++) {
					char outname[256];
					img.data = reinterpret_cast<uchar*>(this->bufferImgs[j][i].data);
					sprintf(outname, "%s/%s_%02d_%05d.png", sub_dir.c_str(), camInfos[i].sn.c_str(), i, j);
					cv::imwrite(outname, img);
				}
				continue;
			}

			// init npp jpeg coder
			std::vector<npp::NPPJpegCoder> coder(4);
			for (size_t k = 0; k < 4; k++) 
			{
				cv::Size size = cam::GenCamera::makeDoubleSize(cv::Size(camInfos[i].width, camInfos[i].height),
					static_cast<cam::GenCamImgRatio>(k));
				coder[k].init(size.width, size.height, JPEGQuality);
			}
			// init video parameter
			std::string videoname = cv::format("%s/%s.h265", dir.c_str(), camInfos[i].sn.c_str());

			SKEncoder skencoder;
			skencoder.init(this->bufferSize, cv::Size(camInfos[i].width, camInfos[i].height), videoname);
			cv::cuda::GpuMat _gpu_Resized_YUV[3];
			for (int j = 0; j < 1; j++)
				_gpu_Resized_YUV[j].create(camInfos[i].height, camInfos[i].width, CV_8U);
			for (int j = 1; j < 3; j++)
				_gpu_Resized_YUV[j].create(camInfos[i].height / 2, camInfos[i].width / 2, CV_8U);
			
			for (size_t j = 0; j < this->bufferSize; j++)
			{

				std::vector<void*> _gpu_decoded_YUVdata(3), _gpu_toEncode_YUVdata(3);
				std::vector<uint32_t> steps(3), steps_toencode(3);
				int ratioInd = static_cast<int>(this->bufferImgs[j][i].ratio);
				cv::Size size = cam::GenCamera::makeDoubleSize(cv::Size(camInfos[i].width, camInfos[i].height),
					static_cast<cam::GenCamImgRatio>(ratioInd));
				coder[ratioInd].decode(
					reinterpret_cast<uchar*>(this->bufferImgs[j][i].data),
					this->bufferImgs[j][i].length,
					_gpu_decoded_YUVdata,
					steps);
				cv::cuda::GpuMat Y(size.height, size.width, CV_8U, reinterpret_cast<uchar*>(_gpu_decoded_YUVdata[0]), steps[0]);
				cv::cuda::GpuMat U(size.height / 2, size.width / 2, CV_8U, reinterpret_cast<uchar*>(_gpu_decoded_YUVdata[1]), steps[1]);
				cv::cuda::GpuMat V(size.height / 2, size.width / 2, CV_8U, reinterpret_cast<uchar*>(_gpu_decoded_YUVdata[2]), steps[2]);
				cv::cuda::resize(Y, _gpu_Resized_YUV[0], _gpu_Resized_YUV[0].size(), cv::INTER_LINEAR);
				cv::cuda::resize(U, _gpu_Resized_YUV[1], _gpu_Resized_YUV[1].size(), cv::INTER_LINEAR);
				cv::cuda::resize(V, _gpu_Resized_YUV[2], _gpu_Resized_YUV[2].size(), cv::INTER_LINEAR);
				for (int k = 0; k < 3; k++)
				{
					_gpu_toEncode_YUVdata[k] = _gpu_Resized_YUV[k].data;
					steps_toencode[k] = _gpu_Resized_YUV[k].step;
				}
				skencoder.encode(_gpu_toEncode_YUVdata, steps_toencode);
			}
			skencoder.endEncode();
			// release npp jpeg coder
			for (size_t k = 0; k < 4; k++) 
			{
				coder[k].release();
			}
		}
		return 0;
	}
	/*************************************************************/
	/*   function to set jepg scale ratio for capture function   */
	/*************************************************************/
	/**
	@brief set scale ratio vector of capture function
	@param std::vector<GenCamImgRatio> imgRatios: input scale ratio vector
	@return int
	*/
	int GenCamera::setImageRatios(std::vector<GenCamImgRatio> imgRatios)
	{
		this->imgRatios = imgRatios;
		return 0;
	}

	/**
	@brief make image size even
	@param cv::Size size: input size
	@param cam::GenCamImgRatio ratio: input resize ratio
	@return cv::Size: even size (NPP only accept even size)
	*/
	cv::Size GenCamera::makeDoubleSize(cv::Size size, cam::GenCamImgRatio ratio) 
	{
		cv::Size out;
		if(ratio == GenCamImgRatio::Full)
			out = size;
		else
		{
			float r = 1.0f / powf(2.0f, static_cast<int>(ratio));
			out.width = static_cast<int>(size.width * r);
			out.height = static_cast<int>(size.height * r);
			out.width += (out.width % 2);
			out.height += (out.height % 2);
		}
		return out;
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

	int GenCamera::setBrightnessAdjustment(std::vector<cv::cuda::GpuMat> adjustment)
	{
		brightness_cuda = adjustment;
		return 0;
	}

	/**
	@brief get camera model string
	@return std::string
	*/
	std::string GenCamera::getCamModelString()
	{
		// XIMEA_xiC = 0,
		// PointGrey_u3 = 1,
		// Network = 2,
		// File = 3,
		// Stereo = 4
		if (this->camModel == CameraModel::XIMEA_xiC)
			return "   XIMEA_xiC";
		else if (this->camModel == CameraModel::PointGrey_u3)
			return "PointGrey_u3";
		else if (this->camModel == CameraModel::Network)
			return "     Network";
		else if (this->camModel == CameraModel::File)
			return "        File";
		else if (this->camModel == CameraModel::Stereo)
			return "      Stereo";
		else
			return "   Undefined";
	}
}



