/**
@brief SKEncoder, Interface of the NvidiaEncoder
@author zhu-ty
@date Dec 29, 2017
*/

#ifndef __H264_ENCODER_SKENCODER_H__
#define __H264_ENCODER_SKENCODER_H__

// include std
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <queue>
#include <thread>
#include <memory>

// opencv
#include <opencv2/opencv.hpp>

// cuda
#ifdef _WIN32
#include <windows.h>
#endif
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>


/* definition to expand macro then apply to pragma message */
#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var) #var "="  VALUE(var)
#pragma message(VAR_NAME_VALUE(CUDART_VERSION))
#if CUDART_VERSION < 10000
#include <dynlink_nvcuvid.h>
#include <dynlink_cuviddec.h>
#endif
#include <device_launch_parameters.h>

#include "NvEncoder/NvEncoderCuda.h"
#include "Utils/Logger.h"
#include "Utils/NvEncoderCLIOptions.h"
#include "Utils/NvCodecUtils.h"

#include "SKCommon.hpp"

#define IMG_MAX_SIDE_LEN 8192

class SKEncoder
{
public:
	enum class FrameType
	{
		ABGR = 0,
		IYUV = 1
	};
public:
	SKEncoder() {};
	~SKEncoder() {};
	int init(int frameNum, cv::Size imgSize,std::string fileName = "output.h265", FrameType type = FrameType::IYUV, int GpuID = 0);

	int encode(std::vector<void*> gpu_YUVdata3, std::vector<uint32_t> step);

	int endEncode();

private:
	int _frameNum, _GpuID;
	int _encodedFrameNum = 0;
	std::string _fileName;
	cv::Size _imgSize;
	FrameType _type;
	std::ofstream _fpOut;
	std::shared_ptr<NvEncoderCuda> _enc;
	//NvEncoderCuda _enc = NvEncoderCuda(NULL, 0, 0, NV_ENC_BUFFER_FORMAT_UNDEFINED);;
	NV_ENC_INITIALIZE_PARAMS _initializeParams = { NV_ENC_INITIALIZE_PARAMS_VER };
	NV_ENC_CONFIG _encodeConfig = { NV_ENC_CONFIG_VER };
	CUcontext _cuContext = NULL;
	NV_ENC_BUFFER_FORMAT _eFormat;
	NvEncoderInitParam _encodeCLIOptions;
	int _nFrameSize, _nFrame;
	int _stat_step;

	uint64_t _stat_last_time;
};



#endif //__H264_ENCODER_SKENCODER_H__