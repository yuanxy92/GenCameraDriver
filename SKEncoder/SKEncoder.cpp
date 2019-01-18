#include "SKEncoder.h"

#include "Utils/Logger.h"

simplelogger::Logger *logger = simplelogger::LoggerFactory::CreateConsoleLogger();

int SKEncoder::init(int frameNum, cv::Size imgSize, std::string fileName, FrameType type, int GpuID)
{
	_frameNum = frameNum;
	_imgSize = imgSize;
	_type = type;
	_GpuID = GpuID;
	_fileName = fileName;
	if (_type != FrameType::IYUV)
	{
		SysUtil::errorOutput("Now Only IYUV is supported");
		return -1;
	}
	if (_imgSize.width <= 0 || _imgSize.width > IMG_MAX_SIDE_LEN || _imgSize.height <= 0 || _imgSize.height > IMG_MAX_SIDE_LEN)
	{
		SysUtil::errorOutput(SysUtil::format("imgSize too huge, please resize or ROI. Max side length support: %d", IMG_MAX_SIDE_LEN));
		return -1;
	}

	_encodeCLIOptions = NvEncoderInitParam("-codec hevc");


	if(_type == FrameType::IYUV)
		_eFormat = NV_ENC_BUFFER_FORMAT_IYUV;
	else if(_type == FrameType::ABGR)
		_eFormat = NV_ENC_BUFFER_FORMAT_ABGR;
	
	ck(cuInit(0));
	CUdevice cuDevice = 0;
	int nGpu = 0;
	ck(cuDeviceGetCount(&nGpu));
	if (_GpuID >= nGpu || _GpuID < 0)
	{
		SysUtil::warningOutput("Gpu id is not vaild, auto changed to the last Gpu");
		_GpuID = nGpu - 1;
	}
	ck(cuDeviceGet(&cuDevice, _GpuID));
	char szDeviceName[80];
	ck(cuDeviceGetName(szDeviceName, sizeof(szDeviceName), cuDevice));
	SysUtil::infoOutput(std::string("GPU in use: ") + szDeviceName);
	ck(cuCtxCreate(&_cuContext, 0, cuDevice));

	_fpOut = std::ofstream(_fileName.c_str(), std::ios::out | std::ios::binary);
	_enc = std::make_shared<NvEncoderCuda>(_cuContext, _imgSize.width, _imgSize.height, _eFormat);
	//_enc = NvEncoderCuda(_cuContext, _imgSize.width, _imgSize.height, _eFormat);
	_initializeParams.encodeConfig = &_encodeConfig;
	_enc->CreateDefaultEncoderParams(&_initializeParams, _encodeCLIOptions.GetEncodeGUID(), _encodeCLIOptions.GetPresetGUID());
	_encodeCLIOptions.SetInitParams(&_initializeParams, _eFormat);
	_enc->CreateEncoder(&_initializeParams);
	_nFrameSize = _enc->GetFrameSize();
	_nFrame = 0;
	_encodedFrameNum = 0;
	_stat_step = (_frameNum / 18 > 5) ? (_frameNum / 18) : 5;
	_stat_last_time = SysUtil::getCurrentTimeMicroSecond();

	return 0;
}

int SKEncoder::encode(std::vector<void*> gpu_YUVdata3, std::vector<uint32_t> step)
{
	if (_encodedFrameNum % _stat_step == 0)
	{
		SysUtil::infoOutput(SysUtil::format("%s : Frame:%d, Total:%d, FrameRate:%f fps\n",
			_fileName.c_str(), _encodedFrameNum, _frameNum,
			_stat_step * 1000000.0f / (SysUtil::getCurrentTimeMicroSecond() - _stat_last_time)));
		_stat_last_time = SysUtil::getCurrentTimeMicroSecond();
	}
	std::vector<std::vector<uint8_t>> vPacket;
	if (_encodedFrameNum < _frameNum - 1)
	{
		const NvEncInputFrame* encoderInputFrame = _enc->GetNextInputFrame();
		NvEncoderCuda::CopyToDeviceFrame_YUV420(_cuContext, gpu_YUVdata3, step, (CUdeviceptr)encoderInputFrame->inputPtr,
			(int)encoderInputFrame->pitch,
			_enc->GetEncodeWidth(),
			_enc->GetEncodeHeight(),
			CU_MEMORYTYPE_DEVICE,
			encoderInputFrame->bufferFormat,
			encoderInputFrame->chromaOffsets,
			encoderInputFrame->numChromaPlanes);
		_enc->EncodeFrame(vPacket);
	}
	else
	{
		_enc->EndEncode(vPacket);
	}
	_nFrame += (int)vPacket.size();
	for (std::vector<uint8_t> &packet : vPacket)
	{
		// For each encoded packet
		_fpOut.write(reinterpret_cast<char*>(packet.data()), packet.size());
	}
	_encodedFrameNum++;
	return 0;
}

int SKEncoder::endEncode()
{
	if (_encodedFrameNum < _frameNum)
	{
		SysUtil::warningOutput("_encodedFrameNum not reached _frameNum");
	}
	_enc->DestroyEncoder();
	_fpOut.close();
	SysUtil::infoOutput(_fileName + " Encode done!\n");
	return 0;
}
