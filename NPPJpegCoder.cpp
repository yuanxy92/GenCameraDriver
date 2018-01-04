/*
@brief c++ source class for NPP jpeg coder
@author Shane Yuan
@date Oct 11, 2017
*/

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudaimgproc.hpp>

#include <cmath>
#include <string>
#include <fstream>
#include <iostream>

#include "Exceptions.h"
#include "NPPJpegCoder.h"

#include "helper_string.h"
#include "helper_cuda.h"

using namespace std;

// #define MEASURE_KERNEL_TIME

namespace npp {

	template<class T>
	T readBigEndian(const unsigned char *pData) {
		if (sizeof(T) > 1)
		{
			unsigned char p[sizeof(T)];
			reverse_copy(pData, pData + sizeof(T), p);
			return *reinterpret_cast<T *>(p);
		}
		else
		{
			return *pData;
		}
	}

	template<class T>
	void writeBigEndian(unsigned char *pData, T value)
	{
		unsigned char *pValue = reinterpret_cast<unsigned char *>(&value);
		reverse_copy(pValue, pValue + sizeof(T), pData);
	}

	int DivUp(int x, int d) {
		return (x + d - 1) / d;
	}

	template<typename T>
	T readAndAdvance(const unsigned char *&pData) {
		T nElement = readBigEndian<T>(pData);
		pData += sizeof(T);
		return nElement;
	}

	template<typename T>
	void writeAndAdvance(unsigned char *&pData, T nElement) {
		writeBigEndian<T>(pData, nElement);
		pData += sizeof(T);
	}

	int nextMarker(const unsigned char *pData, int &nPos, int nLength) {
		unsigned char c = pData[nPos++];

		do
		{
			while (c != 0xffu && nPos < nLength)
			{
				c = pData[nPos++];
			}

			if (nPos >= nLength)
				return -1;

			c = pData[nPos++];
		} while (c == 0 || c == 0x0ffu);

		return c;
	}

	void writeMarker(unsigned char nMarker, unsigned char *&pData) {
		*pData++ = 0x0ff;
		*pData++ = nMarker;
	}

	void writeJFIFTag(unsigned char *&pData) {
		const char JFIF_TAG[] =
		{
			0x4a, 0x46, 0x49, 0x46, 0x00,
			0x01, 0x02,
			0x00,
			0x00, 0x01, 0x00, 0x01,
			0x00, 0x00
		};

		writeMarker(0x0e0, pData);
		writeAndAdvance<unsigned short>(pData, sizeof(JFIF_TAG) + sizeof(unsigned short));
		memcpy(pData, JFIF_TAG, sizeof(JFIF_TAG));
		pData += sizeof(JFIF_TAG);
	}

	void loadJpeg(const char *input_file, unsigned char *&pJpegData, int &nInputLength) {
		// Load file into CPU memory
		ifstream stream(input_file, ifstream::binary);

		if (!stream.good())
		{
			return;
		}

		stream.seekg(0, ios::end);
		nInputLength = (int)stream.tellg();
		stream.seekg(0, ios::beg);

		pJpegData = new unsigned char[nInputLength];
		stream.read(reinterpret_cast<char *>(pJpegData), nInputLength);
	}

	void readFrameHeader(const unsigned char *pData, FrameHeader &header) {
		readAndAdvance<unsigned short>(pData);
		header.nSamplePrecision = readAndAdvance<unsigned char>(pData);
		header.nHeight = readAndAdvance<unsigned short>(pData);
		header.nWidth = readAndAdvance<unsigned short>(pData);
		header.nComponents = readAndAdvance<unsigned char>(pData);

		for (int c = 0; c < header.nComponents; ++c)
		{
			header.aComponentIdentifier[c] = readAndAdvance<unsigned char>(pData);
			header.aSamplingFactors[c] = readAndAdvance<unsigned char>(pData);
			header.aQuantizationTableSelector[c] = readAndAdvance<unsigned char>(pData);
		}

	}

	void writeFrameHeader(const FrameHeader &header, unsigned char *&pData) {
		unsigned char aTemp[128];
		unsigned char *pTemp = aTemp;

		writeAndAdvance<unsigned char>(pTemp, header.nSamplePrecision);
		writeAndAdvance<unsigned short>(pTemp, header.nHeight);
		writeAndAdvance<unsigned short>(pTemp, header.nWidth);
		writeAndAdvance<unsigned char>(pTemp, header.nComponents);

		for (int c = 0; c < header.nComponents; ++c)
		{
			writeAndAdvance<unsigned char>(pTemp, header.aComponentIdentifier[c]);
			writeAndAdvance<unsigned char>(pTemp, header.aSamplingFactors[c]);
			writeAndAdvance<unsigned char>(pTemp, header.aQuantizationTableSelector[c]);
		}

		unsigned short nLength = (unsigned short)(pTemp - aTemp);

		writeMarker(0x0C0, pData);
		writeAndAdvance<unsigned short>(pData, nLength + 2);
		memcpy(pData, aTemp, nLength);
		pData += nLength;
	}


	void readScanHeader(const unsigned char *pData, ScanHeader &header) {
		readAndAdvance<unsigned short>(pData);

		header.nComponents = readAndAdvance<unsigned char>(pData);

		for (int c = 0; c < header.nComponents; ++c)
		{
			header.aComponentSelector[c] = readAndAdvance<unsigned char>(pData);
			header.aHuffmanTablesSelector[c] = readAndAdvance<unsigned char>(pData);
		}

		header.nSs = readAndAdvance<unsigned char>(pData);
		header.nSe = readAndAdvance<unsigned char>(pData);
		header.nA = readAndAdvance<unsigned char>(pData);
	}


	void writeScanHeader(const ScanHeader &header, unsigned char *&pData) {
		unsigned char aTemp[128];
		unsigned char *pTemp = aTemp;

		writeAndAdvance<unsigned char>(pTemp, header.nComponents);

		for (int c = 0; c < header.nComponents; ++c)
		{
			writeAndAdvance<unsigned char>(pTemp, header.aComponentSelector[c]);
			writeAndAdvance<unsigned char>(pTemp, header.aHuffmanTablesSelector[c]);
		}

		writeAndAdvance<unsigned char>(pTemp, header.nSs);
		writeAndAdvance<unsigned char>(pTemp, header.nSe);
		writeAndAdvance<unsigned char>(pTemp, header.nA);

		unsigned short nLength = (unsigned short)(pTemp - aTemp);

		writeMarker(0x0DA, pData);
		writeAndAdvance<unsigned short>(pData, nLength + 2);
		memcpy(pData, aTemp, nLength);
		pData += nLength;
	}


	void readQuantizationTables(const unsigned char *pData, QuantizationTable *pTables) {
		unsigned short nLength = readAndAdvance<unsigned short>(pData) - 2;

		while (nLength > 0)
		{
			unsigned char nPrecisionAndIdentifier = readAndAdvance<unsigned char>(pData);
			int nIdentifier = nPrecisionAndIdentifier & 0x0f;

			pTables[nIdentifier].nPrecisionAndIdentifier = nPrecisionAndIdentifier;
			memcpy(pTables[nIdentifier].aTable, pData, 64);
			pData += 64;

			nLength -= 65;
		}
	}

	void writeQuantizationTable(const QuantizationTable &table, unsigned char *&pData) {
		writeMarker(0x0DB, pData);
		writeAndAdvance<unsigned short>(pData, sizeof(QuantizationTable) + 2);
		memcpy(pData, &table, sizeof(QuantizationTable));
		pData += sizeof(QuantizationTable);
	}

	void readHuffmanTables(const unsigned char *pData, HuffmanTable *pTables) {
		unsigned short nLength = readAndAdvance<unsigned short>(pData) - 2;

		while (nLength > 0)
		{
			unsigned char nClassAndIdentifier = readAndAdvance<unsigned char>(pData);
			int nClass = nClassAndIdentifier >> 4; // AC or DC
			int nIdentifier = nClassAndIdentifier & 0x0f;
			int nIdx = nClass * 2 + nIdentifier;
			pTables[nIdx].nClassAndIdentifier = nClassAndIdentifier;

			// Number of Codes for Bit Lengths [1..16]
			int nCodeCount = 0;

			for (int i = 0; i < 16; ++i)
			{
				pTables[nIdx].aCodes[i] = readAndAdvance<unsigned char>(pData);
				nCodeCount += pTables[nIdx].aCodes[i];
			}

			memcpy(pTables[nIdx].aTable, pData, nCodeCount);
			pData += nCodeCount;

			nLength -= 17 + nCodeCount;
		}
	}

	void writeHuffmanTable(const HuffmanTable &table, unsigned char *&pData) {
		writeMarker(0x0C4, pData);

		// Number of Codes for Bit Lengths [1..16]
		int nCodeCount = 0;

		for (int i = 0; i < 16; ++i)
		{
			nCodeCount += table.aCodes[i];
		}

		writeAndAdvance<unsigned short>(pData, 17 + nCodeCount + 2);
		memcpy(pData, &table, 17 + nCodeCount);
		pData += 17 + nCodeCount;
	}


	void readRestartInterval(const unsigned char *pData, int &nRestartInterval) {
		readAndAdvance<unsigned short>(pData);
		nRestartInterval = readAndAdvance<unsigned short>(pData);
	}

	bool printfNPPinfo(int cudaVerMajor, int cudaVerMinor) {
		const NppLibraryVersion *libVer = nppGetLibVersion();

		printf("NPP Library Version %d.%d.%d\n", libVer->major, libVer->minor, libVer->build);

		int driverVersion, runtimeVersion;
		cudaDriverGetVersion(&driverVersion);
		cudaRuntimeGetVersion(&runtimeVersion);

		printf("  CUDA Driver  Version: %d.%d\n", driverVersion / 1000, (driverVersion % 100) / 10);
		printf("  CUDA Runtime Version: %d.%d\n", runtimeVersion / 1000, (runtimeVersion % 100) / 10);

		bool bVal = checkCudaCapabilities(cudaVerMajor, cudaVerMinor);
		return bVal;
	}

	/**
	@brief constructor
	*/
	NPPJpegCoder::NPPJpegCoder(): isWBRaw(false) {}
	NPPJpegCoder::~NPPJpegCoder() {}

	/**
	@brief set bayer type
	@param int cfaBayerType: cfa bayer type
	@return int
	*/
	int NPPJpegCoder::setCfaBayerType(int cfaBayerType) {
		this->cfaBayerType = static_cast<NppiBayerGridPosition>(cfaBayerType);
		return 0;
	}

	/**
	@brief set input raw data type
	before auto white balance adjustment or
	@param int cfaBayerType: cfa bayer type
	@return int
	*/
	int NPPJpegCoder::setWBRawType(bool isWBRaw) {
		this->isWBRaw = isWBRaw;
		return 0;
	}

	/**
	@brief set white balance gain
	@param float redGain: gain of red channel
	@param float greenGain: gain of green channel
	@param float blueGain: gain of blue channel
	@return int
	*/
	int NPPJpegCoder::setWhiteBalanceGain(float redGain, float greenGain, float blueGain) {
		this->redGain = redGain;
		this->greenGain = greenGain;
		this->blueGain = blueGain;
		wbTwist[0][0] = redGain;
		wbTwist[1][1] = greenGain;
		wbTwist[2][2] = blueGain;
		return 0;
	}

	/**
	@brief init jpeg encoder
	@param int width: input image width
	@param int height: input image height
	@param int quality: jpeg encoding quality
	@return
	*/
	int NPPJpegCoder::init(int width, int height, int quality) {
		if (printfNPPinfo(2, 0) == false) {
			cerr << "jpegNPP requires a GPU with Compute Capability 2.0 or higher" << endl;
			exit(-1);
		}
		// calculate quantization table from quality
		float s;
		if (quality < 50) 
			s = 5000.0f / quality;
		else s = 200.0f - 2 * quality;
		for (size_t i = 0; i < 64; i++) {
			// luminance
			float luminVal = (float)quantiztionTableLuminance[i];
			luminVal = floor((s * luminVal + 50.0f) / 100.0f);
			if (luminVal < 1)
				luminVal = 1;
			else if (luminVal > 255)
				luminVal = 255;
			quantiztionTableLuminance[i] = (unsigned char)luminVal;
			// chroma
			float chromaVal = (float)quantiztionTableChroma[i];
			chromaVal = floor((s * chromaVal + 50.0f) / 100.0f);
			if (chromaVal < 1)
				chromaVal = 1;
			else if (chromaVal > 255)
				chromaVal = 255;
			quantiztionTableChroma[i] = (unsigned char)chromaVal;
		}

		// set width and height
		this->width = width;
		this->height = height;
		// calculate size
		aSrcSize[0].width = width;
		aSrcSize[0].height = height;
		aSrcSize[1].width = width;
		aSrcSize[1].height = height;
		aSrcSize[2].width = width;
		aSrcSize[2].height = height;
		aDstSize[0].width = width;
		aDstSize[0].height = height;
		aDstSize[1].width = width / 2;
		aDstSize[1].height = height / 2;
		aDstSize[2].width = width / 2;
		aDstSize[2].height = height / 2;
		// init output frame header
		memset(&oFrameHeader, 0, sizeof(FrameHeader));
		oFrameHeader.nWidth = width;
		oFrameHeader.nHeight = height;
		oFrameHeader.nSamplePrecision = 8;
		oFrameHeader.nComponents = 3;
		oFrameHeader.aComponentIdentifier[0] = 1;
		oFrameHeader.aComponentIdentifier[1] = 2;
		oFrameHeader.aComponentIdentifier[2] = 3;
		oFrameHeader.aSamplingFactors[0] = 34;
		oFrameHeader.aSamplingFactors[1] = 17;
		oFrameHeader.aSamplingFactors[2] = 17;
		oFrameHeader.aQuantizationTableSelector[0] = 0;
		oFrameHeader.aQuantizationTableSelector[1] = 1;
		oFrameHeader.aQuantizationTableSelector[2] = 1;
		// init quantization table
		memset(aQuantizationTables, 0, 4 * sizeof(QuantizationTable));
		memset(aHuffmanTables, 0, 4 * sizeof(HuffmanTable));
		aQuantizationTables[0].nPrecisionAndIdentifier = 0;
		memcpy(aQuantizationTables[0].aTable, quantiztionTableLuminance, 64 * sizeof(unsigned char));
		aQuantizationTables[1].nPrecisionAndIdentifier = 1;
		memcpy(aQuantizationTables[1].aTable, quantiztionTableChroma, 64 * sizeof(unsigned char));
		cudaMalloc(&pdQuantizationTables, 64 * 4);
		// Copy DCT coefficients and Quantization Tables from host to device
		for (int i = 0; i < 2; ++i) {
			NPP_CHECK_CUDA(cudaMemcpy(pdQuantizationTables + i * 64,
				aQuantizationTables[i].aTable, 64, cudaMemcpyHostToDevice));
		}
		// init huffman table
		memcpy(aHuffmanTables[0].aCodes, huffmanCodeLuminanceDC, 16 * sizeof(unsigned char));
		memcpy(aHuffmanTables[0].aTable, huffmanTableLuminanceDC, 256 * sizeof(unsigned char));
		aHuffmanTables[0].nClassAndIdentifier = 0;
		memcpy(aHuffmanTables[1].aCodes, huffmanCodeChromaDC, 16 * sizeof(unsigned char));
		memcpy(aHuffmanTables[1].aTable, huffmanTableChromaDC, 256 * sizeof(unsigned char));
		aHuffmanTables[1].nClassAndIdentifier = 1;
		memcpy(aHuffmanTables[2].aCodes, huffmanCodeLuminanceAC, 16 * sizeof(unsigned char));
		memcpy(aHuffmanTables[2].aTable, huffmanTableLuminanceAC, 256 * sizeof(unsigned char));
		aHuffmanTables[2].nClassAndIdentifier = 16;
		memcpy(aHuffmanTables[3].aCodes, huffmanCodeChromaAC, 16 * sizeof(unsigned char));
		memcpy(aHuffmanTables[3].aTable, huffmanTableChromaAC, 256 * sizeof(unsigned char));
		aHuffmanTables[3].nClassAndIdentifier = 17;
		pHuffmanDCTables = aHuffmanTables;
		pHuffmanACTables = &aHuffmanTables[2];
		// init scanner header
		oScanHeader.nA = 0;
		oScanHeader.nComponents = 3;
		oScanHeader.nSe = 63;
		oScanHeader.nSs = 0;
		oScanHeader.aComponentSelector[0] = 1;
		oScanHeader.aComponentSelector[1] = 2;
		oScanHeader.aComponentSelector[2] = 3;
		oScanHeader.aHuffmanTablesSelector[0] = 0;
		oScanHeader.aHuffmanTablesSelector[1] = 17;
		oScanHeader.aHuffmanTablesSelector[2] = 17;

		// init nppiEncodeHuffmanSpecInitAlloc_JPEG
		for (int i = 0; i < 3; ++i) {
			nppiEncodeHuffmanSpecInitAlloc_JPEG(pHuffmanDCTables[(oScanHeader.aHuffmanTablesSelector[i] >> 4)].aCodes, nppiDCTable, &apHuffmanDCTable[i]);
			nppiEncodeHuffmanSpecInitAlloc_JPEG(pHuffmanACTables[(oScanHeader.aHuffmanTablesSelector[i] & 0x0f)].aCodes, nppiACTable, &apHuffmanACTable[i]);
		}
		
		// Compute channel sizes as stored in the JPEG (8x8 blocks & MCU block layout)
		for (int i = 0; i < oFrameHeader.nComponents; ++i) {
			nMCUBlocksV = max(nMCUBlocksV, oFrameHeader.aSamplingFactors[i] & 0x0f);
			nMCUBlocksH = max(nMCUBlocksH, oFrameHeader.aSamplingFactors[i] >> 4);
		}
		for (int i = 0; i < 3; ++i) {
			nppiDecodeHuffmanSpecInitAllocHost_JPEG(pHuffmanDCTables[(oScanHeader.aHuffmanTablesSelector[i] >> 4)].aCodes,
				nppiDCTable, &apHuffmanDCTableDecode[i]);
			nppiDecodeHuffmanSpecInitAllocHost_JPEG(pHuffmanACTables[(oScanHeader.aHuffmanTablesSelector[i] & 0x0f)].aCodes,
				nppiACTable, &apHuffmanACTableDecode[i]);
		}

		for (int i = 0; i < oFrameHeader.nComponents; ++i) {
			NppiSize oBlocks;
			NppiSize oBlocksPerMCU = { oFrameHeader.aSamplingFactors[i] >> 4, oFrameHeader.aSamplingFactors[i] & 0x0f };
			oBlocks.width = (int)ceil((oFrameHeader.nWidth + 7) / 8 *
				static_cast<float>(oBlocksPerMCU.width) / nMCUBlocksH);
			oBlocks.width = DivUp(oBlocks.width, oBlocksPerMCU.width) * oBlocksPerMCU.width;
			oBlocks.height = (int)ceil((oFrameHeader.nHeight + 7) / 8 *
				static_cast<float>(oBlocksPerMCU.height) / nMCUBlocksV);
			oBlocks.height = DivUp(oBlocks.height, oBlocksPerMCU.height) * oBlocksPerMCU.height;
			aDstSize[i].width = oBlocks.width * 8;
			aDstSize[i].height = oBlocks.height * 8;
			// Allocate Memory
			size_t nPitch;
			NPP_CHECK_CUDA(cudaMallocPitch(&apdDCT[i], &nPitch, oBlocks.width * 64 * sizeof(Npp16s), oBlocks.height));
			aDCTStep[i] = static_cast<Npp32s>(nPitch);

			NPP_CHECK_CUDA(cudaMallocPitch(&apDstImage[i], &nPitch, aDstSize[i].width, aDstSize[i].height));
			aDstImageStep[i] = static_cast<Npp32s>(nPitch);

			NPP_CHECK_CUDA(cudaHostAlloc(&aphDCT[i], aDCTStep[i] * oBlocks.height, cudaHostAllocDefault));

			pitch[i] = nPitch;
		}

		for (int i = 0; i < oFrameHeader.nComponents; ++i) {
			NppiSize oBlocks;
			NppiSize oBlocksPerMCU = { oFrameHeader.aSamplingFactors[i] >> 4, oFrameHeader.aSamplingFactors[i] & 0x0f };

			oBlocks.width = (int)ceil((oFrameHeader.nWidth + 7) / 8 *
				static_cast<float>(oBlocksPerMCU.width) / nMCUBlocksH);
			oBlocks.width = DivUp(oBlocks.width, oBlocksPerMCU.width) * oBlocksPerMCU.width;

			oBlocks.height = (int)ceil((oFrameHeader.nHeight + 7) / 8 *
				static_cast<float>(oBlocksPerMCU.height) / nMCUBlocksV);
			oBlocks.height = DivUp(oBlocks.height, oBlocksPerMCU.height) * oBlocksPerMCU.height;

			aSrcSize[i].width = oBlocks.width * 8;
			aSrcSize[i].height = oBlocks.height * 8;

			// Allocate Memory
			size_t nPitch;
			//NPP_CHECK_CUDA(cudaMallocPitch(&apdDCT[i], &nPitch, oBlocks.width * 64 * sizeof(Npp16s), oBlocks.height));
			//aDCTStep[i] = static_cast<Npp32s>(nPitch);
			NPP_CHECK_CUDA(cudaMallocPitch(&apSrcImage[i], &nPitch, aSrcSize[i].width, aSrcSize[i].height));
			aSrcImageStep[i] = static_cast<Npp32s>(nPitch);
			//NPP_CHECK_CUDA(cudaHostAlloc(&aphDCT[i], aDCTStep[i] * oBlocks.height, cudaHostAllocDefault));
		}
		
		// Huffman Encoding
		NPP_CHECK_CUDA(cudaMalloc(&pdScan, sizeof(unsigned char) * 1024 * 1024 * 10));
		NPP_CHECK_NPP(nppiEncodeHuffmanGetSize(aSrcSize[0], 3, &nTempSize));
		NPP_CHECK_CUDA(cudaMalloc(&pJpegEncoderTemp, nTempSize));

		// malloc rgb image buffer
		rgb_img_d = nppiMalloc_8u_C3(width, height, &step_rgb);

		// set defaut cfa bayer pattern
		this->cfaBayerType = NPPI_BAYER_RGGB;

		return 0;
	}

	/**
	@brief release jpeg encode
	@return int
	*/
	int NPPJpegCoder::release() {
		// release memory
		for (int i = 0; i < 3; ++i) {
			nppiEncodeHuffmanSpecFree_JPEG(apHuffmanDCTable[i]);
			nppiEncodeHuffmanSpecFree_JPEG(apHuffmanACTable[i]);
			nppiDecodeHuffmanSpecFreeHost_JPEG(apHuffmanDCTableDecode[i]);
			nppiDecodeHuffmanSpecFreeHost_JPEG(apHuffmanACTableDecode[i]);
			cudaFree(apdDCT[i]);
			cudaFreeHost(aphDCT[i]);
			cudaFree(apDstImage[i]);
			cudaFree(apSrcImage[i]);
		}
		cudaFree(pJpegEncoderTemp);
		cudaFree(pdQuantizationTables);
		cudaFree(pdScan);
		nppiFree(rgb_img_d);
		return 0;
	}

	/**
	@brief encode raw image data to jpeg
	@param unsigned char* bayer_img_d: input bayer image
	@param char* jpegdata: output jpeg data
	@param size_t* datalength: output data length
	@param size_t maxlength: max length (bytes) could be copied to in jpeg data
	@param cudaStream_t stream: cudastream
	@return
	*/
	int NPPJpegCoder::encode(unsigned char* bayer_img_d, unsigned char* jpegdata,
		size_t* datalength, size_t maxlength, cudaStream_t stream) {
		
		nppSetStream(stream);
		NppiDCTState *pDCTState;

#ifdef MEASURE_KERNEL_TIME
		cudaEvent_t start, stop;
		float elapsedTime;
		cudaEventCreate(&start);
		cudaEventRecord(start, 0);
#endif

		// debayer
		NppiSize osize;
		osize.width = this->width;
		osize.height = this->height;
		NppiRect orect;
		orect.x = 0;
		orect.y = 0;
		orect.width = this->width;
		orect.height = this->height;

		//luminPitch = pitch[0];
		//chromaPitchU = pitch[1];
		//chromaPitchV = pitch[2];
		//NPPJpegCoderKernel::bayerRG2patchYUV(bayerRGImg, apDstImage[0], apDstImage[1],
		//	apDstImage[2], luminPitch, chromaPitchU, chromaPitchV);

		// bayer to rgb
		NPP_CHECK_NPP(nppiCFAToRGB_8u_C1C3R(bayer_img_d, this->width, osize,
			orect, rgb_img_d, step_rgb, cfaBayerType, NPPI_INTER_UNDEFINED));
		
		if (isWBRaw == false) {
			// apply white balance
			NPP_CHECK_NPP(nppiColorTwist32f_8u_C3IR(rgb_img_d, step_rgb, osize, wbTwist));
		}

		// rgb to yuv420
		NPP_CHECK_NPP(nppiRGBToYUV420_8u_C3P3R(rgb_img_d, step_rgb, apDstImage, aDstImageStep,
			osize));

		NPP_CHECK_NPP(nppiDCTInitAlloc(&pDCTState));
		// Forward DCT
		for (int i = 0; i < 3; ++i) {
			NPP_CHECK_NPP(nppiDCTQuantFwd8x8LS_JPEG_8u16s_C1R_NEW(apDstImage[i], aDstImageStep[i],
				apdDCT[i], aDCTStep[i],
				pdQuantizationTables + oFrameHeader.aQuantizationTableSelector[i] * 64,
				aDstSize[i],
				pDCTState));
		}

		NPP_CHECK_NPP(nppiEncodeHuffmanScan_JPEG_8u16s_P3R(apdDCT, aDCTStep,
			0, oScanHeader.nSs, oScanHeader.nSe, oScanHeader.nA >> 4, oScanHeader.nA & 0x0f,
			pdScan, &nScanLength,
			apHuffmanDCTable,
			apHuffmanACTable,
			aDstSize,
			pJpegEncoderTemp));

#ifdef MEASURE_KERNEL_TIME
		cudaEventCreate(&stop);
		cudaEventRecord(stop, 0);
		cudaEventSynchronize(stop);
		cudaEventElapsedTime(&elapsedTime, start, stop);
		printf("JPEG encode step1: (file:%s, line:%d) elapsed time : %f ms\n", __FILE__, __LINE__, elapsedTime);
#endif

#ifdef MEASURE_KERNEL_TIME
		cudaEventCreate(&start);
		cudaEventRecord(start, 0);
#endif
		
		// Write JPEG
		unsigned char *pDstOutput = jpegdata;

		writeMarker(0x0D8, pDstOutput);
		writeJFIFTag(pDstOutput);
		writeQuantizationTable(aQuantizationTables[0], pDstOutput);
		writeQuantizationTable(aQuantizationTables[1], pDstOutput);
		writeFrameHeader(oFrameHeader, pDstOutput);
		writeHuffmanTable(pHuffmanDCTables[0], pDstOutput);
		writeHuffmanTable(pHuffmanACTables[0], pDstOutput);
		writeHuffmanTable(pHuffmanDCTables[1], pDstOutput);
		writeHuffmanTable(pHuffmanACTables[1], pDstOutput);
		writeScanHeader(oScanHeader, pDstOutput);

		NPP_CHECK_CUDA(cudaMemcpyAsync(pDstOutput, pdScan, nScanLength, cudaMemcpyDeviceToHost, stream));

		if (static_cast<size_t>(pDstOutput + nScanLength + 2 - jpegdata) > maxlength) {
			std::cerr << "FATAL ERROR: Pre-malloced jpeg data size is too small ! " << std::endl;
			exit(-1);
		}

		pDstOutput += nScanLength;
		writeMarker(0x0D9, pDstOutput);

#ifdef MEASURE_KERNEL_TIME
		cudaEventCreate(&stop);
		cudaEventRecord(stop, 0);
		cudaEventSynchronize(stop);
		cudaEventElapsedTime(&elapsedTime, start, stop);
		printf("JPEG encode step2: (file:%s, line:%d) elapsed time : %f ms\n", __FILE__, __LINE__, elapsedTime);
#endif

		// calculate compressed jpeg data length
		*datalength = static_cast<size_t>(pDstOutput - jpegdata);
		// release gpu memory
		nppiDCTFree(pDCTState);
		return 0;
	}

	/**
	@brief decode jpeg image to raw image data (full)
	@param unsigned char* jpegdata: input jpeg data
	@param size_t input_datalength: input jpeg data length
	@param cv::cuda::GpuMat: output gpu mat image
	@return int
	*/
	int NPPJpegCoder::decode(unsigned char* jpegdata, size_t input_datalength,
		cv::cuda::GpuMat & outimg) {
		// init state
		NppiDCTState *pDCTState;
		NPP_CHECK_NPP(nppiDCTInitAlloc(&pDCTState));

#ifdef MEASURE_KERNEL_TIME
		cudaEvent_t start, stop;
		float elapsedTime;
		cudaEventCreate(&start);
		cudaEventRecord(start, 0);
#endif

		// check if this is a vlid JPEG file
		int nPos = 0;
		int nMarker = nextMarker(jpegdata, nPos, input_datalength);
		if (nMarker != 0x0D8) {
			std::cerr << "Invalid Jpeg Image" << std::endl;
			return EXIT_FAILURE;
		}
		nMarker = nextMarker(jpegdata, nPos, input_datalength);
		nMCUBlocksH = 0;
		nMCUBlocksV = 0;

		int nRestartInterval = -1;
		while (nMarker != -1) {
			if (nMarker == 0x0D8) {
				// Embedded Thumbnail, skip it
				int nNextMarker = nextMarker(jpegdata, nPos, input_datalength);
				while (nNextMarker != -1 && nNextMarker != 0x0D9) {
					nNextMarker = nextMarker(jpegdata, nPos, input_datalength);
				}
			}
			if (nMarker == 0x0DD) {
				readRestartInterval(jpegdata + nPos, nRestartInterval);
			}
			if ((nMarker == 0x0C0) | (nMarker == 0x0C2)) {
				//Assert Baseline for this Sample
				//Note: NPP does support progressive jpegs for both encode and decode
				if (nMarker != 0x0C0) {
					cerr << "The sample does only support baseline JPEG images" << endl;
					return EXIT_SUCCESS;
				}
				// Baseline or Progressive Frame Header
				readFrameHeader(jpegdata + nPos, oFrameHeader);
#ifdef MEASURE_KERNEL_TIME	
				cout << "Image Size: " << oFrameHeader.nWidth << "x" << oFrameHeader.nHeight << "x" << static_cast<int>(oFrameHeader.nComponents) << endl;
#endif
				//Assert 3-Channel Image for this Sample
				if (oFrameHeader.nComponents != 3) {
					cerr << "The sample does only support color JPEG images" << endl;
					return EXIT_SUCCESS;
				}
				// Compute channel sizes as stored in the JPEG (8x8 blocks & MCU block layout)
				for (int i = 0; i < oFrameHeader.nComponents; ++i) {
					nMCUBlocksV = max(nMCUBlocksV, oFrameHeader.aSamplingFactors[i] & 0x0f);
					nMCUBlocksH = max(nMCUBlocksH, oFrameHeader.aSamplingFactors[i] >> 4);
				}
				//for (int i = 0; i < oFrameHeader.nComponents; ++i) {
				//	NppiSize oBlocks;
				//	NppiSize oBlocksPerMCU = { oFrameHeader.aSamplingFactors[i] >> 4, oFrameHeader.aSamplingFactors[i] & 0x0f };

				//	oBlocks.width = (int)ceil((oFrameHeader.nWidth + 7) / 8 *
				//		static_cast<float>(oBlocksPerMCU.width) / nMCUBlocksH);
				//	oBlocks.width = DivUp(oBlocks.width, oBlocksPerMCU.width) * oBlocksPerMCU.width;

				//	oBlocks.height = (int)ceil((oFrameHeader.nHeight + 7) / 8 *
				//		static_cast<float>(oBlocksPerMCU.height) / nMCUBlocksV);
				//	oBlocks.height = DivUp(oBlocks.height, oBlocksPerMCU.height) * oBlocksPerMCU.height;

				//	aSrcSize[i].width = oBlocks.width * 8;
				//	aSrcSize[i].height = oBlocks.height * 8;

				//	// Allocate Memory
				//	size_t nPitch;
				//	NPP_CHECK_CUDA(cudaMallocPitch(&apdDCT[i], &nPitch, oBlocks.width * 64 * sizeof(Npp16s), oBlocks.height));
				//	aDCTStep[i] = static_cast<Npp32s>(nPitch);
				//	NPP_CHECK_CUDA(cudaMallocPitch(&apSrcImage[i], &nPitch, aSrcSize[i].width, aSrcSize[i].height));
				//	aSrcImageStep[i] = static_cast<Npp32s>(nPitch);
				//	NPP_CHECK_CUDA(cudaHostAlloc(&aphDCT[i], aDCTStep[i] * oBlocks.height, cudaHostAllocDefault));
				//}
			}
			if (nMarker == 0x0DB) {
				// Quantization Tables
				readQuantizationTables(jpegdata + nPos, aQuantizationTables);
			}
			if (nMarker == 0x0C4) {
				// Huffman Tables
				readHuffmanTables(jpegdata + nPos, aHuffmanTables);
			}
			if (nMarker == 0x0DA) {
				// Scan
				readScanHeader(jpegdata + nPos, oScanHeader);
				nPos += 6 + oScanHeader.nComponents * 2;
				int nAfterNextMarkerPos = nPos;
				int nAfterScanMarker = nextMarker(jpegdata, nAfterNextMarkerPos, input_datalength);
				if (nRestartInterval > 0) {
					while (nAfterScanMarker >= 0x0D0 && nAfterScanMarker <= 0x0D7) {
						// This is a restart marker, go on
						nAfterScanMarker = nextMarker(jpegdata, nAfterNextMarkerPos, input_datalength);
					}
				}
				//for (int i = 0; i < 3; ++i) {
				//	nppiDecodeHuffmanSpecInitAllocHost_JPEG(pHuffmanDCTables[(oScanHeader.aHuffmanTablesSelector[i] >> 4)].aCodes,
				//		nppiDCTable, &apHuffmanDCTableDecode[i]);
				//	nppiDecodeHuffmanSpecInitAllocHost_JPEG(pHuffmanACTables[(oScanHeader.aHuffmanTablesSelector[i] & 0x0f)].aCodes,
				//		nppiACTable, &apHuffmanACTableDecode[i]);
				//}
				NPP_CHECK_NPP(nppiDecodeHuffmanScanHost_JPEG_8u16s_P3R(jpegdata + nPos, nAfterNextMarkerPos - nPos - 2,
					nRestartInterval, oScanHeader.nSs, oScanHeader.nSe, oScanHeader.nA >> 4, oScanHeader.nA & 0x0f,
					aphDCT, aDCTStep,
					apHuffmanDCTableDecode,
					apHuffmanACTableDecode,
					aSrcSize));
				//for (int i = 0; i < 3; ++i){
				//	nppiDecodeHuffmanSpecFreeHost_JPEG(apHuffmanDCTableDecode[i]);
				//	nppiDecodeHuffmanSpecFreeHost_JPEG(apHuffmanACTableDecode[i]);
				//}
			}
			nMarker = nextMarker(jpegdata, nPos, input_datalength);
		}
		// Copy DCT coefficients and Quantization Tables from host to device
		for (int i = 0; i < 4; ++i) {
			NPP_CHECK_CUDA(cudaMemcpyAsync(pdQuantizationTables + i * 64, aQuantizationTables[i].aTable, 64, cudaMemcpyHostToDevice));
		}
		for (int i = 0; i < 3; ++i) {
			NPP_CHECK_CUDA(cudaMemcpyAsync(apdDCT[i], aphDCT[i], aDCTStep[i] * aSrcSize[i].height / 8, cudaMemcpyHostToDevice));
		}
		// Inverse DCT
		for (int i = 0; i < 3; ++i) {
			NPP_CHECK_NPP(nppiDCTQuantInv8x8LS_JPEG_16s8u_C1R_NEW(apdDCT[i], aDCTStep[i],
				apSrcImage[i], aSrcImageStep[i],
				pdQuantizationTables + oFrameHeader.aQuantizationTableSelector[i] * 64,
				aSrcSize[i],
				pDCTState));
		}
		// convert from YUV to BGR
		// yuv420 to rgb
		NppiSize osize;
		osize.width = this->width;
		osize.height = this->height;

		NPP_CHECK_NPP(nppiYUV420ToBGR_8u_P3C3R(apSrcImage, aSrcImageStep, outimg.data, outimg.step,
			osize));

#ifdef MEASURE_KERNEL_TIME
		cudaEventCreate(&stop);
		cudaEventRecord(stop, 0);
		cudaEventSynchronize(stop);
		cudaEventElapsedTime(&elapsedTime, start, stop);
		printf("JPEG decode: (file:%s, line:%d) elapsed time : %f ms\n", __FILE__, __LINE__, elapsedTime);
#endif
		// release gpu memory
		nppiDCTFree(pDCTState);

		return 0;
	}

};


