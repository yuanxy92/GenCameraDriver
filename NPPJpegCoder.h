/*
@brief c++ header file class for NPP jpeg coder
@author Shane Yuan
@date Oct 11, 2017
*/

#ifndef __NPP_JPEG_CODER_H__
#define __NPP_JPEG_CODER_H__

#include <algorithm>

#include <npp.h>
#include <nppi_compression_functions.h>
#include <cuda_runtime.h>

#include <opencv2/cudaimageproc.hpp>

namespace npp {
	// definition
	struct FrameHeader {
		unsigned char nSamplePrecision;
		unsigned short nHeight;
		unsigned short nWidth;
		unsigned char nComponents;
		unsigned char aComponentIdentifier[3];
		unsigned char aSamplingFactors[3];
		unsigned char aQuantizationTableSelector[3];
	};

	struct ScanHeader {
		unsigned char nComponents;
		unsigned char aComponentSelector[3];
		unsigned char aHuffmanTablesSelector[3];
		unsigned char nSs;
		unsigned char nSe;
		unsigned char nA;
	};

	struct QuantizationTable {
		unsigned char nPrecisionAndIdentifier;
		unsigned char aTable[64];
	};

	struct HuffmanTable {
		unsigned char nClassAndIdentifier;
		unsigned char aCodes[16];
		unsigned char aTable[256];
	};

	template<class T>
	T readBigEndian(const unsigned char *pData);
	template<class T>
	void writeBigEndian(unsigned char *pData, T value);

	int DivUp(int x, int d);
	template<typename T>
	T readAndAdvance(const unsigned char *&pData);
	template<typename T>
	void writeAndAdvance(unsigned char *&pData, T nElement);

	int nextMarker(const unsigned char *pData, int &nPos, int nLength);
	void writeMarker(unsigned char nMarker, unsigned char *&pData);
	void writeJFIFTag(unsigned char *&pData);
	void loadJpeg(const char *input_file, unsigned char *&pJpegData, int &nInputLength);
	void readFrameHeader(const unsigned char *pData, FrameHeader &header);
	void writeFrameHeader(const FrameHeader &header, unsigned char *&pData);
	void readScanHeader(const unsigned char *pData, ScanHeader &header);
	void writeScanHeader(const ScanHeader &header, unsigned char *&pData);
	void readQuantizationTables(const unsigned char *pData, QuantizationTable *pTables);
	void writeQuantizationTable(const QuantizationTable &table, unsigned char *&pData);
	void readHuffmanTables(const unsigned char *pData, HuffmanTable *pTables);
	void writeHuffmanTable(const HuffmanTable &table, unsigned char *&pData);
	void readRestartInterval(const unsigned char *pData, int &nRestartInterval);
	bool printfNPPinfo(int cudaVerMajor, int cudaVerMinor);

	class NPPJpegCoder {
	private:
		// cfa bayer pattern type
		NppiBayerGridPosition cfaBayerType;
		// white balance gain
		float redGain;
		float greenGain;
		float blueGain;
		// is raw data already adjusted
		bool isWBRaw;

		QuantizationTable aQuantizationTables[4];
		Npp8u *pdQuantizationTables;
		HuffmanTable aHuffmanTables[4];
		HuffmanTable *pHuffmanDCTables = aHuffmanTables;
		HuffmanTable *pHuffmanACTables = &aHuffmanTables[2];
		ScanHeader oScanHeader;
		int width;
		int height;
		FrameHeader oFrameHeader;
		NppiSize aSrcSize[3];
		Npp32s aDCTStep[3];
		NppiSize aDstSize[3];
		Npp32s aDstImageStep[3];
		Npp32s aSrcImageStep[3];

		Npp8u *pdScan;
		Npp8u *pJpegEncoderTemp;

		Npp32s nScanLength;
		size_t nTempSize;

		NppiEncodeHuffmanSpec *apHuffmanDCTable[3];
		NppiEncodeHuffmanSpec *apHuffmanACTable[3];

		NppiDecodeHuffmanSpec *apHuffmanDCTableDecode[3];
		NppiDecodeHuffmanSpec *apHuffmanACTableDecode[3];

		Npp16s *aphDCT[3];
		Npp16s *apdDCT[3];
		unsigned int pitch[3];
		Npp8u *apDstImage[3];
		Npp8u *apSrcImage[3];

		Npp8u* rgb_img_d;
		int step_rgb;
		int luminPitch;
		int chromaPitchU;
		int chromaPitchV;

		int nMCUBlocksH = 0;
		int nMCUBlocksV = 0;

	public:
		// white balance color twist
		Npp32f wbTwist[3][4] = {
			{1.0, 0.0, 0.0, 0.0},
			{0.0, 1.0, 0.0, 0.0},
			{0.0, 0.0, 1.0, 0.0}
		};
		// quantization table
		unsigned char quantiztionTableLuminance[64] = {
			16,  11,  10,  16,  24,  40,  51,  61,
			12,  12,  14,  19,  26,  58,  60,  55,
			14,  13,  16,  24,  40,  57,  69,  56,
			14,  17,  22,  29,  51,  87,  80,  62,
			18,  22,  37,  56,  68, 109, 103,  77,
			24,  35,  55,  64,  81, 104, 113,  92,
			49,  64,  78,  87, 103, 121, 120, 101,
			72,  92,  95,  98, 112, 100, 103,  99
		};
		unsigned char quantiztionTableChroma[64] = {
			17,  18,  24,  47,  99,  99,  99,  99,
			18,  21,  26,  66,  99,  99,  99,  99,
			24,  26,  56,  99,  99,  99,  99,  99,
			47,  66,  99,  99,  99,  99,  99,  99,
			99,  99,  99,  99,  99,  99,  99,  99,
			99,  99,  99,  99,  99,  99,  99,  99,
			99,  99,  99,  99,  99,  99,  99,  99,
			99,  99,  99,  99,  99,  99,  99,  99
		};
		// huffman table
		const unsigned char huffmanCodeLuminanceDC[16] = {
			0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0
		};
		const unsigned char huffmanTableLuminanceDC[256] = {
			0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 0, 0, 0, //vals
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0
		};
		const unsigned char huffmanCodeChromaDC[16] = {
			0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0
		};
		const unsigned char huffmanTableChromaDC[256] = {
			0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 0, 0, 0, //vals
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0
		};
		const unsigned char huffmanCodeLuminanceAC[16] = {
			0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d
		};
		const unsigned char huffmanTableLuminanceAC[256] = {
			0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, //vals
			0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
			0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
			0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
			0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
			0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
			0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
			0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
			0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
			0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
			0xf9, 0xfa, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
			0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
			0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
			0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
			0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
			0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0
		};
		const unsigned char huffmanCodeChromaAC[16] = {
			0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77
		};
		const unsigned char huffmanTableChromaAC[256] = {
			0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71, //vals
			0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
			0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
			0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
			0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
			0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
			0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
			0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
			0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
			0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
			0xf9, 0xfa, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
			0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
			0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
			0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
			0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
			0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0
		};
		
	private:

	public:
		NPPJpegCoder();
		~NPPJpegCoder();

		/**
		@brief init jpeg encoder
		@param int width: input image width
		@param int height: input image height
		@param int quality: jpeg encoding quality
		@return 
		*/
		int init(int width, int height, int quality = -1);

		/**
		@brief set bayer type
		@param int cfaBayerType: cfa bayer type
		@return int
		*/
		int setCfaBayerType(int cfaBayerType);

		/**
		@brief set white balance gain
		@param float redGain: gain of red channel
		@param float greenGain: gain of green channel
		@param float blueGain: gain of blue channel
		@return int
		*/
		int setWhiteBalanceGain(float redGain, float greenGain, float blueGain);

		/**
		@brief set input raw data type
		before auto white balance adjustment or
		@param int cfaBayerType: cfa bayer type
		@return int
		*/
		int setWBRawType(bool isWBRaw);

		/**
		@brief release jpeg encode
		@return int
		*/
		int release();

		/**
		@brief encode raw image data to jpeg
		@param unsigned char* bayer_img_d: input bayer image (default step = image width)
		@param unsigned char* jpegdata: output jpeg data
		@param size_t* datalength: output data length
		@param size_t maxlength: max length (bytes) could be copied to in jpeg data
		@param cudaStream_t stream: cudastream
		@return int
		*/
		int encode(unsigned char* bayer_img_d, unsigned char* jpegdata, 
			size_t* datalength, size_t maxlength, cudaStream_t stream);
		
		/**
		@brief encode raw image data to jpeg
		@param cv::cuda::GpuMat bayer_img_d: input bayer image 
		@param unsigned char* jpegdata: output jpeg data
		@param size_t* datalength: output data length
		@param size_t maxlength: max length (bytes) could be copied to in jpeg data
		@param cudaStream_t stream: cudastream
		@return int
		*/
		int encode(cv::cuda::GpuMat bayer_img_d, unsigned char* jpegdata, 
			size_t* datalength, size_t maxlength, cudaStream_t stream);

		/**
		@brief decode jpeg image to raw image data (full)
		@param unsigned char* jpegdata: input jpeg data
		@param size_t input_datalength: input jpeg data length
		@param cv::cuda::GpuMat: output gpu mat image
		@param int type: output pixel format type:
			0: BGR
			1:  RGB (default)
		@return int
		*/
		int decode(unsigned char* jpegdata, size_t input_datalength,
			cv::cuda::GpuMat & outimg, int type = 1);
	};

};

#endif