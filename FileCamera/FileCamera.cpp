/**
@brief Generic Camera Driver Class
Implementation of file camera
@author Shane Yuan
@date Mar 29, 2018
*/

#include "FileCamera.h"

namespace cam {
	int GenCameraFile::loadConfigFile()
	{
		return 0;
	}
	int GenCameraFile::bufferImageData()
	{
		return 0;
	}
	// constructor
	GenCameraFile::GenCameraFile() {}
	GenCameraFile::GenCameraFile(std::string dir) {
		this->dir = dir;
	}
	GenCameraFile::~GenCameraFile() {}

	/*************************************************************/
	/*                     private function                      */
	/*************************************************************/
	/**
	@brief load file information from config file
	@return int
	*/
	int GenCameraFile::loadConfigFile() {
		std::string configname = cv::format("%s/camera_setup.yml", dir.c_str());
		cv::FileStorage fs(configname, cv::FileStorage::READ);
		// read basic information
		fs["dir"] >> this->dir;
		fs["CamNum"] >> this->camNum;
		fs["BufferScale"] >> this->bufferScale;
		filenames.resize(this->camNum);
		// read detail information of file cameras
		cv::FileNodeIterator it;
		cv::FileNode node = fs["CamParams"];
		size_t ind = 0;
		for (it = node.begin(); it != node.end(); ++it) {
			// read serialnum
			(*it)["serialnum"] >> filenames[ind];
			ind++;
		}
		fs.release();
		return 0;
	}

	/**
	@brief buffer image data
	@return int
	*/
	int GenCameraFile::bufferImageData() {

		return 0;
	}

	/*************************************************************/
	/*                   basic camera function                   */
	/*************************************************************/
	/**
	@brief init function
	@return int
	*/
	int GenCameraFile::init() {
		// read config file in dir
		this->loadConfigFile();
		return 0;
	}

	/**
	@brief release function
	@return int
	*/
	int GenCameraFile::release() {

		return 0;
	}

	/**
	@brief get camera information
	@param std::vector<GenCamInfo> & camInfos: output camera infos
	@return int
	*/
	int GenCameraFile::getCamInfos(std::vector<GenCamInfo>& camInfos) {
		return 0;
	}

	/*************************************************************/
	/*                  camera setting function                  */
	/*************************************************************/
	/**
	@brief set/get bayer pattern
	@param int camInd: input camera index
	@param GenCamBayerPattern & bayerPattern: output bayer pattern
	@return int
	*/
	int GenCameraFile::getBayerPattern(int camInd, GenCamBayerPattern & bayerPattern) {
		return 0;
	}

	/*************************************************************/
	/*                     capturing function                    */
	/*************************************************************/
	/**
	@brief set capturing mode
	@param GenCamCaptureMode captureMode: capture mode
	@param int size: buffer size
	@return int
	*/
	int GenCameraFile::setCaptureMode(GenCamCaptureMode captureMode, int bufferSize) {
		return 0;
	}
};