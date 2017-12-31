/**
@brief General Camera Driver Class
@author Shane Yuan
@date Dec 29, 2017
*/

#include "GenCameraDriver.h"
#include "XIMEACamera.h"
#include "PointGreyCamera.h"

cam::GenCamera::GenCamera() : isInit(false), isCapture(false) {}
cam::GenCamera::~GenCamera() {}

namespace cam {
	/**
	@breif function to init camera array
	@return
	*/
	std::shared_ptr<GenCamera> createCamera(CameraModel camModel) {
		if (camModel == CameraModel::XIMEA_xiC) {
			std::shared_ptr<GenCameraXIMEA> cameraPtr = std::make_shared<GenCameraXIMEA>();
			return std::static_pointer_cast<GenCamera>(cameraPtr);
		}
	}

	/**
	@brief set capturing mode
	@param GenCamCaptureMode captureMode: capture mode
	@param int size: buffer size
	@return
	*/
	int GenCamera::setCaptureMode(GenCamCaptureMode captureMode,
		int bufferSize) {
		// get camera info
		std::vector<GenCamInfo> camInfos;
		this->getCamInfos(camInfos);
		// init capture buffer
		if (captureMode == cam::GenCamCaptureMode::Continous ||
			captureMode == cam::GenCamCaptureMode::ContinousTrigger) {
			// resize vector
			this->bufferImgs.resize(bufferSize);
			for (size_t i = 0; i < this->cameraNum; i++) {
				this->bufferImgs[i].resize(this->cameraNum);
			}
			// malloc mat memory
			for (size_t i = 0; i < this->cameraNum; i++) {
				int width, height;
				width = camInfos[i].width;
				height = camInfos[i].height;
				for (size_t j = 0; j < bufferSize; j++) {
					this->bufferImgs[j][i].create(height, width, CV_8U);
				}
			}
		}
		// start capturing

		return 0;
	}

}

