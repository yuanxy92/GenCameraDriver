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

}

