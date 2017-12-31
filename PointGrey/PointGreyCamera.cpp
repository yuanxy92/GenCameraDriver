/**
@brief General Camera Driver Class
Implementation of XIMEA camera
@author Shane Yuan
@date Dec 31, 2017
*/

#include "PointGreyCamera.h"

namespace cam {
	// function to check XIMEA function error
	void cam::checkPTGREYInternal(int result, char const *const func,
		const char *const file, int const line) {
		if (result != 0) {
			char info[256];
			sprintf(info, "PointGrey camera error at %s:%d function: %s\n",
				file, line, func);
			SysUtil::errorOutput(info);
			exit(-1);
		}
	}
	
	// constructor
	GenCameraPTGREY::GenCameraPTGREY() {}
	GenCameraPTGREY::~GenCameraPTGREY() {}


}