## CMakeLists.txt for generic camera driver
# author: Shane Yuan
# date: Dec 29, 2017
#

cmake_minimum_required (VERSION 3.1)
set(name "GenCameraDriver")
project ( ${name} )

# c++ version
set (CMAKE_CXX_STANDARD 11)

# opencv 
if ( WIN32 )
    set(OpenCV_DIR "E:/libs/OpenCV/x64/vc14/lib")
    set(OpenCV_CONFIG_PATH "E:/libs/OpenCV")
    set(WIN_HEADER_PATH "${PROJECT_SOURCE_DIR}/winheadfile")
    include_directories(${WIN_HEADER_PATH})
endif()
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# cuda
find_package(CUDA)
set(CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS}
#    -O3 
	-gencode=arch=compute_61,code=\"sm_61,compute_61\"
    )

# pointgrey camera sdk
set(PTGREY_SDK_INCLUDE_DIR "C:/Program Files/Point Grey Research/Spinnaker/include")
set(PTGREY_SDK_LIBRARY_DEBUG "C:/Program Files/Point Grey Research/Spinnaker/lib64/vs2015/Spinnakerd_v140.lib")
set(PTGREY_SDK_LIBRARY_RELEASE "C:/Program Files/Point Grey Research/Spinnaker/lib64/vs2015/Spinnaker_v140.lib")
include_directories(${PTGREY_SDK_INCLUDE_DIR})

# XIMEA camera
set(XIMEA_SDK_INCLUDE_DIR "C:/XIMEA/API")
set(XIMEA_SDK_LIBRARY_DEBUG "C:/XIMEA/API/x64/xiapi64.lib")
set(XIMEA_SDK_LIBRARY_RELEASE "C:/XIMEA/API/x64/xiapi64.lib")
include_directories(${XIMEA_SDK_INCLUDE_DIR})

set(MY_HEADER_FILES
    Exceptions.h
    helper_cuda.h
    helper_string.h
    NPPJpegCoder.h
    GenCameraDriver.h
    RealCameraDriver.h
    # point grey camera
    PointGrey/PointGreyCamera.h
    # XIMEA camera
    XIMEA/XIMEACamera.h
)

set(MY_SOURCE_FILES 
    # point grey camera
    PointGrey/PointGreyCamera.cpp
    # XIMEA camera
    XIMEA/XIMEACamera.cpp
    # C++ source file
    NPPJpegCoder.cpp
    # generic camera driver
    GenCameraDriver.cpp
	GenCameraDriverImpl.cpp
    RealCameraDriver.cpp
    # main function
    main.cpp
)

source_group("PointGrey" FILES
    PointGrey/PointGreyCamera.h
    PointGrey/PointGreyCamera.cpp 
)

source_group("XIMEA" FILES
    XIMEA/XIMEACamera.h
    XIMEA/XIMEACamera.cpp
)

source_group("cuda" FILES
    cuda/NPPJpegCoderKernel.h
    cuda/NPPJpegCoderKernel.cu
)

include_directories(
	${PROJECT_SOURCE_DIR}	
	${PROJECT_SOURCE_DIR}/cuda	
	${PROJECT_SOURCE_DIR}/XIMEA	
	${PROJECT_SOURCE_DIR}/PointGrey	
)

cuda_add_executable(${name}
    ${MY_HEADER_FILES}
    ${MY_SOURCE_FILES}
)

target_link_libraries(${name}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARY} 	
	${CUDA_npp_LIBRARY} 
	${CUDA_nppc_LIBRARY} 
	${CUDA_nppicc_LIBRARY}
	${CUDA_nppidei_LIBRARY}
	${CUDA_nppif_LIBRARY}
	${CUDA_nppig_LIBRARY}
	${CUDA_nppim_LIBRARY}
	${CUDA_nppist_LIBRARY}
	${CUDA_nppisu_LIBRARY}
	${CUDA_nppitc_LIBRARY}
	${CUDA_npps_LIBRARY} 
    debug ${PTGREY_SDK_LIBRARY_DEBUG} ${XIMEA_SDK_LIBRARY_DEBUG}
    optimized ${PTGREY_SDK_LIBRARY_RELEASE} ${XIMEA_SDK_LIBRARY_RELEASE}
)