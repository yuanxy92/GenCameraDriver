## CMakeLists.txt for general camera driver
#
#
#

cmake_minimum_required (VERSION 3.1)
set(name "GenCameraDriver")
project ( ${name} )

#c++ version
set (CMAKE_CXX_STANDARD 11)

#opencv 
if ( WIN32 )
    set(OpenCV_DIR "E:/libs/OpenCV/x64/vc14/lib")
    set(OpenCV_CONFIG_PATH "E:/libs/OpenCV")
    set(WIN_HEADER_PATH "${PROJECT_SOURCE_DIR}/winheadfile")
    include_directories(${WIN_HEADER_PATH})
endif()
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#cuda
find_package(CUDA)
set(CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS}
#    -O3 
	-gencode=arch=compute_61,code=\"sm_61,compute_61\"
    )

set(MY_HEADER_FILES
    # C++ header file
    cuda/NPPJpegCoderKernel.h
)

set(MY_SOURCE_FILES 
    # C++ source file
    NPPJpegCoder.cpp
    # cuda source file
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
    ${CUDA_npp_LIBRARY}
    ${CUDA_nppc_LIBRARY}
    debug ${PTGREY_SDK_LIBRARY_DEBUG}
    optimized ${PTGREY_SDK_LIBRARY_RELEASE}
)