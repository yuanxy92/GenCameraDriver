## CMakeLists.txt for generic camera driver
# author: Shane Yuan
# date: Dec 29, 2017
#

##
##                     _oo0oo_
##                    o8888888o
##                    88" . "88
##                    (| -_- |)
##                    0\  =  /0
##                  ___/`---'\___
##                .' \\|     |// '.
##               / \\|||  :  |||// \
##              / _||||| -:- |||||- \
##             |   | \\\  -  /// |   |
##             | \_|  ''\---/''  |_/ |
##             \  .-\__  '-'  ___/-. /
##           ___'. .'  /--.--\  `. .'___
##        ."" '<  `.___\_<|>_/___.' >' "".
##       | | :  `- \`.;`\ _ /`;.`/ - ` : | |
##       \  \ `_.   \_ __\ /__ _/   .-` /  /
##   =====`-.____`.___ \_____/___.-`___.-'=====
##                     `=---='
##
##             佛祖保佑         永无BUG
##               Buddha Bless No Bug
##

message(
"##
##                     _oo0oo_
##                    o8888888o
##                    88\" . \"88
##                    (| -_- |)
##                    0\\  =  /0
##                  ___/`---'\\___
##                .' \\\\|     |// '.
##               / \\\\|||  :  |||// \\
##              / _||||| -:- |||||- \\
##             |   | \\\\\\  -  /// |   |
##             | \\_|  ''\\---/''  |_/ |
##             \\  .-\\__  '-'  ___/-. /
##           ___'. .'  /--.--\\  `. .'___
##        .\"\" '<  `.___\\_<|>_/___.' >' \"\".
##       | | :  `- \\`.;`\\ _ /`;.`/ - ` : | |
##       \\  \\ `_.   \\_ __\\ /__ _/   .-` /  /
##   =====`-.____`.___ \\_____/___.-`___.-'=====
##                     `=---='
##
##             佛祖保佑         永无BUG
##               Buddha Bless No Bug
##
"
)

cmake_minimum_required (VERSION 3.10)
set(name "GenCameraDriver")
project ( ${name} )

# c++ version
set (CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

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
if ( WIN32 )
    set(PTGREY_SDK_INCLUDE_DIR "C:/Program Files/Point Grey Research/Spinnaker/include")
	set(PTGREY_SDK_LIBRARY_DEBUG debug "C:/Program Files/Point Grey Research/Spinnaker/lib64/vs2015/Spinnakerd_v140.lib")
	set(PTGREY_SDK_LIBRARY_RELEASE  optimized "C:/Program Files/Point Grey Research/Spinnaker/lib64/vs2015/Spinnaker_v140.lib")
else()
    set(PTGREY_SDK_INCLUDE_DIR "~/zhu-ty/spinnaker/include")
    set(PTGREY_SDK_INCLUDE_DIR ${PTGREY_SDK_INCLUDE_DIR} "/usr/include/spinnaker/")
    if(EXISTS "/home/zhu-ty/zhu-ty/spinnaker/lib/libSpinnaker.so")
        set(PTGREY_SDK_LIBRARY_DEBUG debug "~/zhu-ty/spinnaker/lib/libSpinnaker.so")
        set(PTGREY_SDK_LIBRARY_RELEASE optimized "~/zhu-ty/spinnaker/lib/libSpinnaker.so")
    else()
    set(PTGREY_SDK_LIBRARY_DEBUG debug "libSpinnaker.so")
    set(PTGREY_SDK_LIBRARY_RELEASE optimized "libSpinnaker.so")
    endif()
endif()
include_directories(${PTGREY_SDK_INCLUDE_DIR})

# XIMEA camera
if ( WIN32 )
    set(XIMEA_SDK_INCLUDE_DIR "C:/XIMEA/API")
    set(XIMEA_SDK_LIBRARY_DEBUG debug "C:/XIMEA/API/x64/xiapi64.lib")
    set(XIMEA_SDK_LIBRARY_RELEASE optimized "C:/XIMEA/API/x64/xiapi64.lib")
else ()
    set(XIMEA_SDK_INCLUDE_DIR "/opt/XIMEA/include")
    set(XIMEA_SDK_LIBRARY_DEBUG debug "/usr/lib/libm3api.so")
    set(XIMEA_SDK_LIBRARY_RELEASE optimized "/usr/lib/libm3api.so")
endif()
include_directories(${XIMEA_SDK_INCLUDE_DIR})

#Stereo RealtimeDepth
if ( NOT WIN32 )
    FIND_PACKAGE(Eigen3 REQUIRED) 
    include_directories(${EIGEN3_INCLUDE_DIR})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wno-unknown-pragmas -std=c++11 -msse4")  #-Wall
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wno-unknown-pragmas -msse4")  #-Wall
    include_directories(${PROJECT_SOURCE_DIR}/Stereo/RealTimeDepth)
    set(MY_HEADER_FILES
        ${MY_HEADER_FILES}
        Stereo/RealTimeDepth/FDF1.0.1/image.h
        Stereo/RealTimeDepth/FDF1.0.1/opticalflow_aux.h
        Stereo/RealTimeDepth/FDF1.0.1/solver.h
        Stereo/RealTimeDepth/depthmap.h
        Stereo/RealTimeDepth/GhostElemer.h
        Stereo/RealTimeDepth/patchgrid.h
        Stereo/RealTimeDepth/refine_variational.h
        Stereo/RealTimeDepth/DepthMapUpdater.h
        Stereo/RealTimeDepth/oflow.h
        Stereo/RealTimeDepth/patch.h
    )
    set(MY_SOURCE_FILES 
        ${MY_SOURCE_FILES}
        Stereo/RealTimeDepth/FDF1.0.1/image.c
        Stereo/RealTimeDepth/FDF1.0.1/opticalflow_aux.c
        Stereo/RealTimeDepth/FDF1.0.1/solver.c
        Stereo/RealTimeDepth/depthmap.cpp
        Stereo/RealTimeDepth/GhostElemer.cpp
        Stereo/RealTimeDepth/patch.cpp
        Stereo/RealTimeDepth/refine_variational.cpp
        Stereo/RealTimeDepth/DepthMapUpdater.cpp
        Stereo/RealTimeDepth/oflow.cpp
        Stereo/RealTimeDepth/patchgrid.cpp 
    )
    
endif()

# NvEncoder
set(MY_HEADER_FILES
    ${MY_HEADER_FILES}
	SKEncoder/NvEncoder/NvEncoder/nvEncodeAPI.h
    SKEncoder/NvEncoder/NvEncoder/NvEncoder.h
    SKEncoder/NvEncoder/NvEncoder/NvEncoderCuda.h
    SKEncoder/NvEncoder/Utils/Logger.h
    SKEncoder/NvEncoder/Utils/NvEncoderCLIOptions.h
    SKEncoder/NvEncoder/Utils/NvCodecUtils.h
    SKEncoder/SKEncoder.h
    SKEncoder/SKCommon.hpp
)
set(MY_SOURCE_FILES 
    ${MY_SOURCE_FILES}
	SKEncoder/NvEncoder/NvEncoder/NvEncoder.cpp
    SKEncoder/NvEncoder/NvEncoder/NvEncoderCuda.cpp
    SKEncoder/SKEncoder.cpp
	)
include_directories(
    ${PROJECT_SOURCE_DIR}/SKEncoder
    ${PROJECT_SOURCE_DIR}/SKEncoder/NvEncoder
)
if( WIN32 )
    set(NVCUVID_LIB ${PROJECT_SOURCE_DIR}/SKEncoder/lib/x64/nvcuvid.lib)
else()
    set(NVCUVID_LIB 
    ${PROJECT_SOURCE_DIR}/SKEncoder/lib/linux/stubs/x86_64/libnvcuvid.so
    ${PROJECT_SOURCE_DIR}/SKEncoder/lib/linux/stubs/x86_64/libnvidia-encode.so)
endif()


set(MY_HEADER_FILES
    ${MY_HEADER_FILES}
    Exceptions.h
    helper_cuda.h
    helper_string.h
    NPPJpegCoder.h
    GenCameraDriver.h
    RealCameraDriver.h
    LinuxSocket.hpp
    # point grey camera
    PointGrey/PointGreyCamera.h
    # XIMEA camera
    XIMEA/XIMEACamera.h
    # File camera
    FileCamera/FileCamera.h
    # Stereo camera
    Stereo/StereoCamera.h
    Stereo/INIReader.h
    Stereo/StereoRectify.h
    Stereo/ExposureFusion.h
    Stereo/DisparityProcessor.h
)

set(MY_SOURCE_FILES 
    ${MY_SOURCE_FILES}
    # point grey camera
    PointGrey/PointGreyCamera.cpp
    # XIMEA camera
    XIMEA/XIMEACamera.cpp
    # File camera
    FileCamera/FileCamera.cpp
    # Stereo camera
    Stereo/StereoCamera.cpp
    Stereo/DisparityProcessor.cu
    Stereo/StereoRectify.cpp
    Stereo/ExposureFusion.cpp
	Stereo/ExposureFusion.cu
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

source_group("Stereo" FILES
    Stereo/INIReader.h
    Stereo/StereoCamera.h
    Stereo/StereoCamera.cpp
    Stereo/StereoRectify.h
    Stereo/StereoRectify.cpp
    Stereo/ExposureFusion.h
    Stereo/ExposureFusion.cpp
	Stereo/ExposureFusion.cu
)

source_group("FileCamera" FILES
    FileCamera/FileCamera.h
    FileCamera/FileCamera.cpp
)

#source_group("cuda" FILES
#    cuda/NPPJpegCoderKernel.h
#    cuda/NPPJpegCoderKernel.cu
#)

include_directories(
	${PROJECT_SOURCE_DIR}	
	${PROJECT_SOURCE_DIR}/cuda	
	${PROJECT_SOURCE_DIR}/XIMEA	
	${PROJECT_SOURCE_DIR}/PointGrey	
    ${PROJECT_SOURCE_DIR}/FileCamera	
    ${PROJECT_SOURCE_DIR}/Stereo
)

cuda_add_executable(${name}
    ${MY_HEADER_FILES}
    ${MY_SOURCE_FILES}
)

if(NOT WIN32)
    set_target_properties (${name} PROPERTIES COMPILE_DEFINITIONS "SELECTMODE=2")
    set_target_properties (${name} PROPERTIES COMPILE_DEFINITIONS "SELECTCHANNEL=1")
endif()

target_link_libraries(${name}
	cuda
    ${OpenCV_LIBS}
	${NVCUVID_LIB}
    ${CUDA_LIBRARY} 	
	${CUDA_npp_LIBRARY} 
	${CUDA_nppc_LIBRARY} 
    ${CUDA_nppicc_LIBRARY}
    ${CUDA_nppicom_LIBRARY}
	${CUDA_nppidei_LIBRARY}
	${CUDA_nppif_LIBRARY}
	${CUDA_nppig_LIBRARY}
	${CUDA_nppim_LIBRARY}
	${CUDA_nppist_LIBRARY}
	${CUDA_nppisu_LIBRARY}
	${CUDA_nppitc_LIBRARY}
	${CUDA_npps_LIBRARY} 
    ${PTGREY_SDK_LIBRARY_DEBUG} ${XIMEA_SDK_LIBRARY_DEBUG}
    ${PTGREY_SDK_LIBRARY_RELEASE} ${XIMEA_SDK_LIBRARY_RELEASE}
)
