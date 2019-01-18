## CMakeLists.txt for H264Encoder
# author: zhu-ty
# date: Jan 18, 2019
#

cmake_minimum_required (VERSION 3.10)
set(name "SKEncoder")
project ( ${name} )

# c++ version
set (CMAKE_CXX_STANDARD 11)

# opencv 
if ( WIN32 )
    set(OpenCV_CONFIG_PATH "E:/libs/OpenCV")
    set(OpenCV_DIR "E:/libs/OpenCV/x64/vc14/lib")
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

set(headers_h
    NvEncoder/NvEncoder/nvEncodeAPI.h
    NvEncoder/NvEncoder/NvEncoder.h
    NvEncoder/NvEncoder/NvEncoderCuda.h
    NvEncoder/Utils/Logger.h
    NvEncoder/Utils/NvEncoderCLIOptions.h
    NvEncoder/Utils/NvCodecUtils.h
    SKEncoder.h
    SKCommon.hpp
)

set(source_cpp
    NvEncoder/NvEncoder/NvEncoder.cpp
    NvEncoder/NvEncoder/NvEncoderCuda.cpp
    SKEncoder.cpp
    main.cpp
)



# file (GLOB_RECURSE headers_h "./*.h")
# file (GLOB_RECURSE headers_hpp "./*.hpp")
# file (GLOB_RECURSE source_cpp "./*.cpp")

#list(REMOVE_ITEM source_cpp "build/*")
FOREACH(item ${source_cpp})
  IF(${item} MATCHES "build/*")
    LIST(REMOVE_ITEM source_cpp ${item})
  ENDIF(${item} MATCHES "build/*")
ENDFOREACH(item)

set(MY_HEADER_FILES
    ${headers_h}
    ${headers_hpp}
)

set(MY_SOURCE_FILES 
    ${source_cpp}
)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/NvEncoder
)

cuda_add_executable(${name}
    ${MY_HEADER_FILES}
    ${MY_SOURCE_FILES}
)

#message (${PROJECT_SOURCE_DIR}/lib/x64/nvcuvid.lib)

if( WIN32 )
    set(NVCUVID_LIB ${PROJECT_SOURCE_DIR}/lib/x64/nvcuvid.lib)
else()
    set(NVCUVID_LIB 
    ${PROJECT_SOURCE_DIR}/lib/linux/stubs/x86_64/libnvcuvid.so
    ${PROJECT_SOURCE_DIR}/lib/linux/stubs/x86_64/libnvidia-encode.so)
endif()


target_link_libraries(${name}
    cuda
	#${PROJECT_SOURCE_DIR}/lib/x64/nvcuvid.lib  
    ${NVCUVID_LIB}
    ${OpenCV_LIBS}
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
)
