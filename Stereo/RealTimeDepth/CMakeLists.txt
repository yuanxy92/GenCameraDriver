cmake_minimum_required (VERSION 2.8) 
project (FastPatchOF) 

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wno-unknown-pragmas -Wall -std=c++11 -msse4")  #-Wall
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wno-unknown-pragmas -Wall -msse4")  #-Wall

if ( WIN32 )
    set (WIN_LIB_PATH "E:/libs")
	set(OpenCV_DIR ${WIN_LIB_PATH}/OpenCV/x64/vc14/lib)
    set(OpenCV_CONFIG_PATH ${WIN_LIB_PATH}/OpenCV)
    set(Eigen3_DIR ${WIN_LIB_PATH}/Eigen/share/eigen3/cmake)
    set(EIGEN_INCLUDE_DIR ${WIN_LIB_PATH}/Eigen/include/eigen3)
endif()


FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED) 
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})

# # # UNCOMMENT THIS IF YOU WANT TO USE OPENMP PARALLELIZATION
# add_definitions(-DWITH_OPENMP=true)
# FIND_PACKAGE( OpenMP REQUIRED)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
# # 
# # ENABLE PARALLEL FLOW AGGREGATION, CAN LEAD TO DATA RACES, BUT IN PRACTICE HAS ONLY A WEAK NEGATIVE EFFECT ON THE RESULT, [affects only PatGridClass::AggregateFlowDense() ]
# # add_definitions(-DUSE_PARALLEL_ON_FLOWAGGR) 


set(CODEFILES run_dense.cpp oflow.cpp patch.cpp patchgrid.cpp refine_variational.cpp FDF1.0.1/image.c FDF1.0.1/opticalflow_aux.c FDF1.0.1/solver.c depthmap.cpp GhostElemer.cpp)
if ( WIN32 )
    set(CODEFILES ${CODEFILES} depthmap.h DepthMapUpdater.h GhostElemer.h oflow.h patch.h patchgrid.h refine_variational.h DepthMapUpdater.cpp)
endif()

## GrayScale, Optical Flow
#add_executable (run_OF_INT ${CODEFILES})
#set_target_properties (run_OF_INT PROPERTIES COMPILE_DEFINITIONS "SELECTMODE=1")
#set_property(TARGET run_OF_INT APPEND PROPERTY COMPILE_DEFINITIONS "SELECTCHANNEL=1") # use grey-valued image
#TARGET_LINK_LIBRARIES(run_OF_INT ${OpenCV_LIBS})

## RGB, Optical Flow
#add_executable (run_OF_RGB ${CODEFILES})
#set_target_properties (run_OF_RGB PROPERTIES COMPILE_DEFINITIONS "SELECTMODE=1")
#set_property(TARGET run_OF_RGB APPEND PROPERTY COMPILE_DEFINITIONS "SELECTCHANNEL=3") # use RGB image
#TARGET_LINK_LIBRARIES(run_OF_RGB ${OpenCV_LIBS})

# GrayScale, Depth from Stereo
add_executable (run_DE_INT ${CODEFILES})
set_target_properties (run_DE_INT PROPERTIES COMPILE_DEFINITIONS "SELECTMODE=2")
set_property(TARGET run_DE_INT APPEND PROPERTY COMPILE_DEFINITIONS "SELECTCHANNEL=1")
TARGET_LINK_LIBRARIES(run_DE_INT ${OpenCV_LIBS})

## RGB, Depth from Stereo
#add_executable (run_DE_RGB ${CODEFILES})
#set_target_properties (run_DE_RGB PROPERTIES COMPILE_DEFINITIONS "SELECTMODE=2")
#set_property(TARGET run_DE_RGB APPEND PROPERTY COMPILE_DEFINITIONS "SELECTCHANNEL=3")
#TARGET_LINK_LIBRARIES(run_DE_RGB ${OpenCV_LIBS})

