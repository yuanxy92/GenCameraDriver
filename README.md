# GenCameraDriver
## Introduction
Generic Industry Camera Driver, support capturing (software synchronization), JPEG compression, Saving to video

Now two kinds of cameras are support 

1. PointGrey (only for color cameras, tested with FL3-U3-120S3C)
2. XIMEA (tested with MC031CG-SY-UB)

Examples are in main.cpp

## Dependency (only tested on Windows 10)
1. OpenCV 3.4.0 with CUDA 9.1
2. Intel MKL (Math Kernel Library), optional but recommended
3. CUDA 9.1
4. Visual Studio 2015
5. CMake >= 3.10 (FindCUDA in low version can not work for CUDA 9.1 since NPP library has changed a lot in CUDA 9)
6. Spinnaker SDK for PointGrey cameras
7. XIMEA SDK for XIMEA cameras

## Downloads
Camera SDKs downloads:

link: https://pan.baidu.com/s/1mhBJ0i0 passwd: pafn

Softwares (CUDA, cudnn, vs2015, cmake, ...) downloads:

link: https://pan.baidu.com/s/1ragGrik passwd: j3ew

Pre-compiled windows libs: (Windows 10 + VS2015 + CUDA9.1 + MKL)

link: https://pan.baidu.com/s/1dxhAGI passwd: sgw4

Good luck!
