/**
@brief Generic Camera Driver Class
Used when cameras are connected to this computer 
directly
@author Shane Yuan
@date Jan 8, 2018
*/

#ifndef __GENERIC_CAMERA_DRIVER_REAL_H__
#define __GENERIC_CAMERA_DRIVER_REAL_H__

#include "GenCameraDriver.h"

// cuda npp JPEG coder
#include "NPPJpegCoder.h"

namespace cam {
    class RealCamera : public GenCamera {
        protected:
            // buffers to save cuda memory pointer
			std::vector<Imagedata> bufferImgs_singleframe;
		    std::vector<cv::cuda::GpuMat> bufferImgs_cuda;

            // threads to capture images
		    std::vector<std::thread> ths; 
			bool isCaptureThreadRunning;
		    // thread to compress raw image into jpeg
            std::thread thJPEG;
			bool isCompressThreadRunning;

            // status of capturing threads
            // 0: stop capturing images
            // 1: capturing images
            // 2: compress images use jpeg
            std::vector<int> thStatus; 

			// variable to exit all the threads (used for streaming mode)
			// thexit == 1, exit thread
			// thexit == 0, keep running
			int thexit;

            // NPP JPEG coders
		    std::vector<npp::NPPJpegCoder> coders;
        public:

        protected:
            /**
            @brief multi-thread capturing function (raw buffer)
            used for continous mode
            thread function to get images from camera and buffer to vector
            and wait until the next frame (based on fps)
            @param int camInd: index of camera
            */
            void capture_thread_raw_(int camInd);

            /**
            @brief multi-thread captureing function
            used for single mode
            thread function to get images from camera and buffer to vector
            @param int camInd: index of camera
            @param Imagedata & img: output captured image
            */
            void capture_thread_single_(int camInd, Imagedata & img);

            /**
            @brief multi-thread capturing function (jpeg buffer)
            used for continous mode
            thread function to get images from camera and wait for compresss
            thread to compress the raw data into jpeg data
            @param int camInd: index of camera
            */
            void capture_thread_JPEG_(int camInd);

            /**
            @brief single-thread compressing function
            because npp only support single thread, jpeg compress function is not 
            thread safe
            thread function to compress raw image into jpeg data
            and wait until the next frame (based on fps)
            */
            void compress_thread_JPEG_();

			/*************************************************************/
			/*                virtual capturing function                 */
			/*************************************************************/
			/**
			@brief capture single image of single camera
			@param int camInd: input index of camera
			@param Imagedata & img: output captured images
			@return int
			*/
			virtual int captureFrame(int camInd, Imagedata & img) = 0;

        public:
            RealCamera();
            ~RealCamera(); 

			/**
			@brief set capturing mode
			@param GenCamCaptureMode captureMode: capture mode
			@param int size: buffer size
			@return int
			*/
			int setCaptureMode(GenCamCaptureMode captureMode,
				int bufferSize);

		    /**
            @brief wait for recording threads to finish
            @return int
            */
            int waitForRecordFinish();
 
            /**
            @brief start capture threads
            @return int 
            */
            int startCaptureThreads();

            /**
            @brief stop capture threads
            @return int
            */
            int stopCaptureThreads();
    };
};


#endif
