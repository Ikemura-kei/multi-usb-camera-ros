/**
 * @file UsbCamera.hpp
 * @author IKEMURA, Kei (ikemurakei2001@gmial.com)
 * @brief This file defines the handler class to a single usb camera.
 * @version 0.1
 * @date 2023-06-27
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

namespace MultiUsbCamera
{

    class UsbCamera
    {
    public:
        /**
         * @brief Default constructor, be aware that the constructed object will not work.
         *
         */
        UsbCamera();
        /**
         * @brief Construct a new usb camera object
         *
         * @param IMG_WIDTH the image height, set directly to the camera (that is, not by post-processing, the raw image will be of this size)
         * @param IMG_HEIGHT the image width, set directly to the camera (that is, not by post-processing, the raw image will be of this size)
         * @param device the index after /dev/videox, (that is 'x')
         */
        UsbCamera(const int IMG_WIDTH, const int IMG_HEIGHT, std::string device);
        /**
         * @brief Copy constructor
         *
         * @param other
         */
        UsbCamera(const UsbCamera &other);
        /**
         * @brief Copy operator
         *
         * @param other the source object to copy from
         * @return UsbCamera& the copied object
         */
        UsbCamera &operator=(const UsbCamera &other);
        /**
         * @brief Get the current frame
         *
         * @param out the output argument to store the current frame
         * @return true if frame retrieval has succeeded.
         * @return false otherwise.
         */
        bool getFrame(cv::Mat &out);
        /**
         * @brief Checks if this camera has been initialized correctly.
         *
         * @return true
         * @return false
         */
        bool initialized();

    private:
        const int IMG_WIDTH;
        const int IMG_HEIGHT;
        int deviceIndex;
        cv::VideoCapture cap;
        bool isInitialized = false;
    };
}