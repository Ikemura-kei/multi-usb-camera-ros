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
#include <Config.hpp>

namespace MultiUsbCamera
{
    struct CameraConfig
    {
        std::string cameraName = "bocchi";
        std::string deviceId = "0";
        int8_t cvRotateFlag = -1; // -1 refers to not rotating, tho not defined in OpenCV's rotate flags
        cv::Size imageResizedSize = cv::Size(Config::SET_CAMERA_IMG_WIDTH, Config::SET_CAMERA_IMG_HEIGHT);
    };

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
         * @param config the camera configuration object
         */
        UsbCamera(const int IMG_WIDTH, const int IMG_HEIGHT, CameraConfig config);
        /**
         * @brief Copy constructor
         *
         * @param other the source object to copy from
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
         * @brief Get the current frame, with resize and rotation performed
         *
         * @param out the output argument to store the current frame
         * @return true if frame retrieval has succeeded.
         * @return false otherwise.
         */
        bool getFrame(cv::Mat &out);
        /**
         * @brief Checks if this camera has been initialized correctly.
         *
         * @return true if the camera has been initialized correctly
         * @return false otherwise
         */
        bool initialized();

    private:
        const int IMG_WIDTH;
        const int IMG_HEIGHT;
        cv::VideoCapture cap;
        bool isInitialized = false;
        CameraConfig config;
    };
}

inline std::ostream &operator<<(std::ostream &os, MultiUsbCamera::CameraConfig const &cameraConfig)
{
    return os << "[Camera Name]: {" << cameraConfig.cameraName << "},\n[Camera Index]: {/dev/video" << cameraConfig.deviceId << "},\n[Image Resized Size]: {" << cameraConfig.imageResizedSize << "}\n[Camera Rotation Flag]: {" << std::to_string(cameraConfig.cvRotateFlag) << "}";
}