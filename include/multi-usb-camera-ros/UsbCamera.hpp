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
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <mutex>

namespace MultiUsbCamera
{
    /**
     * @brief the configuration structure for the camera
     * 
     */
    struct CameraConfig
    {
        std::string cameraName = "bocchi"; // custom identifier, will be printed to the presented image for identification
        std::string deviceId = "0"; // the index before /dev/video, for example, for /dev/video2 this argument shall be "2"
        int8_t cvRotateFlag = -1; // -1 refers to not rotating, tho not defined in OpenCV's rotate flags
        cv::Size imageResizedSize = cv::Size(Config::SET_CAMERA_IMG_WIDTH, Config::SET_CAMERA_IMG_HEIGHT);
        bool publishImage = false; // if publishing the image from this camera is needed
        std::string frameId = ""; // the frame id associated with the published image
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
        UsbCamera(const int IMG_WIDTH, const int IMG_HEIGHT, CameraConfig config, ros::NodeHandle &nh);
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
         * @brief Directly get the current frame immediately
         *
         * @param out the output argument to store the current frame
         * @return true if frame retrieval has succeeded.
         * @return false otherwise.
         */
        bool getFrameAtOnce(cv::Mat &out);
        /**
         * @brief Checks if this camera has been initialized correctly.
         *
         * @return true if the camera has been initialized correctly
         * @return false otherwise
         */
        bool initialized();
        /**
         * @brief Update the current frame, and saved for retrieving later
         *
         * @return true if the frame update has succeeded
         * @return false otherwise (say the camera has disconnected)
         */
        bool updateFrame();
        /**
         * @brief Set the the activation state of the camera
         *
         * @param activation if false, no image will be published to the topic (in case publishing was intended) when getFrameAtOnce() has been called
         */
        void setActivation(bool activation);
        /**
         * @brief Get the most recent updated frame by the updateFrame() function, the udpate-get frame pair is suitable for multi-thread operation of the camera
         *
         * @param out the output argument to hold the most recent frame from this camera
         * @return true if the image is valid
         * @return false otherwise
         */
        bool getFrame(cv::Mat &out);
        /**
         * @brief Get the configuration of this camera, by copy, so basically read-only
         *
         * @return CameraConfig the configuration object
         */
        CameraConfig getConfig() { return config; }

    private:
        void publish();
        void cvImgToImageMsg(cv::Mat img, sensor_msgs::Image &msg);
        const int IMG_WIDTH;
        const int IMG_HEIGHT;
        cv::VideoCapture cap;
        bool isInitialized = false;
        CameraConfig config;
        ros::Publisher imgPub;
        std_msgs::Header header;
        cv::Mat frame;
        bool isActivated = false;
        std::mutex frameMutex;
    };
}
/**
 * @brief The operator overloaded to print-out the configuration to terminal
 *
 * @param os input output stream object
 * @param cameraConfig the camera configuration
 * @return std::ostream& output stream object
 */
inline std::ostream &operator<<(std::ostream &os, MultiUsbCamera::CameraConfig const &cameraConfig)
{
    return os << "[Camera Name]: {" << cameraConfig.cameraName << "},\n[Camera Index]: {/dev/video" << cameraConfig.deviceId << "},\n[Image Resized Size]: {" << cameraConfig.imageResizedSize << "}\n[Camera Rotation Flag]: {" << std::to_string(cameraConfig.cvRotateFlag) << "}\n[Frame ID]: {" << cameraConfig.frameId << "}\n[Publish Image]: {" << std::boolalpha << cameraConfig.publishImage << "}";
}