/**
 * @file UsbCamera.cpp
 * @author IKEMURA, Kei (ikemurakei2001@gmial.com)
 * @brief The definition of the single usb camera handler class
 * @version 0.1
 * @date 2023-06-27
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <UsbCamera.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <regex>

namespace MultiUsbCamera
{

    UsbCamera::UsbCamera() : IMG_WIDTH(640), IMG_HEIGHT(480)
    {
    }

    UsbCamera::UsbCamera(const UsbCamera &other) : IMG_HEIGHT(other.IMG_HEIGHT), IMG_WIDTH(other.IMG_WIDTH)
    {
        (*this) = other;
    }

    UsbCamera &UsbCamera::operator=(const UsbCamera &other)
    {
        this->isInitialized = other.isInitialized;
        this->cap = other.cap;
        this->deviceIndex = other.deviceIndex;

        return *this;
    }

    UsbCamera::UsbCamera(const int IMG_WIDTH, const int IMG_HEIGHT, std::string device) : IMG_HEIGHT(IMG_HEIGHT), IMG_WIDTH(IMG_WIDTH)
    {
        std::string gstreamerStr = "v4l2src device=DEVICE_PLACEHOLDER ! image/jpeg, width=IMAGE_WIDTH_PLACEHOLDER, height=IMAGE_HEIGHT_PLACEHOLDER, framerate=30/1 ! jpegdec ! videoconvert ! appsink";
        std::string dev = "/dev/video" + device;
        gstreamerStr = std::regex_replace(gstreamerStr, std::regex("DEVICE_PLACEHOLDER"), dev);
        gstreamerStr = std::regex_replace(gstreamerStr, std::regex("IMAGE_WIDTH_PLACEHOLDER"), std::to_string(IMG_WIDTH));
        gstreamerStr = std::regex_replace(gstreamerStr, std::regex("IMAGE_HEIGHT_PLACEHOLDER"), std::to_string(IMG_HEIGHT));
        std::cout << gstreamerStr << std::endl;
        // this->cap = cv::VideoCapture(gstreamerStr, cv::CAP_GSTREAMER);

        int deviceIndex = std::stoi(device);
        this->deviceIndex = deviceIndex;
        this->cap = cv::VideoCapture(deviceIndex);

        if (!cap.isOpened())
        {
            this->isInitialized = false;
            return;
        }

        // -- test read --
        cv::Mat testFrame;
        cap >> testFrame;

        if (testFrame.empty())
        {
            this->isInitialized = false;
            return;
        }

        this->isInitialized = true;
    }

    bool UsbCamera::getFrame(cv::Mat &out)
    {
        cap >> out;

        return !out.empty() && cap.isOpened();
    }

    bool UsbCamera::initialized()
    {
        return this->isInitialized;
    }
}