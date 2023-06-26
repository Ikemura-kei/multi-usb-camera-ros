#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

namespace MultiUsbCamera
{

    class UsbCamera
    {
    public:
        UsbCamera();

        UsbCamera(const int IMG_WIDTH, const int IMG_HEIGHT, std::string device);

        UsbCamera(const UsbCamera &other);

        UsbCamera &operator=(const UsbCamera &other);

        bool getFrame(cv::Mat &out);

        bool initialized();

    private:
        const int IMG_WIDTH;
        const int IMG_HEIGHT;
        int deviceIndex;
        cv::VideoCapture cap;
        bool isInitialized = false;
    };
}