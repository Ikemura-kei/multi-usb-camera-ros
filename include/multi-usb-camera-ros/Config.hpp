#pragma once
#include <opencv2/opencv.hpp>

namespace MultiUsbCamera
{
    namespace Config
    {
        const uint8_t MAX_NUN_CAM = 10;
        const cv::Size SCREEN_RESOLUTION(800, 480);
        const int SET_CAMERA_IMG_WIDTH = 640;
        const int SET_CAMERA_IMG_HEIGHT = 480;
    }
}