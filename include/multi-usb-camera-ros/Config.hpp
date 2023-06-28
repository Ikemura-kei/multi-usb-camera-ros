/**
 * @file Config.hpp
 * @author IKEMURA, Kei (ikemurakei2001@gmial.com)
 * @brief the static configurations for the camera driver
 * @version 0.1
 * @date 2023-06-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */
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