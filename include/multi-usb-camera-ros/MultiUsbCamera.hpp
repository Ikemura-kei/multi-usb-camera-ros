/**
 * @file MultiUsbCamera.hpp
 * @author IKEMURA, Kei (ikemurakei2001@gmial.com)
 * @brief This file defines a multi-camera handler class for usb cameras. Note there is not multi-thread support, for example, 
 *          if you want to update each camera in a separate frame that is not possible, for that, refer to the multi-thread node
 * @version 0.1
 * @date 2023-06-27
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include <UsbCamera.hpp>
#include <vector>

namespace MultiUsbCamera
{
    class MultiUsbCamera
    {
    public:
        enum LogCode
        {
            LOG_NO_CAMERA_SET = -2,
            LOG_OK = -1,
        };
        /**
         * @brief Construct a new multi-usb camera object
         *
         * @param deviceIndexes the vector of device indexes, like {0, 1, 2, 3}..., corresponding to /dev/video0, /dev/video1, /dev/video2, and /dev/video3,
         */
        MultiUsbCamera(std::vector<CameraConfig> configurations, ros::NodeHandle &nh)
        {
            this->numCam = configurations.size();
            if (this->numCam <= 0)
                return;

            for (std::vector<CameraConfig>::iterator it = configurations.begin(); it != configurations.end(); it++)
                usbCameras.push_back(UsbCamera(Config::SET_CAMERA_IMG_WIDTH, Config::SET_CAMERA_IMG_HEIGHT, *it, nh));
        }
        /**
         * @brief Set the current activated camera
         *
         * @param idx the index of the current activated camera, 0-started
         */
        void setCameraPointer(int idx)
        {
            curCamPointer = idx % this->numCam;

            int cnt = 0;
            for (auto it = usbCameras.begin(); it != usbCameras.end(); it++, cnt++)
                it->setActivation(curCamPointer == cnt); // set only the selected camera as activated
        }
        /**
         * @brief Get the frame object
         *
         * @param out the output parameter to store the current frame.
         * @return true if the frame retrieval succeeded.
         * @return false otherwise
         */
        bool getFrame(cv::Mat &out)
        {
            return usbCameras[curCamPointer].getFrameAtOnce(out); // note, as we use vector here, the index operation is bit slower, but the benefit is allowing dynamic #camera configuration.
        }
        /**
         * @brief Checks if all the cameras are initialized correctly.
         *
         * @return int if all succeeded, -1 is returned. Otherwise the index of the first failed camera is returned.
         */
        int initialized()
        {
            if (usbCameras.size() <= 0)
                return LOG_NO_CAMERA_SET;

            // -- check if all cameras are initialized --
            int idx = 0;
            for (auto it = usbCameras.begin(); it != usbCameras.end(); it++)
            {
                if (!it->initialized())
                    return idx;
                idx++;
            }

            return LOG_OK;
        }

    private:
        int curCamPointer = 0; // the index to the currently activated camera.
        int numCam = 0;
        std::vector<UsbCamera> usbCameras;
    };
}