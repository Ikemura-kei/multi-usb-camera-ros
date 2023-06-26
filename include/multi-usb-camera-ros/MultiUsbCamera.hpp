/**
 * @file MultiUsbCamera.hpp
 * @author IKEMURA, Kei (ikemurakei2001@gmial.com)
 * @brief This file defines a multi-camera handler class for usb cameras
 * @version 0.1
 * @date 2023-06-27
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include <UsbCamera.hpp>

namespace MultiUsbCamera
{
    template <int NUM_CAM>
    class MultiUsbCamera
    {
    public:
        /**
         * @brief Construct a new multi-usb camera object
         *
         * @param SKIP_WEBCAM whether to skip the embedded webcam (by default at /dev/video0), some computers do not have webcams, for those, set this parameters to false.
         */
        MultiUsbCamera(const bool SKIP_WEBCAM)
        {
            int initIndex = 0;
            int endIndex = NUM_CAM;

            // -- by default, some computers have integrated camera (webcam), so we skip it. --
            if (SKIP_WEBCAM)
            {
                initIndex += 1;
                endIndex += 1;
            }

            int camCounter = 0;
            for (int i = initIndex; i < endIndex; i++)
            {
                usbCamera[camCounter] = UsbCamera(640, 480, std::to_string(i * 2));
                camCounter += 1;
            }
        }
        /**
         * @brief Set the current activated camera
         *
         * @param idx the index of the current activated camera, 0-started
         */
        void setCameraPointer(int idx)
        {
            curCamPointer = idx % NUM_CAM;
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
            return usbCamera[curCamPointer].getFrame(out);
        }
        /**
         * @brief Checks if all the cameras are initialized correctly.
         *
         * @return int if all succeeded, -1 is returned. Otherwise the index of the first failed camera is returned.
         */
        int initialized()
        {
            // -- check if all cameras are initialized --
            for (int i = 0; i < NUM_CAM; i++)
                if (!usbCamera[i].initialized())
                    return i;

            return -1;
        }

    private:
        int curCamPointer = 0; // the index to the currently activated camera.
        UsbCamera usbCamera[NUM_CAM];
    };
}