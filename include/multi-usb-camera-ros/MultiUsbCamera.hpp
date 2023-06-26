#pragma once
#include <UsbCamera.hpp>

namespace MultiUsbCamera
{
    template <int NUM_CAM>
    class MultiUsbCamera
    {
    public:
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

        void setCameraPointer(int idx)
        {
            curCamPointer = idx % NUM_CAM;
        }

        bool getFrame(cv::Mat &out)
        {
            return usbCamera[curCamPointer].getFrame(out);
        }

        int initialized()
        {
            // -- check if all cameras are initialized --
            for (int i = 0; i < NUM_CAM; i++)
                if (!usbCamera[i].initialized())
                    return i;

            return -1;
        }

    private:
        int curCamPointer = 0;
        UsbCamera usbCamera[NUM_CAM];
    };
}