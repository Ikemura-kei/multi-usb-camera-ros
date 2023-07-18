#pragma once
#include <iostream>
#include <vector>
#include <UsbCamera.hpp>

namespace MultiUsbCamera
{
    bool getCameraIdByBusId(std::vector<MultiUsbCamera::CameraConfig> &cameraConfigs, std::vector<MultiUsbCamera::UsbCamera> &cameras, bool updateCamera);
}