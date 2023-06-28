#pragma once

#include <UsbCamera.hpp>
#include <iostream>
#include <vector>
#include <UsbCamera.hpp>

namespace MultiUsbCamera
{
    std::vector<CameraConfig> getCameraConfigs(const std::string &pathToYaml);
} 
