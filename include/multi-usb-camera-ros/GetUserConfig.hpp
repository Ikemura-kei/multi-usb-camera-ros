#pragma once

#include <UsbCamera.hpp>
#include <iostream>
#include <vector>
#include <UsbCamera.hpp>

namespace MultiUsbCamera
{
    /**
     * @brief Get the camera configuration objects from a YAML file
     * 
     * @param pathToYaml the absolute path to the YAML configuration file
     * @return std::vector<CameraConfig> the list of all camera configurations
     */
    std::vector<CameraConfig> getCameraConfigs(const std::string &pathToYaml);
} 
