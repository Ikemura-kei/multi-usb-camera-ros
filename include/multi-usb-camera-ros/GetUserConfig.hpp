/**
 * @file GetUserConfig.hpp
 * @author IKEMURA, Kei (ikemurakei2001@gmial.com)
 * @brief Utility function to retrieve user configurations on the cameras from YAML file
 * @version 0.1
 * @date 2023-06-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */
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
