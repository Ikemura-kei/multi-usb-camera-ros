/**
 * @file GetUserConfig.cpp
 * @author IKEMURA, Kei (ikemurakei2001@gmial.com)
 * @brief Utility function to retrieve user configurations on the cameras from YAML file
 * @version 0.1
 * @date 2023-06-29
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <GetUserConfig.hpp>
#include <yaml-cpp/yaml.h>

namespace MultiUsbCamera
{
    static uint8_t getCvRotateCode(int angle)
    {
        if (angle == 0)
            return -1;
        else if (angle == 180)
            return cv::ROTATE_180;
        else if (angle == 90)
            return cv::ROTATE_90_CLOCKWISE;
        else if (angle == -90)
            return cv::ROTATE_90_COUNTERCLOCKWISE;
        else
            return -1;
    }

    std::vector<CameraConfig> getCameraConfigs(const std::string &pathToYaml)
    {
        std::vector<CameraConfig> ret;
        YAML::Node config = YAML::LoadFile(pathToYaml);

        for (int i = 0; i < Config::MAX_NUN_CAM; i++)
        {
            std::string camParamName = std::string("camera") + std::to_string(i);

            if (!config[camParamName]) // param not found
                continue;
            else
            {
                CameraConfig thisConfig;
                thisConfig.cameraName = config[camParamName]["name"].as<std::string>();
                thisConfig.cvRotateFlag = getCvRotateCode(config[camParamName]["rotate_angle"].as<int>());
                thisConfig.busId = config[camParamName]["bus_id"].as<std::string>();
                thisConfig.imageResizedSize.width = config[camParamName]["width"].as<int>();
                thisConfig.imageResizedSize.height = config[camParamName]["height"].as<int>();
                thisConfig.publishImage = config[camParamName]["publish_image"].as<bool>();
                thisConfig.frameId = config[camParamName]["frame_id"].as<std::string>();
                thisConfig.rawWidth = config[camParamName]["raw_width"].as<int>();
                thisConfig.rawHeight = config[camParamName]["raw_height"].as<int>();

                // std::cout << thisConfig << std::endl
                //           << std::endl;
                ret.push_back(thisConfig);
            }
        }

        return ret;
    }
}
