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
                thisConfig.deviceId = config[camParamName]["device_id"].as<std::string>();
                thisConfig.imageResizedSize.width = config[camParamName]["width"].as<int>();
                thisConfig.imageResizedSize.height = config[camParamName]["height"].as<int>();
                thisConfig.publishImage = config[camParamName]["publish_image"].as<bool>();
                thisConfig.frameId = config[camParamName]["frame_id"].as<std::string>();

                std::cout << thisConfig << std::endl
                          << std::endl;
                ret.push_back(thisConfig);
            }
        }

        return ret;
    }
}
