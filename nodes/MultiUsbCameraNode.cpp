/**
 * @file MultiUsbCameraNode.cpp
 * @author IKEMURA, Kei (ikemurakei2001@gmial.com)
 * @brief This file defines the node to show and switch different camera sources
 * @version 0.1
 * @date 2023-06-27
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <ros/ros.h>

#include <UsbCamera.hpp>
#include <MultiUsbCamera.hpp>
#include <Config.hpp>
#include <robot_msgs/CameraCmd.h>

#include <yaml-cpp/yaml.h>

// -- globals --
static int numCam = 0;
static int curCamIdx = 0;

// -- helper functions --
uint8_t getCvRotateCode(int angle);
std::vector<MultiUsbCamera::CameraConfig> getCameraConfigs(const std::string &pathToYaml);
static void camCmdCb(const robot_msgs::CameraCmdConstPtr &msg);

int main(int ac, char **av)
{
    ros::init(ac, av, "multi_camera_usb_node");
    ros::Time::init();

    ros::NodeHandle nh("~");
    ros::Subscriber camCmdSub = nh.subscribe<robot_msgs::CameraCmd>("/camera_cmd", 10, camCmdCb);

    // -- get camera configurations --
    std::string pathToYaml = "/home/rm/engineer_2023_ws/config/multi_usb_camera.yaml";
    std::vector<MultiUsbCamera::CameraConfig> cameraConfigs = getCameraConfigs(pathToYaml);

    // -- initialize and check if camera has started correctly --
    MultiUsbCamera::MultiUsbCamera multiCameraHandler(cameraConfigs);
    int initState = multiCameraHandler.initialized(); // return -1 if ok, otherwise the index of the failed camera (0 starting)
    if (initState != MultiUsbCamera::MultiUsbCamera::LogCode::LOG_OK)
    {
        if (initState == MultiUsbCamera::MultiUsbCamera::LogCode::LOG_NO_CAMERA_SET)
            ROS_FATAL_STREAM("No camera set!");
        else
            ROS_FATAL_STREAM("Camera initialization failed at " << std::to_string(initState + 1));
        return 1;
    }

    // -- create a visualization window that covers the entire screen --
    cv::namedWindow("video", cv::WINDOW_NORMAL);
    cv::setWindowProperty("video", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    // -- static variables --
    static cv::Mat curFrame;
    static cv::Mat showFrame;

    // -- the main loop --
    while (ros::ok())
    {
        ros::spinOnce();

        multiCameraHandler.setCameraPointer(curCamIdx);
        bool success = multiCameraHandler.getFrame(curFrame);
        cv::rotate(curFrame, curFrame, cv::ROTATE_180);

        if (!success)
            curFrame = cv::Mat::zeros(MultiUsbCamera::Config::SCREEN_RESOLUTION, CV_8UC3);
        else
            cv::resize(curFrame, showFrame, MultiUsbCamera::Config::SCREEN_RESOLUTION);

        cv::imshow("video", showFrame);

        char k = cv::waitKey(1);
        if (k == 'q')
            break;
    }
}

uint8_t getCvRotateCode(int angle)
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

std::vector<MultiUsbCamera::CameraConfig> getCameraConfigs(const std::string &pathToYaml)
{
    std::vector<MultiUsbCamera::CameraConfig> ret;

    YAML::Node config = YAML::LoadFile(pathToYaml);

    for (int i = 0; i < MultiUsbCamera::Config::MAX_NUN_CAM; i++)
    {
        std::string camParamName = std::string("camera") + std::to_string(i);

        if (!config[camParamName]) // param not found
            continue;
        else
        {
            MultiUsbCamera::CameraConfig thisConfig;
            thisConfig.cameraName = config[camParamName]["name"].as<std::string>();
            thisConfig.cvRotateFlag = getCvRotateCode(config[camParamName]["rotate_angle"].as<int>());
            thisConfig.deviceId = config[camParamName]["device_id"].as<std::string>();
            thisConfig.imageResizedSize.width = config[camParamName]["width"].as<int>();
            thisConfig.imageResizedSize.height = config[camParamName]["height"].as<int>();

            std::cout << thisConfig << std::endl;
            ret.push_back(thisConfig);
            numCam += 1;
        }
    }

    return ret;
}

static void camCmdCb(const robot_msgs::CameraCmdConstPtr &msg)
{
    curCamIdx = msg->camera_id % numCam;
}