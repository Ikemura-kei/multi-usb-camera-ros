/**
 * @file MultiThreadedMultiUsbCameraNode.cpp
 * @author IKEMURA, Kei (ikemurakei2001@gmial.com)
 * @brief The multi-threaded version of multi-camera video retrieval
 * @version 0.1
 * @date 2023-06-29
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <thread>
#include <ros/ros.h>
#include <Config.hpp>
#include <UsbCamera.hpp>
#include <GetUserConfig.hpp>
#include <chrono>
#include <robot_msgs/CameraCmd.h>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <regex>

#define VERBOSE false

std::string exec(const char *cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
    {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }
    return result;
}

// -- helper functions --
static void camCmdCb(const robot_msgs::CameraCmdConstPtr &msg); // camera switching command from micro-controllers
void cameraThreadFunc(MultiUsbCamera::UsbCamera *usbCamera);

// -- globals --
static int numCam = 0;
static int curCamIdx = 0;

int main(int ac, char **av)
{
    ros::init(ac, av, "multi_threaded_multi_usb_camera_node");
    ros::NodeHandle nh("~");
    ros::Subscriber camCmdSub = nh.subscribe<robot_msgs::CameraCmd>("/camera_cmd", 10, camCmdCb);

    // -- get camera configurations --
    std::string pathToYaml = "";
    if (!nh.getParam("config_file", pathToYaml))
    {
        ROS_FATAL_STREAM("Camera configuration file missing!");
        return 1;
    }
    std::vector<MultiUsbCamera::CameraConfig> cameraConfigs = MultiUsbCamera::getCameraConfigs(pathToYaml);
    numCam = cameraConfigs.size();

    // -- get the device indexes that correspond to each bus id --
    std::string s = exec("v4l2-ctl --list-devices");
    std::string delimiter = "\n";
    size_t pos = 0;
    int counter = 0;
    std::string token;
    while ((pos = s.find(delimiter)) != std::string::npos)
    {
        token = s.substr(0, pos);

        // -- this is the line containing bus id --
        for (auto it = cameraConfigs.begin(); it != cameraConfigs.end(); it++)
        {
            if (it->deviceId != "-1")
                continue;

            if (token.find(it->busId) != std::string::npos)
            {
                size_t nextPos = s.find(delimiter, pos + 1);
                std::string id = s.substr(pos + 1, nextPos - pos - 1);
                it->deviceId = id.back();
                // std::cout << "last second: " << id[id.size() - 2] << std::endl;
                if (id[id.size() - 2] != 'o')
                {
                    char cp = id.back();
                    std::string s;
                    std::stringstream ss;
                    ss << id[id.size() - 2] << cp;
                    ss >> s;
                    it->deviceId = s;
                }
                break;
            }
        }

        s.erase(0, pos + delimiter.length());

        counter += 1;
    }

    // -- print configurations for sanity checks --
    for (auto it = cameraConfigs.begin(); it != cameraConfigs.end(); it++)
    {
        if (it->deviceId == "-1")
        {
            ROS_WARN_STREAM("--> Camera for {" << it->busId << "} was not found!!!");
            // cameraConfigs.erase(it);
        }
        else{
            ROS_INFO_STREAM("--> Camera for {" << it->busId << "} was found successfully :)");
        }
        std::cout << *it << std::endl
                  << std::endl;
    }

    // -- initialize cameras --
    std::vector<MultiUsbCamera::UsbCamera> cameras;
    for (auto it = cameraConfigs.begin(); it != cameraConfigs.end(); it++)
    {
        ros::Duration(0.5).sleep();
        cameras.push_back(MultiUsbCamera::UsbCamera(MultiUsbCamera::Config::SET_CAMERA_IMG_WIDTH, MultiUsbCamera::Config::SET_CAMERA_IMG_HEIGHT, *it, nh));
    }

    // -- create a visualization window that covers the entire screen --
    cv::namedWindow("video", cv::WINDOW_NORMAL);
    cv::setWindowProperty("video", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    // -- initialize thread objects --
    std::vector<std::thread> threads;
    for (auto it = cameras.begin(); it != cameras.end(); it++)
        threads.push_back(std::thread(cameraThreadFunc, &(*it)));

    // -- static variables --
    static cv::Mat curFrame;

    // -- main loop --
    while (ros::ok())
    {
        ros::spinOnce();

        bool success = cameras[curCamIdx].getFrame(curFrame);
        // std::cout << curCamIdx << std::endl;
        if (!success)
            continue;

        cv::imshow("video", curFrame);
        char k = cv::waitKey(1);
        if (k == 'q')
            break;
    }

    // -- wait for threads to finish --
    for (auto it = threads.begin(); it != threads.end(); it++)
        it->join();
}

void cameraThreadFunc(MultiUsbCamera::UsbCamera *usbCamera)
{
#if VERBOSE
    auto lastTime = std::chrono::high_resolution_clock::now();
#endif
    while (ros::ok())
    {
#if VERBOSE
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - lastTime);
        lastTime = std::chrono::high_resolution_clock::now();
#endif

        usbCamera->updateFrame();

#if VERBOSE
        std::cout << usbCamera->getConfig().cameraName << ": " << duration.count() / 1000000.0 << std::endl;
#endif
    }
}

static void camCmdCb(const robot_msgs::CameraCmdConstPtr &msg)
{
    curCamIdx = msg->camera_id % numCam;
}