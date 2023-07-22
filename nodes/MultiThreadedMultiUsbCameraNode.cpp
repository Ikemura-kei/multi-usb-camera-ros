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
#include <robot_msgs/CameraCmd.h>
#include <GetCameraIdByBusId.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <regex>

#define VERBOSE false

// -- helper functions --
static void camCmdCb(const robot_msgs::CameraCmdConstPtr &msg); // camera switching command from micro-controllers
void cameraThreadFunc(MultiUsbCamera::UsbCamera *usbCamera);
void drawConnectionStates(cv::Mat &frame, std::vector<MultiUsbCamera::UsbCamera> cameras);

// -- globals --
static int numCam = 0;
static int curCamIdx = 0;
static bool allCameraInitialized = false;
static ros::Time lastReconnectionAttemptTime;
static const float RECONNECTION_ATTEMPT_PERIOD = 1.75; // seconds

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
    std::vector<MultiUsbCamera::UsbCamera> cameras;
    allCameraInitialized = MultiUsbCamera::getCameraIdByBusId(cameraConfigs, cameras, false);
    lastReconnectionAttemptTime = ros::Time::now();

    // -- initialize cameras --
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

        // -- if this is true, some cameras were not found at initialization time, so we keep finding them with a fixed period --
        if (!allCameraInitialized && (ros::Time::now() - lastReconnectionAttemptTime).toSec() >= RECONNECTION_ATTEMPT_PERIOD)
        {
            lastReconnectionAttemptTime = ros::Time::now();
            allCameraInitialized = MultiUsbCamera::getCameraIdByBusId(cameraConfigs, cameras, true);
            if (!allCameraInitialized)
            {
                for (auto c : cameraConfigs)
                {
                    if (c.deviceId == "-1")
                    {
                        ROS_WARN_STREAM("--> Reconnection failed, camera " << c.cameraName << " is still not connected");
                    }
                }
            }
        }

        // -- retreive the current frame of the currently selected camera --
        bool success = cameras[curCamIdx].getFrame(curFrame);

        if (cameras[curCamIdx].getConfig().cameraName == "camera_states")
        {
            drawConnectionStates(curFrame, cameras);
        }
        else
        {
            curFrame.setTo(200, curFrame > 249);
        }

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
    static ros::Rate rate(20); // Clamp 20Hz
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
        rate.sleep();

#if VERBOSE
        std::cout << usbCamera->getConfig().cameraName << ": " << duration.count() / 1000000.0 << std::endl;
#endif
    }
}

static void camCmdCb(const robot_msgs::CameraCmdConstPtr &msg)
{
    curCamIdx = msg->camera_id % numCam;
}

void drawConnectionStates(cv::Mat &frame, std::vector<MultiUsbCamera::UsbCamera> cameras)
{
    int cnt = 0;
    for (auto cam : cameras)
    {
        if (cam.getConfig().busId == "extra")
            continue;
        // std::cout << "--> Camera is " << cam.isCameraOK << std::endl;
        cnt += 1;
        std::string cameraString = std::string("<") + cam.getConfig().cameraName + std::string(">");
        cv::putText(frame, cameraString, cv::Point(200, 71 + cnt * 36), cv::FONT_ITALIC, 1.12114514, cv::Scalar(255, 12, 120), 3);
        std::string stateString = (cam.isCameraOK ? std::string("TRUE") : std::string("FALSE"));
        cv::putText(frame, stateString, cv::Point(589, 71 + cnt * 35), cv::FONT_ITALIC, 1.12114514, (cam.isCameraOK ? cv::Scalar(2, 129, 2) : cv::Scalar(15, 15, 255)), 3);
    }
}