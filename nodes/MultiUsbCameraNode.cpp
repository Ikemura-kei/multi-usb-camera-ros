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

static int curCamIdx = 0;
static void camCmdCb(const robot_msgs::CameraCmdConstPtr &msg)
{
    curCamIdx = msg->camera_id % MultiUsbCamera::Config::NUM_CAM;
}

int main(int ac, char **av)
{
    ros::init(ac, av, "multi_camera_usb_node");
    ros::Time::init();

    ros::NodeHandle nh("~");
    ros::Subscriber camCmdSub = nh.subscribe<robot_msgs::CameraCmd>("/camera_cmd", 10, camCmdCb);

    // -- initialize and check if camera has started correctly --
    MultiUsbCamera::MultiUsbCamera<MultiUsbCamera::Config::NUM_CAM> multiCameraHandler(false);
    int initState = multiCameraHandler.initialized(); // return -1 if ok, otherwise the index of the failed camera (0 starting)
    if (initState != -1)
    {
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