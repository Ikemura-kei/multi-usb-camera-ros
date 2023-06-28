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
#include <GetUserConfig.hpp>
#include <yaml-cpp/yaml.h>

#define EVAL_PERFORMANCE false

// -- globals --
static int numCam = 0;
static int curCamIdx = 0;

// -- helper functions --
static void camCmdCb(const robot_msgs::CameraCmdConstPtr &msg);

#if EVAL_PERFORMANCE
class ExeEvaluator
{
public:
    ExeEvaluator(std::string procedureName) : procedureName(procedureName) {}

    void tic()
    {
        start = std::chrono::high_resolution_clock::now();
    }

    void tac()
    {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        ROS_INFO_STREAM("[multi-usb-camera]: Execution duration for " << procedureName << " took: " << duration.count() / 1000.0f << " ms.");
    }

private:
    std::string procedureName;
    std::chrono::_V2::system_clock::time_point start;
};

static ExeEvaluator loopEvaluator("Loop");
static ExeEvaluator getFrameEvaluator("Get Frame");
#endif

int main(int ac, char **av)
{
    ros::init(ac, av, "multi_camera_usb_node");
    ros::Time::init();

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

    // -- initialize and check if camera has started correctly --
    MultiUsbCamera::MultiUsbCamera multiCameraHandler(cameraConfigs, nh);
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
#if EVAL_PERFORMANCE
        loopEvaluator.tic();
#endif
        ros::spinOnce();

        multiCameraHandler.setCameraPointer(curCamIdx);
#if EVAL_PERFORMANCE
        getFrameEvaluator.tic();
#endif
        bool success = multiCameraHandler.getFrame(curFrame);
#if EVAL_PERFORMANCE
        getFrameEvaluator.tac();
#endif

        if (!success)
            curFrame = cv::Mat::zeros(MultiUsbCamera::Config::SCREEN_RESOLUTION, CV_8UC3);
        else
            cv::resize(curFrame, showFrame, MultiUsbCamera::Config::SCREEN_RESOLUTION);

        cv::imshow("video", showFrame);

        char k = cv::waitKey(1);
        if (k == 'q')
            break;

#if EVAL_PERFORMANCE
        loopEvaluator.tac();
#endif
    }
}

static void camCmdCb(const robot_msgs::CameraCmdConstPtr &msg)
{
    curCamIdx = msg->camera_id % numCam;
}