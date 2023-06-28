#include <thread>
#include <ros/ros.h>
#include <Config.hpp>
#include <UsbCamera.hpp>
#include <GetUserConfig.hpp>
#include <chrono>
#include <robot_msgs/CameraCmd.h>

// -- helper functions --
static void camCmdCb(const robot_msgs::CameraCmdConstPtr &msg);
void cameraThreadFunc(MultiUsbCamera::UsbCamera *usbCamera);

// -- globals --
static int numCam = 0;
static int curCamIdx = 0;

int main(int ac, char **av)
{
    ros::init(ac, av, "multi_threaded_multi_usb_camera_node");
    ros::NodeHandle nh;
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

    // -- initialize cameras --
    std::vector<MultiUsbCamera::UsbCamera> cameras;
    for (auto it = cameraConfigs.begin(); it != cameraConfigs.end(); it++)
        cameras.push_back(MultiUsbCamera::UsbCamera(MultiUsbCamera::Config::SET_CAMERA_IMG_WIDTH, MultiUsbCamera::Config::SET_CAMERA_IMG_HEIGHT, *it, nh));

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
        bool success = cameras[curCamIdx].getFrame(curFrame);
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
    auto lastTime = std::chrono::high_resolution_clock::now();
    while (ros::ok())
    {
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - lastTime);
        lastTime = std::chrono::high_resolution_clock::now();

        usbCamera->updateFrame();

        std::cout << usbCamera->getConfig().cameraName << ": " << duration.count() / 1000000.0 << std::endl;
    }
}

static void camCmdCb(const robot_msgs::CameraCmdConstPtr &msg)
{
    curCamIdx = msg->camera_id % numCam;
}