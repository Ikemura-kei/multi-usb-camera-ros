#include <thread>
#include <ros/ros.h>
#include <Config.hpp>
#include <UsbCamera.hpp>
#include <GetUserConfig.hpp>
#include <chrono>

void cameraThreadFunc(MultiUsbCamera::UsbCamera *usbCamera)
{
    auto lastTime = std::chrono::high_resolution_clock::now();
    while (ros::ok())
    {
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - lastTime);
        lastTime = std::chrono::high_resolution_clock::now();

        usbCamera->updateFrame();

        std::cout << usbCamera->getConfig().cameraName << " " << duration.count() / 1000000.0 << std::endl;
    }
}

int main(int ac, char **av)
{
    ros::init(ac, av, "multi_threaded_multi_usb_camera_node");

    ros::NodeHandle nh;

    // -- get camera configurations --
    std::string pathToYaml = "/home/rm/engineer_2023_ws/config/multi_usb_camera.yaml";
    std::vector<MultiUsbCamera::CameraConfig> cameraConfigs = MultiUsbCamera::getCameraConfigs(pathToYaml);

    MultiUsbCamera::UsbCamera camera1(MultiUsbCamera::Config::SET_CAMERA_IMG_WIDTH, MultiUsbCamera::Config::SET_CAMERA_IMG_HEIGHT, cameraConfigs[0], nh);
    MultiUsbCamera::UsbCamera camera2(MultiUsbCamera::Config::SET_CAMERA_IMG_WIDTH, MultiUsbCamera::Config::SET_CAMERA_IMG_HEIGHT, cameraConfigs[1], nh);
    MultiUsbCamera::UsbCamera camera3(MultiUsbCamera::Config::SET_CAMERA_IMG_WIDTH, MultiUsbCamera::Config::SET_CAMERA_IMG_HEIGHT, cameraConfigs[2], nh);

    std::thread cameraThread1(cameraThreadFunc, &camera1);
    std::thread cameraThread2(cameraThreadFunc, &camera2);
    std::thread cameraThread3(cameraThreadFunc, &camera3);

    // -- create a visualization window that covers the entire screen --
    cv::namedWindow("video", cv::WINDOW_NORMAL);
    cv::setWindowProperty("video", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    // -- static variables --
    static cv::Mat curFrame;
    while (ros::ok())
    {
        camera1.getFrame(curFrame);
        cv::imshow("video", curFrame);
    }

    cameraThread1.join();
    cameraThread2.join();
    cameraThread3.join();
}