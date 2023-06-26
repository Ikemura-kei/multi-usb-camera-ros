#include <ros/ros.h>

#include <UsbCamera.hpp>
#include <MultiUsbCamera.hpp>

const uint8_t NUM_CAM = 3;

int main(int ac, char **av)
{
    ros::init(ac, av, "multi_camera_usb_node");
    ros::Time::init();

    ros::NodeHandle nh("~");

    MultiUsbCamera::MultiUsbCamera<NUM_CAM> multiCameraHandler(false);
    int initState = multiCameraHandler.initialized();
    if (initState != -1)
    {
        ROS_FATAL_STREAM("Camera initialization failed at " << std::to_string(initState+1));
        return 1;
    }

    cv::namedWindow("video", cv::WINDOW_NORMAL);
    cv::setWindowProperty("video", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    static ros::Rate rate(1000);
    static int curCamIdx = 0;
    static cv::Mat curFrame;
    static cv::Mat showFrame;
    while (ros::ok())
    {
        rate.sleep();

        multiCameraHandler.setCameraPointer(curCamIdx);
        multiCameraHandler.getFrame(curFrame);

        cv::resize(curFrame, showFrame, cv::Size(800, 480));

        cv::imshow("video", showFrame);

        char k = cv::waitKey(1);
        if (k == 'q')
            break;
        else if (k == 'c')
            curCamIdx = (curCamIdx + 1) % NUM_CAM;
    }
}