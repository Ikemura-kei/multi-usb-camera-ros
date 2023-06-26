#include <ros/ros.h>

#include <UsbCamera.hpp>
#include <MultiUsbCamera.hpp>

const uint8_t NUM_CAM = 3;

int main(int ac, char **av)
{
    ros::init(ac, av, "multi_camera_usb_node");
    ros::Time::init();

    ros::NodeHandle nh("~");

    MultiUsbCamera::MultiUsbCamera<NUM_CAM> multiCameraHandler(true); // we do skip the webcam, since the test was conducted on my laptop
    if (!multiCameraHandler.initialized())
    {
        ROS_FATAL_STREAM("Camera initialization failed!");
        return 1;
    }

    static ros::Rate rate(1000);
    static int curCamIdx = 0;
    static cv::Mat curFrame;
    while (ros::ok())
    {
        rate.sleep();

        multiCameraHandler.setCameraPointer(curCamIdx);
        multiCameraHandler.getFrame(curFrame);

        cv::imshow("video", curFrame);

        char k = cv::waitKey(1);
        if (k == 'q')
            break;
        else if (k == 'c')
            curCamIdx = (curCamIdx + 1) % NUM_CAM;
    }
}