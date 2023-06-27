#include <ros/ros.h>

#include <UsbCamera.hpp>
#include <MultiUsbCamera.hpp>
#include <robot_msgs/CameraCmd.h>

const uint8_t NUM_CAM = 3;
const cv::Size SCREEN_RESOLUTION(800, 480);

static int curCamIdx = 0;
static void camCmdCb(const robot_msgs::CameraCmdConstPtr &msg)
{
    curCamIdx = msg->camera_id % NUM_CAM;
}

int main(int ac, char **av)
{
    ros::init(ac, av, "multi_camera_usb_node");
    ros::Time::init();

    ros::NodeHandle nh("~");
    ros::Subscriber camCmdSub = nh.subscribe<robot_msgs::CameraCmd>("/camera_cmd", 10, camCmdCb);

    // -- initialize and check if camera has started correctly --
    MultiUsbCamera::MultiUsbCamera<NUM_CAM> multiCameraHandler(false);
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
        multiCameraHandler.getFrame(curFrame);
        cv::rotate(curFrame, curFrame, cv::ROTATE_180);

        cv::resize(curFrame, showFrame, SCREEN_RESOLUTION);
        cv::imshow("video", showFrame);

        char k = cv::waitKey(1);
        if (k == 'q')
            break;
    }
}