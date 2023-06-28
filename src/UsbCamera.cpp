/**
 * @file UsbCamera.cpp
 * @author IKEMURA, Kei (ikemurakei2001@gmial.com)
 * @brief The definition of the single usb camera handler class
 * @version 0.1
 * @date 2023-06-27
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <UsbCamera.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <sensor_msgs/Image.h>
#include <regex>
#include <cv_bridge/cv_bridge.h>
#include <chrono>

namespace MultiUsbCamera
{

    UsbCamera::UsbCamera() : IMG_WIDTH(640), IMG_HEIGHT(480)
    {
        // default constructor, do not set any config
    }

    UsbCamera::UsbCamera(const UsbCamera &other) : IMG_HEIGHT(other.IMG_HEIGHT), IMG_WIDTH(other.IMG_WIDTH)
    {
        (*this) = other;
    }

    UsbCamera &UsbCamera::operator=(const UsbCamera &other)
    {
        this->isInitialized = other.isInitialized;
        this->cap = other.cap;
        this->config = other.config;
        this->header = other.header;
        this->imgPub = other.imgPub;

        return *this;
    }

    UsbCamera::UsbCamera(const int IMG_WIDTH, const int IMG_HEIGHT, CameraConfig config, ros::NodeHandle &nh) : IMG_HEIGHT(IMG_HEIGHT), IMG_WIDTH(IMG_WIDTH), config(config)
    {
        // -- construct the gstreamer configuration stream --
        std::string gstreamerStr = "v4l2src device=DEVICE_PLACEHOLDER ! image/jpeg, width=IMAGE_WIDTH_PLACEHOLDER, height=IMAGE_HEIGHT_PLACEHOLDER, framerate=30/1 ! jpegdec ! videoconvert ! appsink";
        std::string dev = "/dev/video" + this->config.deviceId;
        gstreamerStr = std::regex_replace(gstreamerStr, std::regex("DEVICE_PLACEHOLDER"), dev);
        gstreamerStr = std::regex_replace(gstreamerStr, std::regex("IMAGE_WIDTH_PLACEHOLDER"), std::to_string(IMG_WIDTH));
        gstreamerStr = std::regex_replace(gstreamerStr, std::regex("IMAGE_HEIGHT_PLACEHOLDER"), std::to_string(IMG_HEIGHT));
        std::cout << gstreamerStr << std::endl;
        this->cap = cv::VideoCapture(gstreamerStr, cv::CAP_GSTREAMER);

        int deviceIndex = std::stoi(this->config.deviceId);
        // this->cap = cv::VideoCapture(deviceIndex);

        if (!cap.isOpened())
        {
            this->isInitialized = false;
            return;
        }

        // -- test read --
        cv::Mat testFrame;
        cap >> testFrame;

        if (testFrame.empty())
        {
            this->isInitialized = false;
            return;
        }

        this->isInitialized = true;
        this->header.frame_id = config.frameId;
        this->header.seq = 0;

        // published topic has a name of "/cameras/<camera_name>"
        this->imgPub = nh.advertise<sensor_msgs::Image>(std::string("/cameras/") + this->config.cameraName, 10);
    }

    bool UsbCamera::getFrameAtOnce(cv::Mat &out)
    {
        cap >> out;
        bool imValid = !out.empty() && cap.isOpened();

        if (imValid)
        {
            if (config.cvRotateFlag != -1)
                cv::rotate(out, out, config.cvRotateFlag);

            if (this->isActivated)
            {
                // -- publish before any post processings --
                out.copyTo(frame);
                publish(); // internally checks if publishing is needed, so don't need to check outside

                // -- post processings --
                cv::resize(out, out, config.imageResizedSize);
                cv::putText(out, this->config.cameraName, cv::Point(20, 20), cv::FONT_ITALIC, 1.75, cv::Scalar(255, 255, 255), 3);
            }
        }

        return imValid;
    }

    bool UsbCamera::getFrame(cv::Mat &out)
    {
        if (frame.empty() || !cap.isOpened())
            return false;

        frame.copyTo(out);

        // -- post processings --
        cv::resize(out, out, config.imageResizedSize);
        cv::putText(out, this->config.cameraName, cv::Point(20, 20), cv::FONT_ITALIC, 1.75, cv::Scalar(255, 255, 255), 3);

        return true;
    }

    bool UsbCamera::initialized()
    {
        return this->isInitialized;
    }

    void UsbCamera::cvImgToImageMsg(cv::Mat img, sensor_msgs::Image &msg)
    {
        header.seq += 1;
        header.stamp = ros::Time::now();

        cv_bridge::CvImage bridge(header, sensor_msgs::image_encodings::BGR8, img);
        bridge.toImageMsg(msg);
    }

    void UsbCamera::publish()
    {
        if (config.publishImage)
        {
            sensor_msgs::Image msg;
            cvImgToImageMsg(frame, msg);
            this->imgPub.publish(msg);
        }
    }

    bool UsbCamera::updateFrame()
    {
        cap >> frame;
        bool imValid = !frame.empty() && cap.isOpened();

        if (imValid)
        {
            if (config.cvRotateFlag != -1)
                cv::rotate(frame, frame, config.cvRotateFlag);

            publish();
        }

        return imValid;
    }

    void UsbCamera::setActivation(bool activation)
    {
        this->isActivated = activation;
    }
}