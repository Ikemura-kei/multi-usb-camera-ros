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
        initFrameSubstitution();
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

        initFrameSubstitution();

        return *this;
    }

    bool UsbCamera::initCamera()
    {
        if (this->config.deviceId == "-1")
        {
            return false; // the associated device id was not found yet, no way we can initialize camera
        }

        // -- construct the gstreamer configuration stream --
        std::string gstreamerStr = "v4l2src device=DEVICE_PLACEHOLDER ! image/jpeg, width=IMAGE_WIDTH_PLACEHOLDER, height=IMAGE_HEIGHT_PLACEHOLDER, framerate=30/1 ! jpegdec ! videoconvert ! appsink";
        std::string dev = "/dev/video" + this->config.deviceId;
        gstreamerStr = std::regex_replace(gstreamerStr, std::regex("DEVICE_PLACEHOLDER"), dev);
        gstreamerStr = std::regex_replace(gstreamerStr, std::regex("IMAGE_WIDTH_PLACEHOLDER"), std::to_string(IMG_WIDTH));
        gstreamerStr = std::regex_replace(gstreamerStr, std::regex("IMAGE_HEIGHT_PLACEHOLDER"), std::to_string(IMG_HEIGHT));
        std::cout << gstreamerStr << std::endl;
        this->cap = cv::VideoCapture(gstreamerStr, cv::CAP_GSTREAMER);

        if (!cap.isOpened())
        {
            return false;
        }

        // -- test read --
        cv::Mat testFrame;
        cap >> testFrame;

        if (testFrame.empty())
        {
            return false;
        }

        return true;
    }

    UsbCamera::UsbCamera(const int IMG_WIDTH, const int IMG_HEIGHT, CameraConfig &config, ros::NodeHandle &nh) : IMG_HEIGHT(IMG_HEIGHT), IMG_WIDTH(IMG_WIDTH), config(config)
    {
        this->lastReconnectionTime = ros::Time::now();

        // int deviceIndex = std::stoi(this->config.deviceId);
        // this->cap = cv::VideoCapture(deviceIndex);

        this->isInitialized = initCamera();
        this->header.frame_id = config.frameId;
        this->header.seq = 0;

        // published topic has a name of "/cameras/<camera_name>"
        this->imgPub = nh.advertise<sensor_msgs::Image>(std::string("/cameras/") + this->config.cameraName, 10);

        initFrameSubstitution();
    }

    void UsbCamera::initFrameSubstitution()
    {
        this->noFrameSubstitution = cv::Mat::zeros(cv::Size(this->IMG_WIDTH, this->IMG_HEIGHT), CV_8UC1);
        cv::putText(this->noFrameSubstitution, "No Frame Retreived!!! Camera Disconnected.", cv::Point(((int)(this->IMG_WIDTH / 2), (int)(this->IMG_HEIGHT / 2))), cv::FONT_ITALIC, 1.5, 255, 2);
    }

    void UsbCamera::updateCameraId(std::string newId)
    {
        this->config.deviceId = newId;
    }

    bool UsbCamera::getFrameAtOnce(cv::Mat &out)
    {
        bool imValid = false;
        if (this->isInitialized)
        {
            cap >> out;
            imValid = !out.empty() && cap.isOpened();

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
        }
        else
        {
            this->isInitialized = false;
            if ((ros::Time::now() - this->lastReconnectionTime).toSec() >= this->RECONNECTION_PERIOD)
            {
                this->lastReconnectionTime = ros::Time::now();

                this->isInitialized = initCamera();

                if (this->isInitialized)
                {
                    ROS_INFO_STREAM("--> Camera reconnected!!! Id is: {" << config.cameraName << "}");
                }
                else
                {
                    ROS_WARN_STREAM("--> Reconnection failed!!! Id is {" << config.cameraName << "}");
                }
            }
        }

        return imValid;
    }

    bool UsbCamera::getFrame(cv::Mat &out)
    {
        if (frame.empty() || !cap.isOpened() || !this->isInitialized)
        {
            this->noFrameSubstitution.copyTo(out);
            cv::resize(out, out, config.imageResizedSize);
            cv::putText(out, this->config.cameraName, cv::Point(50, 50), cv::FONT_ITALIC, 1.25, cv::Scalar(255, 255, 255), 3);
            return false;
        }

        this->frameMutex.lock();
        frame.copyTo(out);
        this->frameMutex.unlock();

        // -- post processings --
        cv::resize(out, out, config.imageResizedSize);
        cv::putText(out, this->config.cameraName, cv::Point(50, 50), cv::FONT_ITALIC, 1.25, cv::Scalar(255, 255, 255), 3);

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
        cv::Mat tmp;
        cap >> tmp;
        bool imValid = !tmp.empty() && cap.isOpened() && this->isInitialized;

        // ROS_INFO_STREAM("--> Image valid is " << (imValid ? "YES" : "NO"));

        if (imValid)
        {
            if (config.cvRotateFlag != -1)
                cv::rotate(tmp, tmp, config.cvRotateFlag);

            this->frameMutex.lock();
            tmp.copyTo(this->frame);
            this->frameMutex.unlock();

            publish();
        }
        else
        {
            this->isInitialized = false;
            if ((ros::Time::now() - this->lastReconnectionTime).toSec() >= this->RECONNECTION_PERIOD)
            {
                this->lastReconnectionTime = ros::Time::now();

                this->isInitialized = initCamera();

                if (this->isInitialized)
                {
                    ROS_INFO_STREAM("--> Camera reconnected!!! Id is: {" << config.cameraName << "}");
                }
                else
                {
                    ROS_WARN_STREAM("--> Reconnection failed!!! Id is {" << config.cameraName << "}");
                }
            }
        }

        return imValid;
    }

    void UsbCamera::setActivation(bool activation)
    {
        this->isActivated = activation;
    }
}