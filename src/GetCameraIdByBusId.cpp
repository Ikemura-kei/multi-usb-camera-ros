#include <iostream>
#include <vector>
#include <UsbCamera.hpp>
/**/
namespace MultiUsbCamera
{
    static std::string exec(const char *cmd)
    {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
        if (!pipe)
        {
            throw std::runtime_error("popen() failed!");
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
        {
            result += buffer.data();
        }
        return result;
    }

    bool getCameraIdByBusId(std::vector<MultiUsbCamera::CameraConfig> &cameraConfigs, std::vector<MultiUsbCamera::UsbCamera> &cameras, bool updateCamera)
    {
        // -- run this command to get camera information --
        std::string s = exec("v4l2-ctl --list-devices");
        /*
         * An example of the output from the command above:
         * USB HD Camera: USB HD Camera (usb-0000:00:14.0-3.4):
         *         /dev/video0
         *         /dev/video1
         *
         * USB HD Camera: USB HD Camera (usb-0000:00:14.0-4.3):
         *         /dev/video2
         *         /dev/video3
         *
         * USB HD Camera: USB HD Camera (usb-0000:00:14.0-4.4):
         *         /dev/video4
         *         /dev/video5
         */
        std::string endOfLineToken = "\n";
        size_t posOfEoL = 0;
        std::string line;

        // -- look for the next end-of-line token, until we can't find any --
        while ((posOfEoL = s.find(endOfLineToken)) != std::string::npos)
        {
            // -- examine the current line --
            line = s.substr(0, posOfEoL);

            int camIdx = 0;
            for (auto it = cameraConfigs.begin(); it != cameraConfigs.end(); it++, camIdx++)
            {
                if (it->deviceId != "-1" || it->busId == "extra") // we mark uninitialized ids to be -1, not being equal to -1 means the camera has already been associated to a propare camera index
                    continue;

                // -- see if this line contains the desired bus id for this camera --
                if (line.find(it->busId) != std::string::npos)
                {
                    // -- this line contains the desired bus id, now parse the corresponding device id --

                    /* For example, the output should look something like this
                     * An example of the output from the command above:
                     * USB HD Camera: USB HD Camera (usb-0000:00:14.0-3.4):
                     *         /dev/video0
                     *         /dev/video1
                     */
                    size_t nextPos = s.find(endOfLineToken, posOfEoL + 1);                 // the next EoL token, that is the "     /dev/video0" line
                    std::string nextLine = s.substr(posOfEoL + 1, nextPos - posOfEoL - 1); // get the next line
                    it->deviceId = nextLine.back();                                        // this should retrieve the last character, that is the camera index, in the example above, is "0"
                    // -- however, there are cases that the camera index is of two digits, like "10" or more, here we need to handle this --
                    if (nextLine[nextLine.size() - 2] != 'o') // if the camera index is indeed 1 digit, the second last character should be 'o', since the line is something like '/dev/video0'
                    {
                        char cp = nextLine.back();
                        std::string cameraIndex;

                        std::stringstream stringStream;
                        stringStream << nextLine[nextLine.size() - 2] << cp;

                        stringStream >> cameraIndex;
                        it->deviceId = cameraIndex;
                    }
                    if (updateCamera)
                    {
                        cameras[camIdx].updateCameraId(it->deviceId);
                        ROS_INFO_STREAM("--> Device id updated to " << it->deviceId);
                    }
                    break;
                }
            }

            // -- get rid of the currently examined line --
            s.erase(0, posOfEoL + endOfLineToken.length());
        }

        // -- print configurations for sanity checks --
        bool allCameraOk = true;
        for (auto it = cameraConfigs.begin(); it != cameraConfigs.end(); it++)
        {
            if (it->deviceId == "-1" && it->busId != "extra")
            {
                allCameraOk = false;
                ROS_WARN_STREAM("--> Camera for {" << it->busId << "} was not found!!!");
            }
            else
            {
                ROS_INFO_STREAM("--> Camera for {" << it->busId << "} was found successfully :)");
            }

            // std::cout << std::endl
            //           << *it << std::endl;
        }

        return allCameraOk;
    }
}