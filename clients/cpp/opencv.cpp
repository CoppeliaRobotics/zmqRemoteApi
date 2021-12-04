/*
 * Make sure to have the add-on "ZMQ remote API" running in
 * CoppeliaSim and have following scene loaded:
 *
 * scenes/messaging/synchronousImageTransmissionViaRemoteApi.ttt
 *
 * Do not launch simulation, but run this script
 */

#include "RemoteAPIClient.h"

#include <iostream>

#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

json bin(const cv::Mat &mat)
{
    return bin(mat.data, mat.rows * mat.cols * mat.elemSize());
}

int edgeThresh = 12;
cv::Mat image, gray, blurImage, edge, cedge;
const char* window_name = "Press any key to step, q to quit";

// OpenCV event / image processing callback:
static void onTrackbar(int value, void *data)
{
    cv::blur(gray, blurImage, cv::Size(3, 3));
    // Run the edge detector on grayscale
    cv::Canny(blurImage, edge, edgeThresh, edgeThresh * 3, 3);
    cedge = cv::Scalar::all(0);
    image.copyTo(cedge, edge);
    cv::imshow(window_name, cedge);
}

int main()
{
    RemoteAPIClient client;

    auto visionSensorHandle = client.call("sim.getObject", {"/VisionSensor"})[0];
    auto passiveVisionSensorHandle = client.call("sim.getObject", {"/PassiveVisionSensor"})[0];

    client.setStepping(true);
    client.call("sim.startSimulation");

    while(1)
    {
        client.step();

        auto ret = client.call("sim.getVisionSensorCharImage", {visionSensorHandle});
        auto img = ret[0].as<std::vector<uint8_t>>();
        auto resX = ret[1].as<int>();
        auto resY = ret[2].as<int>();

        image = cv::Mat(resY, resX, CV_8UC3, img.data());
        // In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
        // (consistent with the axes of vision sensors, pointing Z outwards, Y up)
        // and color format is RGB triplets, whereas OpenCV uses BGR:
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        cv::flip(image, image, 0);

        // Create grayscale copy of input image:
        cedge.create(image.size(), image.type());
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        // Create a window with some UI controls:
        cv::namedWindow(window_name, 1);
        cv::createTrackbar("Canny threshold default", window_name, &edgeThresh, 100, onTrackbar);
        // Trigger image processing function:
        onTrackbar(0, 0);

        // Write displayed image back to CoppeliaSim:
        cv::cvtColor(cedge, cedge, cv::COLOR_BGR2RGB);
        cv::flip(cedge, cedge, 0);
        client.call("sim.setVisionSensorCharImage", {passiveVisionSensorHandle, bin(cedge)});

        // Wait for a key stroke; the same function arranges events processing:
        auto key = cv::waitKey(0) & 0xFF;
        if(key == 'q' || key == 27) break;
    }
    client.call("sim.stopSimulation");

    return 0;
}

