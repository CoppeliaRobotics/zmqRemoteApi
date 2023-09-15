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
    auto sim = client.getObject().sim();

    auto visionSensorHandle = sim.getObject("/VisionSensor");
    auto passiveVisionSensorHandle = sim.getObject("/PassiveVisionSensor");

    sim.setStepping(true);
    sim.startSimulation();

    while(1)
    {
        sim.step();

        auto [img, res] = sim.getVisionSensorImg(visionSensorHandle);

        image = cv::Mat(res[1], res[0], CV_8UC3, img.data());
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
        sim.setVisionSensorImg(passiveVisionSensorHandle, std::vector<uint8_t>(cedge.data, cedge.data + cedge.total() * cedge.elemSize()));

        // Wait for a key stroke; the same function arranges events processing:
        auto key = cv::waitKey(0) & 0xFF;
        if(key == 'q' || key == 27) break;
    }
    sim.stopSimulation();

    return 0;
}

