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

int main()
{
    RemoteAPIClient client;
    auto sim = client.getObject().sim();

    auto visionSensorHandle = sim.getObject("/VisionSensor");
    auto passiveVisionSensorHandle = sim.getObject("/PassiveVisionSensor");

    sim.setStepping(true);
    sim.startSimulation();

    auto startTime = sim.getSimulationTime();
    while(sim.getSimulationTime() - startTime < 5)
    {
        auto [img, res] = sim.getVisionSensorImg(visionSensorHandle);
        sim.setVisionSensorImg(passiveVisionSensorHandle, img);
        sim.step();
    }
    sim.stopSimulation();

    return 0;
}

