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

    auto visionSensorHandle = client.call("sim.getObject", {"/VisionSensor"})[0];
    auto passiveVisionSensorHandle = client.call("sim.getObject", {"/PassiveVisionSensor"})[0];

    client.setStepping(true);
    client.call("sim.startSimulation");

    auto startTime = client.call("sim.getSimulationTime")[0].as<double>();
    while(client.call("sim.getSimulationTime")[0].as<double>() - startTime < 5)
    {
        auto ret = client.call("sim.getVisionSensorCharImage", {visionSensorHandle});
        auto img = ret[0];
        auto resX = ret[1];
        auto resY = ret[2];
        client.call("sim.setVisionSensorCharImage", {passiveVisionSensorHandle, img});
        client.step();
    }
    client.call("sim.stopSimulation");

    return 0;
}

