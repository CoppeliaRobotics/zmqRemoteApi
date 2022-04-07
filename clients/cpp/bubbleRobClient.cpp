#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include "RemoteAPIClient.h"
using namespace std::chrono_literals;

int main(int argc,char* argv[])
{
    int portNb=0;
    int leftMotorHandle;
    int rightMotorHandle;
    int sensorHandle;

    if (argc>=5)
    {
        portNb=atoi(argv[1]);
        leftMotorHandle=atoi(argv[2]);
        rightMotorHandle=atoi(argv[3]);
        sensorHandle=atoi(argv[4]);
    }
    else
    {
        printf("Indicate following arguments: 'portNumber leftMotorHandle rightMotorHandle sensorHandle'!\n");
        std::this_thread::sleep_for(5000ms);
        return 0;
    }

    RemoteAPIClient client;
    auto sim = client.getObject().sim();

    int driveBackStartTime=-99000;
    float motorSpeeds[2];

    while (true)
    {
        auto [res, dist, vect, h, n]=sim.readProximitySensor(sensorHandle);
        float simulationTime=sim.getSimulationTime();
        if (simulationTime-driveBackStartTime<3.0f)
        { // driving backwards while slightly turning:
            motorSpeeds[0]=-7.0f*0.5f;
            motorSpeeds[1]=-7.0f*0.25f;
        }
        else
        { // going forward:
            motorSpeeds[0]=7.0f;
            motorSpeeds[1]=7.0f;
            if (res>0)
                driveBackStartTime=simulationTime; // We detected something, and start the backward mode
        }
        sim.setJointTargetVelocity(leftMotorHandle,motorSpeeds[0]);
        sim.setJointTargetVelocity(rightMotorHandle,motorSpeeds[1]);
        std::this_thread::sleep_for(5ms);
    }
    return(0);
}

