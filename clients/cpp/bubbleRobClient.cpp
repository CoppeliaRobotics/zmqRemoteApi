#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include "RemoteAPIClient.h"
using namespace std::chrono_literals;

int main(int argc,char* argv[])
{
    int leftMotorHandle;
    int rightMotorHandle;
    int sensorHandle;

    if (argc>=4)
    {
        leftMotorHandle=atoi(argv[1]);
        rightMotorHandle=atoi(argv[2]);
        sensorHandle=atoi(argv[3]);
    }
    else
    {
        printf("Indicate following arguments: 'leftMotorHandle rightMotorHandle sensorHandle'!\n");
        std::this_thread::sleep_for(5000ms);
        return 0;
    }

    RemoteAPIClient client;
    auto sim = client.getObject().sim();

    float driveBackStartTime=-5.0f;
    float motorSpeeds[2];

    while ( sim.isHandle(sensorHandle) && (sim.getSimulationState()!=0) )
    {
        printf(".");
        auto [res, dist, detectPt, h, n]=sim.readProximitySensor(sensorHandle);
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
    }
    return(0);
}

