#include "RemoteAPIClient.h"
#include <iostream>

int main()
{
    RemoteAPIClient client;
    client.setStepping(true);
    client.call("sim.startSimulation", nullptr);
    while (client.call("sim.getSimulationTime")[0]<3.0f)
        client.step();
    client.call("sim.stopSimulation", nullptr);

    return 0;
}
