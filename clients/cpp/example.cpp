#include "RemoteAPIClient.h"
#include <iostream>
#include <iomanip>

int main()
{
    RemoteAPIClient client;
    auto sim = client.getObject().sim();
    sim.setStepping(true);
    sim.startSimulation();
    float simTime = 0.0f;
    while((simTime = sim.getSimulationTime()) < 3)
    {
        std::cout << "Simulation time: " << std::setprecision(3) << simTime << " [s]" << std::endl;
        sim.step();
    }
    sim.stopSimulation();
    return 0;
}
