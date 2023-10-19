#include "RemoteAPIClient.h"
#include <iostream>
#include <iomanip>

json func(const json& input)
{
    // std::cout << pretty_print(input) << "\n\n";
    return 21;
}
std::function<json(const json &)> myCallback = func;

int main()
{
    RemoteAPIClient client;
    auto sim = client.getObject().sim();
    client.registerCallback("myCallback", myCallback);
    sim.setStepping(true);
    sim.startSimulation();
    double simTime = 0.0;
    while((simTime = sim.getSimulationTime()) < 3)
    {
        std::cout << "Simulation time: " << std::setprecision(3) << simTime << " [s]" << std::endl;
        // auto retVal = sim.testCB(21, "myCallback@func", 42);
        sim.step();
    }
    sim.stopSimulation();
    return 0;
}

/* test callback on CoppeliaSim side:
int ret = sim.testCB(int a, func cb, int b)
function sim.testCB(a, cb, b)
    for i = 1, 99, 1 do
        cb(a, b)
    end
    return cb(a, b)
end
*/
