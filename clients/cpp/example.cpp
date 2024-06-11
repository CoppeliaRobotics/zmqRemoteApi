#include "RemoteAPIClient.h"
#include <iostream>
#include <iomanip>

json myFunc(const json& input)
{
    std::cout << pretty_print(input) << "\n\n";
    return 21;
}
std::function<json(const json &)> myCallback = myFunc;

int main()
{
    try 
    {
        RemoteAPIClient client;
        auto sim = client.getObject().sim();
        client.registerCallback("myCallback", myFunc);
        sim.setStepping(true);
        sim.startSimulation();
        double simTime = 0.0;
        while((simTime = sim.getSimulationTime()) < 3)
        {
            std::cout << "Simulation time: " << std::setprecision(3) << simTime << " [s]" << std::endl;
            // auto retVal = sim.testCB(21, "myCallback@func", 42); // sim.testCB is calling back above "myFunc"
            sim.step();
        }
        // e.g. calling a script object function (make sure the script is running!):
        /*
        int script = sim.getObject("/path/to/scriptObject");
        auto args = json::array();
        args.push_back("Hello");
        args.push_back("Paul");
        args.push_back(21);
        auto reply = sim.callScriptFunction("functionName", script, args);
        */
        sim.executeScriptString("print('Hello there')", sim.getScript(sim.scripttype_sandbox));
        sim.stopSimulation();
    } 
    catch (const std::runtime_error& e)
    {
        std::cerr << "Caught a runtime error: " << e.what() << std::endl;
    }
    
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
