import java.util.Arrays;

import com.coppeliarobotics.remoteapi.zmq.*;

public class Example
{
    public static Object[] myFunc(Object[] items)
    {
        // System.out.println(Arrays.toString(items));

        Object[] retVals = new Object[1];
        retVals[0] = 21;
        return retVals;
    }
    
    public static void main(String[] _args) throws java.io.IOException, co.nstant.in.cbor.CborException
    {
        var client = new RemoteAPIClient();
        var sim = client.getObject().sim();
        
        client.registerCallback("myFunc", Example::myFunc);

        sim.setStepping(true);
        sim.startSimulation();

        double simTime = 0.0;
        while((simTime = sim.getSimulationTime()) < 3)
        {
            System.out.printf("Simulation time: %.2f [s]%n", simTime);
            // Long retVal = sim.testCB(21,"myFunc@func",42);
            sim.step();
        }
        sim.stopSimulation();
    }
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
