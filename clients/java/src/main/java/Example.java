import java.util.List;
import java.util.Arrays;

import com.coppeliarobotics.remoteapi.zmq.*;

public class Example
{
    public static void main(String[] _args) throws java.io.IOException, co.nstant.in.cbor.CborException
    {
        var client = new RemoteAPIClient();
        var sim = client.getObject().sim();

        sim.setStepping(true);
        sim.startSimulation();

        double simTime = 0.0f;
        while((simTime = sim.getSimulationTime()) < 3)
        {
            System.out.println("Simulation time: " + simTime + " [s]");
            sim.step();
        }
        sim.stopSimulation();
    }
}
