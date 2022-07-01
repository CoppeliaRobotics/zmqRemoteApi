import java.util.List;
import java.util.Arrays;

import com.coppeliarobotics.remoteapi.zmq.*;

public class Example
{
    public static void main(String[] _args) throws java.io.IOException, co.nstant.in.cbor.CborException
    {
        var client = new RemoteAPIClient();
        var sim = client.getObject().sim();

        client.setStepping(true);
        sim.startSimulation();

        float simTime = 0.0f;
        while((simTime = sim.getSimulationTime()) < 3)
        {
            System.out.println("Simulation time: " + simTime + " [s]");
            client.step();
        }
        sim.stopSimulation();
    }
}
