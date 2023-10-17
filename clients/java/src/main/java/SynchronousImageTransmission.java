import java.util.List;
import java.util.Arrays;

import com.coppeliarobotics.remoteapi.zmq.*;

public class SynchronousImageTransmission
{
    /*
     * Make sure to have the add-on "ZMQ remote API" running in
     * CoppeliaSim and have following scene loaded:
     *
     * scenes/messaging/synchronousImageTransmissionViaRemoteApi.ttt
     *
     * Do not launch simulation, but run this script
     */

    public static void main(String[] _args) throws java.io.IOException, co.nstant.in.cbor.CborException
    {
        var client = new RemoteAPIClient();
        var sim = client.getObject().sim();

        var visionSensorHandle = sim.getObject("/VisionSensor");
        var passiveVisionSensorHandle = sim.getObject("/PassiveVisionSensor");

        sim.setStepping(true);
        sim.startSimulation();

        double startTime = sim.getSimulationTime();
        while(sim.getSimulationTime() - startTime < 5)
        {
            Object[] r = sim.getVisionSensorImg(visionSensorHandle);
            byte[] imgData = (byte[])r[0];
            List<Long> res = (List<Long>)r[1];
            sim.setVisionSensorImg(passiveVisionSensorHandle, imgData);
            sim.step();
        }
        sim.stopSimulation();
    }
}
