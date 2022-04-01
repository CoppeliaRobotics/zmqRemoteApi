package com.coppeliarobotics.remoteapi.zmq;

import java.util.List;
import java.util.Arrays;
import co.nstant.in.cbor.*;
import co.nstant.in.cbor.model.*;

public class Example
{
    public static void main(String[] _args) throws CborException
    {
        RemoteAPIClient client = new RemoteAPIClient();
        System.out.println("calling sim.getObject('/Floor')...");
        List<DataItem> ret = client.call(
            "sim.getObject",
            Arrays.asList(
                new UnicodeString("/Floor")
            )
        );
        for(int i = 0; i < ret.size(); i++)
            System.out.println("ret[" + i + "]: " + ret.get(i));
    }
}
