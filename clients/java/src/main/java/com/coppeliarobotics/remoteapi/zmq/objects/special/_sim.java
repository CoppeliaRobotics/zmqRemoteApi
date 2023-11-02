package com.coppeliarobotics.remoteapi.zmq.objects.special;

import co.nstant.in.cbor.CborException;

import com.coppeliarobotics.remoteapi.zmq.RemoteAPIClient;
import com.coppeliarobotics.remoteapi.zmq.RemoteAPIObject;

public class _sim extends RemoteAPIObject
{
    public _sim(RemoteAPIClient client)
    {
        super(client);
    }

    public Object[] callScriptFunction(Object... args) throws CborException
    {
        return this.client.call("sim.callScriptFunction", args);
    }
}
