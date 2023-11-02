package com.coppeliarobotics.remoteapi.zmq;

public class RemoteAPIObject
{
    protected final RemoteAPIClient client;

    public RemoteAPIObject(RemoteAPIClient client)
    {
        this.client = client;
    }
}
