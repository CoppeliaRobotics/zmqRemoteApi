package com.coppeliarobotics.remoteapi.zmq;

import java.util.List;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;

import co.nstant.in.cbor.*;
import co.nstant.in.cbor.model.*;
import org.zeromq.*;

public class RemoteAPIClient
{
    public RemoteAPIClient()
    {
        this("localhost", 23000);
    }

    public RemoteAPIClient(String host, int port)
    {
        this(host, port, -1);
    }

    public RemoteAPIClient(String host, int port, int verbose)
    {
        this.verbose = verbose;
        if(this.verbose == -1)
        {
            String verboseStr = System.getenv("VERBOSE");
            if(verboseStr != null)
                this.verbose = Integer.parseInt(verboseStr);
            else
                this.verbose = 0;
        }
        this.context = new ZContext(1);
        this.socket = context.createSocket(SocketType.REQ);
        this.socket.connect(String.format("tcp://%s:%d", host, port));
    }

    protected void send(DataItem req) throws CborException
    {
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        new CborEncoder(baos).encode(req);
        byte[] encodedBytes = baos.toByteArray();
        this.socket.send(encodedBytes);
    }

    protected DataItem recv() throws CborException
    {
        ByteArrayInputStream bais = new ByteArrayInputStream(this.socket.recv(0));
        List<DataItem> dataItems = new CborDecoder(bais).decode();
        return dataItems.get(0);
    }

    public List<DataItem> call(String func, List<DataItem> args) throws CborException
    {
        UnicodeString k_func = new UnicodeString("func"),
                      k_args = new UnicodeString("args"),
                      k_success = new UnicodeString("success"),
                      k_error = new UnicodeString("error"),
                      k_ret = new UnicodeString("ret");
        Map req = new Map();
        req.put(k_func, new UnicodeString("sim.getObject"));
        Array v_args = new Array();
        for(int i = 0; i < args.size(); i++)
            v_args.add(args.get(i));
        req.put(k_args, v_args);
        if(this.verbose > 0)
            System.out.println("Sending: " + req.toString());
        this.send(req);
        Map rep = (Map)this.recv();
        if(this.verbose > 0)
            System.out.println("Received: " + rep.toString());
        if(SimpleValue.FALSE == rep.get(k_success))
        {
            DataItem error = rep.get(k_error);
            throw new RuntimeException(error != null ? error.toString() : "Unknown error");
        }
        DataItem ret = rep.get(k_ret);
        return ((Array)ret).getDataItems();
    }

    private int verbose = -1;
    ZContext context;
    ZMQ.Socket socket;
}
