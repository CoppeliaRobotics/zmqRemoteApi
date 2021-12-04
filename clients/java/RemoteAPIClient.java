import org.json.*;
import org.zeromq.*;

/*
 * you need the jars of JeroMQ and JSON-java in the classpath
 */

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
        this.socket.connect("tcp://localhost:23000");
    }

    protected void send(JSONObject req)
    {
        this.socket.send(req.toString());
    }

    protected JSONObject recv()
    {
        return new JSONObject(this.socket.recvStr(0));
    }

    public JSONArray call(String func, Object... _args)
    {
        JSONObject req = new JSONObject();
        req.put("func", "sim.getObject");
        JSONArray args = new JSONArray();
        for(int i = 0; i < _args.length; i++)
            args.put(_args[i]);
        req.put("args", args);

        if(this.verbose > 0)
            System.out.println("Sending: " + req.toString());

        this.send(req);

        JSONObject rep = this.recv();

        if(this.verbose > 0)
            System.out.println("Received: " + rep.toString());

        if(rep.getBoolean("success") == false)
        {
            if(rep.has("error"))
                throw new RuntimeException(rep.getString("error"));
            else
                throw new RuntimeException("Unknown error");
        }

        return (JSONArray)rep.get("ret");
    }

    private int verbose = -1;
    ZContext context;
    ZMQ.Socket socket;
}
