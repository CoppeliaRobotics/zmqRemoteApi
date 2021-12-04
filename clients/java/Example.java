import org.json.JSONArray;

public class Example
{
    public static void main(String[] _args)
    {
        RemoteAPIClient client = new RemoteAPIClient();
        JSONArray ret = client.call("sim.getObject", "/Floor");
        for(int i = 0; i < ret.length(); i++)
            System.out.println("ret[" + i + "]: " + ret.get(i));
    }
}
