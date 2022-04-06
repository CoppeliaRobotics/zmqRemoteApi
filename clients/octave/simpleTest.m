arraySize=300;
client = RemoteAPIClient();
sim = client.getObject('sim');

defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps);
sim.setInt32Param(sim.intparam_idle_fps, 0);

st=sim.getSystemTimeInMs(-1);

dummy=sim.createDummy(0.01, zeros(12, 1));
for i=1:50
    a=zeros(arraySize, 1);
    %sim.setObjectPosition(dummy, -1, zeros(arraySize, 1));
end

sim.getSystemTimeInMs(st)
sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps);
