% Make sure to have the add-on "ZMQ remote API"
% running in CoppeliaSim
%

fprintf('Program started\n')

client = RemoteAPIClient();
sim = client.getObject('sim');

% When simulation is not running, ZMQ message handling could be a bit
% slow, since the idle loop runs at 8 Hz by default. So let's make
% sure that the idle loop runs at full speed for this program:
defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps);
sim.setInt32Param(sim.intparam_idle_fps, 0);

% Create a few dummies and set their positions:
n = 50;
handles = zeros(n, 1);
for i=1:n
    handles(i) = sim.createDummy(0.01, zeros(12, 1));
    sim.setObjectPosition(handles(i), -1, [0.01 * i, 0.01 * i, 0.01 * i]);
end

% Run a simulation in asynchronous mode:
sim.startSimulation();

while 1
    t = sim.getSimulationTime();
    if t >= 3; break; end
    s = sprintf('Simulation time: %.2f [s] (simulation running asynchronously to client, i.e. non-stepped)', t);
    fprintf('%s\n', s);
    sim.addLog(sim.verbosity_scriptinfos, s);
end
sim.stopSimulation();
% If you need to make sure we really stopped:
while sim.getSimulationState() ~= sim.simulation_stopped
    pause(0.1);
end

% Run a simulation in stepping mode:
client.setStepping(true);
sim.startSimulation();
while 1
    t = sim.getSimulationTime();
    if t >= 3; break; end
    s = sprintf('Simulation time: %.2f [s] (simulation running synchronously to client, i.e. stepped)', t);
    fprintf('%s\n', s);
    sim.addLog(sim.verbosity_scriptinfos, s)
    client.step();  % triggers next simulation step
end
sim.stopSimulation();

% Remove the dummies created earlier:
for i=1:n
    sim.removeObject(handles(i));
end

% Restore the original idle loop frequency:
sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps);

fprintf('Program ended\n');
