% Make sure to have the add-on "ZMQ remote API"
% running in CoppeliaSim
%

fprintf('Program started\n')

client = RemoteAPIClient();
sim = client.require('sim');

% register a callback function (make sure it is in an own file):
client.addCallback(@myCallback, 'myCallback');

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
    s = sprintf('Simulation time: %.2f [s] (simulation running asynchronously to client, i.e. non-stepping)', t);
    fprintf('%s\n', s);
    sim.addLog(sim.verbosity_scriptinfos, s);
    % sim.testCB(21,'myCallback@func',42);
end
sim.stopSimulation();
% If you need to make sure we really stopped:
while sim.getSimulationState() ~= sim.simulation_stopped
    pause(0.1);
end

% Run a simulation in stepping mode:
sim.setStepping(true);
sim.startSimulation();
while 1
    t = sim.getSimulationTime();
    if t >= 3; break; end
    s = sprintf('Simulation time: %.2f [s] (simulation running synchronously to client, i.e. stepping)', t);
    fprintf('%s\n', s);
    sim.addLog(sim.verbosity_scriptinfos, s);
    sim.step();  % triggers next simulation step
end
sim.stopSimulation();

% Remove the dummies created earlier:
for i=1:n
    sim.removeObject(handles(i));
end

fprintf('Program ended\n');

% test callback on CoppeliaSim side:
% int ret = sim.testCB(int a, func cb, int b)
% function sim.testCB(a, cb, b)
%     for i = 1, 99, 1 do
%         cb(a, b)
%     end
%     return cb(a, b)
% end
