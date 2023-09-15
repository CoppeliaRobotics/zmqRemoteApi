% Make sure to have the add-on "ZMQ remote API" running in
% CoppeliaSim and have following scene loaded:
%
% scenes/messaging/synchronousImageTransmissionViaRemoteApi.ttt
%
% Do not launch simulation, but run this script
%

fprintf('Program started\n');

client = RemoteAPIClient();
sim = client.require('sim');

visionSensorHandle = sim.getObject('/VisionSensor');
passiveVisionSensorHandle = sim.getObject('/PassiveVisionSensor');

sim.setStepping(true);
sim.startSimulation();

startTime = sim.getSimulationTime();
t = startTime;
while t - startTime < 5
    [img, resX, resY] = sim.getVisionSensorCharImage(visionSensorHandle);
    % display image in MATLAB:
    %imshow(flip(permute(reshape(img, 3, resY, resX), [3 2 1]), 1));
    sim.setVisionSensorCharImage(passiveVisionSensorHandle, img);
    sim.step();
    t = sim.getSimulationTime();
end
sim.stopSimulation();

fprintf('Program ended\n');
