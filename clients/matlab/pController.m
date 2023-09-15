% Make sure to have CoppeliaSim running, with followig scene loaded:
%
% scenes/messaging/pControllerViaRemoteApi.ttt
%
% Do not launch simulation, but run this script

fprintf('Program started\n');

maxForce = 100;
PID_P = 0.1;
dynStepSize = 0.005;
velUpperLimit = deg2rad(360);

client = RemoteAPIClient();
sim = client.require('sim');

jointHandle = sim.getObject('/Cuboid[0]/joint');
jointAngle = sim.getJointPosition(jointHandle);
sim.setJointTargetVelocity(jointHandle, deg2rad(360));

% enable the stepping mode on the client:
sim.setStepping(true);

sim.startSimulation();

angles = [
    45
    90
    -89 % no -90, to avoid passing below
    0
];

for i=1:numel(angles)
    targetAngle = deg2rad(angles(i));
    while abs(jointAngle - targetAngle) > deg2rad(0.1)
        % Calculate the velocity needed to reach the position
        % in one dynamic time step:
        errorValue = targetAngle - jointAngle;
        sinAngle = sin(errorValue);
        cosAngle = cos(errorValue);
        errorValue = atan2(sinAngle, cosAngle);
        ctrl = errorValue * PID_P;
        velocity = max(-velUpperLimit, min(velUpperLimit, ctrl / dynStepSize));
        % .
        sim.setJointTargetVelocity(jointHandle, velocity);
        sim.setJointMaxForce(jointHandle, maxForce);
        sim.step();
        jointAngle = sim.getJointPosition(jointHandle);
    end
end

sim.stopSimulation();

fprintf('Program ended\n');
