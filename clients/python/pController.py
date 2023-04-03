# Make sure to have CoppeliaSim running, with followig scene loaded:
#
# scenes/messaging/pControllerViaRemoteApi.ttt
#
# Do not launch simulation, but run this script

import math

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

print('Program started')

maxForce = 100

client = RemoteAPIClient()
sim = client.getObject('sim')


def moveToAngle(targetAngle):
    global jointAngle
    while abs(jointAngle - targetAngle) > 0.1 * math.pi / 180:
        vel = computeTargetVelocity(jointAngle, targetAngle)
        sim.setJointTargetVelocity(jointHandle, vel)
        sim.setJointMaxForce(jointHandle, maxForce)
        client.step()
        jointAngle = sim.getJointPosition(jointHandle)


def computeTargetVelocity(jointAngle, targetAngle):
    dynStepSize = 0.005
    velUpperLimit = 360 * math.pi / 180
    PID_P = 0.1
    errorValue = targetAngle - jointAngle
    sinAngle = math.sin(errorValue)
    cosAngle = math.cos(errorValue)
    errorValue = math.atan2(sinAngle, cosAngle)
    ctrl = errorValue * PID_P

    # Calculate the velocity needed to reach the position
    # in one dynamic time step:
    velocity = ctrl / dynStepSize
    if velocity > velUpperLimit:
        velocity = velUpperLimit

    if velocity < -velUpperLimit:
        velocity = -velUpperLimit

    return velocity


jointHandle = sim.getObject('/Cuboid[0]/joint')
jointAngle = sim.getJointPosition(jointHandle)
sim.setJointTargetVelocity(jointHandle, 360 * math.pi / 180)

# enable the stepping mode on the client:
client.setStepping(True)

sim.startSimulation()

moveToAngle(45 * math.pi / 180)
moveToAngle(90 * math.pi / 180)
moveToAngle(-89 * math.pi / 180)  # no -90, to avoid passing below
moveToAngle(0 * math.pi / 180)

sim.stopSimulation()

print('Program ended')
