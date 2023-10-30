# Make sure to have the add-on "ZMQ remote API" running in
# CoppeliaSim. Do not launch simulation, but run this script

import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


print('Program started')

client = RemoteAPIClient()
sim = client.require('sim')

sim.loadScene(sim.getStringParam(sim.stringparam_scenedefaultdir) + '/messaging/synchronousImageTransmissionViaRemoteApi.ttt')

visionSensorHandle = sim.getObject('/VisionSensor')
passiveVisionSensorHandle = sim.getObject('/PassiveVisionSensor')

sim.setStepping(True)
sim.startSimulation()

startTime = sim.getSimulationTime()
while sim.getSimulationTime() - startTime < 15:
    img, res = sim.getVisionSensorImg(visionSensorHandle)
    sim.setVisionSensorImg(passiveVisionSensorHandle, img)
    sim.step()
sim.stopSimulation()

print('Program ended')
