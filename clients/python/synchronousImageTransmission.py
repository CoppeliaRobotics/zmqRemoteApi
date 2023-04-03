# Make sure to have the add-on "ZMQ remote API" running in
# CoppeliaSim and have following scene loaded:
#
# scenes/messaging/synchronousImageTransmissionViaRemoteApi.ttt
#
# Do not launch simulation, but run this script
#

import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')

visionSensorHandle = sim.getObject('/VisionSensor')
passiveVisionSensorHandle = sim.getObject('/PassiveVisionSensor')

client.setStepping(True)
sim.startSimulation()

startTime = sim.getSimulationTime()
while sim.getSimulationTime() - startTime < 5:
    img, resX, resY = sim.getVisionSensorCharImage(visionSensorHandle)
    sim.setVisionSensorCharImage(passiveVisionSensorHandle, img)
    client.step()
sim.stopSimulation()

print('Program ended')
