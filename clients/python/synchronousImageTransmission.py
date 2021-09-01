# Make sure to have the add-on "ZMQ remote API" running in
# CoppeliaSim and have following scene loaded:
#
# scenes/messaging/synchronousImageTransmissionViaRemoteApi.ttt
#
# Do not launch simulation, but run this script
#

import time

from zmqRemoteApi import RemoteAPIClient


print('Program started')

client = RemoteAPIClient()
sim = client.getobject('sim')

visionSensorHandle = sim.getObjectHandle('/VisionSensor')
passiveVisionSensorHandle = sim.getObjectHandle('/PassiveVisionSensor')

client.setstepping(True)
sim.startSimulation()

startTime = time.time()
while time.time()-startTime < 5:
    img, resX, resY = sim.getVisionSensorCharImage(visionSensorHandle)
    sim.setVisionSensorCharImage(passiveVisionSensorHandle, img)
    client.step()
sim.stopSimulation()

print('Program ended')
