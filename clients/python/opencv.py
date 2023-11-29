# Make sure to have the add-on "ZMQ remote API" running in
# CoppeliaSim. Do not launch simulation, but run this script

import time

import numpy as np
import cv2

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


print('Program started')

client = RemoteAPIClient()
sim = client.require('sim')

sim.loadScene(sim.getStringParam(sim.stringparam_scenedefaultdir) + '/messaging/synchronousImageTransmissionViaRemoteApi.ttt')

visionSensorHandle = sim.getObject('/VisionSensor')
passiveVisionSensorHandle = sim.getObject('/PassiveVisionSensor')

# Run a simulation in stepping mode:
sim.setStepping(True)
sim.startSimulation()

while (t := sim.getSimulationTime()) < 3:
    img, [resX, resY] = sim.getVisionSensorImg(visionSensorHandle)
    img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)

    # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
    # (consistent with the axes of vision sensors, pointing Z outwards, Y up)
    # and color format is RGB triplets, whereas OpenCV uses BGR:
    img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)

    cv2.imshow('', img)
    cv2.waitKey(1)
    sim.step()  # triggers next simulation step

sim.stopSimulation()

cv2.destroyAllWindows()

print('Program ended')
