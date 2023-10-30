# Make sure to have the add-on "ZMQ remote API" running in
# CoppeliaSim. Do not launch simulation, but run this script

import time
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from PIL import Image
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class ImageStreamDisplay:
    def __init__(self, resolution):
        self.fig, self.ax = plt.subplots()
        self.im_display = self.ax.imshow(np.zeros(resolution), animated=True)
        plt.ion()
        plt.show()

    def displayUpdated(self, rgbBuffer, resolution):
        img_data = np.frombuffer(rgbBuffer, dtype=np.uint8).reshape(resolution[1], resolution[0], 3)
        img_data = img_data[::-1, :, :]
        self.im_display.set_array(img_data)
        self.fig.canvas.flush_events()
        plt.pause(0.01)
        
print('Program started')

client = RemoteAPIClient()
sim = client.require('sim')

sim.loadScene(sim.getStringParam(sim.stringparam_scenedefaultdir) + '/messaging/synchronousImageTransmissionViaRemoteApi.ttt')

visionSensorHandle = sim.getObject('/VisionSensor')

_, resolution = sim.getVisionSensorImg(visionSensorHandle)

display = ImageStreamDisplay(resolution)

sim.startSimulation()

startTime = sim.getSimulationTime()
while sim.getSimulationTime() - startTime < 15:
    img, res = sim.getVisionSensorImg(visionSensorHandle)
    display.displayUpdated(img, res)
sim.stopSimulation()

print('Program ended')
