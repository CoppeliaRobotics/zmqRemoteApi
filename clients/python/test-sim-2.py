# Make sure to have the add-on "ZMQ remote API"
# running in CoppeliaSim
#
# All CoppeliaSim commands will run in blocking mode (block
# until a reply from CoppeliaSim is received). For a non-
# blocking example, see simpleTest-nonBlocking.py

import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require('sim-2')
print("sim.getObject('/Floor')", sim.getObject('/Floor'))
print("sim.getVector3Property(sim.handle_scene, 'gravity')", sim.getVector3Property(sim.handle_scene, 'gravity'))
print("sim.getColorProperty(sim.handle_scene, 'ambientLight')", sim.getColorProperty(sim.handle_scene, 'ambientLight'))
print("sim.getQuaternionProperty(sim.handle_app, 'randomQuaternion')", sim.getQuaternionProperty(sim.handle_app, 'randomQuaternion'))
print("sim.getPoseProperty(sim.getObject('/Floor'), 'pose')", sim.getPoseProperty(sim.getObject('/Floor'), 'pose'))
