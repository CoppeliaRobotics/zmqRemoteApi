# Make sure to have the add-on "ZMQ remote API"
# running in CoppeliaSim
#
# All CoppeliaSim commands will run in blocking mode (block
# until a reply from CoppeliaSim is received). For a non-
# blocking example, see simpleTest-nonBlocking.py

import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


print('Program started')

client = RemoteAPIClient()
sim = client.require('sim')

# Create a few dummies and set their positions:
handles = [sim.createDummy(0.01, 12 * [0]) for _ in range(50)]
for i, h in enumerate(handles):
    sim.setObjectPosition(h, -1, [0.01 * i, 0.01 * i, 0.01 * i])

# Run a simulation in asynchronous mode:
sim.startSimulation()
while (t := sim.getSimulationTime()) < 3:
    s = f'Simulation time: {t:.2f} [s] (simulation running asynchronously '\
        'to client, i.e. non-stepped)'
    print(s)
    sim.addLog(sim.verbosity_scriptinfos, s)
sim.stopSimulation()
# If you need to make sure we really stopped:
while sim.getSimulationState() != sim.simulation_stopped:
    time.sleep(0.1)

# Run a simulation in stepping mode:
client.setStepping(True)
sim.startSimulation()
while (t := sim.getSimulationTime()) < 3:
    s = f'Simulation time: {t:.2f} [s] (simulation running synchronously '\
        'to client, i.e. stepped)'
    print(s)
    sim.addLog(sim.verbosity_scriptinfos, s)
    client.step()  # triggers next simulation step
sim.stopSimulation()

# Remove the dummies created earlier:
for h in handles:
    sim.removeObject(h)

print('Program ended')
