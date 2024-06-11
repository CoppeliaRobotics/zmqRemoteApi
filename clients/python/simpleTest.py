# Make sure to have the add-on "ZMQ remote API"
# running in CoppeliaSim
#
# All CoppeliaSim commands will run in blocking mode (block
# until a reply from CoppeliaSim is received). For a non-
# blocking example, see simpleTest-nonBlocking.py

import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def myFunc(input1, input2):
    print('Hello', input1, input2)
    return 21

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
        'to client, i.e. non-stepping)'
    print(s)
    sim.addLog(sim.verbosity_scriptinfos, s)
    # sim.testCB(21,myFunc,42) # see below. sim.testCB is calling back above "myFunc"

# e.g. calling a script object function (make sure the script is running!):
'''    
script = sim.getObject('/path/to/scriptObject')
reply = sim.callScriptFunction('functionName', script, 'Hello', 'Paul', 21)
'''
#or
'''
script = sim.getObject('/path/to/scriptObject')
funcs = client.getScriptFunctions(script)
reply = funcs.functionName('Hello', 'Paul', 21)
'''    
    
sim.stopSimulation()
# If you need to make sure we really stopped:
while sim.getSimulationState() != sim.simulation_stopped:
    time.sleep(0.1)

# Run a simulation in stepping mode:
sim.setStepping(True)
sim.startSimulation()
while (t := sim.getSimulationTime()) < 3:
    s = f'Simulation time: {t:.2f} [s] (simulation running synchronously '\
        'to client, i.e. stepping)'
    print(s)
    sim.addLog(sim.verbosity_scriptinfos, s)
    sim.step()  # triggers next simulation step
sim.stopSimulation()

# Remove the dummies created earlier:
for h in handles:
    sim.removeObject(h)

print('Program ended')

''' test callback on CoppeliaSim side:
int ret = sim.testCB(int a, func cb, int b)
function sim.testCB(a, cb, b)
    for i = 1, 99, 1 do
        cb(a, b)
    end
    return cb(a, b)
end
'''
