# Testing

import time
import numpy as np

#from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from zmqRemoteApi import RemoteAPIClient # use the local source code for testing

def reentranceCallback(a,b):
    return a,b

def reentranceCallbackFirst(a,b):
    ret1, ret2 = sim.testCB("Paul", reentranceCallback, "Jeanine", 1)
    assert ret1 == 'Paul' and ret2 == 'Jeanine'
    return a,b

def systemCallback(ui,button):
    print("button was clicked!")
    startTime = time.time()
    ret1, ret2 = sim.testCB("Paul", reentranceCallbackFirst, "Jeanine")
    assert ret1 == 'Paul' and ret2 == 'Jeanine'
    dt = time.time() - startTime
    print(f"Elapsed time: {dt} secs")

print('Program started')

client = RemoteAPIClient()
client.timeout = 5

sim = client.require('sim')
simIK = client.require('simIK')

a = [1,2,3]
b = sim.packFloatTable(a)
c = sim.unpackFloatTable(b)
print(a)
print(c)

sim.loadScene('')

simIK.createEnvironment()

a = np.random.uniform(-1, 1, 2)
a = np.array(a, dtype=np.float32)
x, y = a
handles = sim.getObjectsInTree(sim.handle_scene)
sp = sim.getObjectPose(handles[0])
sim.setObjectPose(handles[0], [x, y, 0, 0, 0, 0, 1])
sim.setObjectPose(handles[0], sp)

simUI = client.require('simUI')
xml ='<ui title="Threaded"> <button text=" ***** Click me! ***** " on-click="systemCallback"/> </ui>'
ui=simUI.create(xml)
startTime = time.time()
print(f"Click button for 5 seconds")
while time.time() - startTime < 5:
    sim.handleExtCalls()

print(f"Click button for 3 seconds")
sim.wait(3, False)

print(f"Click button for 5 seconds")
sim.setStepping(True)
sim.startSimulation()
startTime = time.time()
while time.time() - startTime < 5:
    sim.step()
sim.stopSimulation(True)
sim.setStepping(False)

simUI.destroy(ui)

sim.startSimulation()
sim.wait(20, True)
print('Should be very close to 20: ', sim.getSimulationTime())
sim.stopSimulation(True)

for cnt in range(1000):
    print(f"Total iteration {cnt}/1000")
    totTime1 = 0.0
    totTime2 = 0.0
    totCnt = 0.0
    
    startATime = time.time()
    sim.setStepping(False)
    sim.startSimulation()
    itCnt = 0
    while sim.getSimulationTime() < 5:
        startTime = time.time()
        ret1, ret2 = sim.testCB("Paul",reentranceCallback,"Jeanine")
        assert ret1 == 'Paul' and ret2 == 'Jeanine'
        totTime1 += time.time() - startTime
        totCnt += 1.0
        startTime = time.time()
        sim.getInt32Param(sim.intparam_platform)
        sim.getInt32Param(sim.intparam_platform)
        sim.getInt32Param(sim.intparam_platform)
        sim.getInt32Param(sim.intparam_platform)
        sim.getInt32Param(sim.intparam_platform)
        sim.getInt32Param(sim.intparam_platform)
        sim.getInt32Param(sim.intparam_platform)
        sim.getInt32Param(sim.intparam_platform)
        sim.getInt32Param(sim.intparam_platform)
        sim.getInt32Param(sim.intparam_platform)
        totTime2 += time.time() - startTime
        itCnt += 1
    sim.stopSimulation(True)
    print(f"Average sim. step time (sim1): {(time.time() - startATime)/100} secs, {itCnt} iterations")

    startATime = time.time()
    sim.setStepping(True)
    sim.startSimulation()
    itCnt = 0
    while sim.getSimulationTime() < 5:
        startTime = time.time()
        ret1, ret2 = sim.testCB("Paul",reentranceCallback,"Jeanine")
        assert ret1 == 'Paul' and ret2 == 'Jeanine'
        totTime1 += time.time() - startTime
        totCnt += 1.0
        startTime = time.time()
        sim.getInt32Param(sim.intparam_platform)
        sim.getInt32Param(sim.intparam_platform)
        sim.getInt32Param(sim.intparam_platform)
        sim.getInt32Param(sim.intparam_platform)
        sim.getInt32Param(sim.intparam_platform)
        sim.getInt32Param(sim.intparam_platform)
        sim.getInt32Param(sim.intparam_platform)
        sim.getInt32Param(sim.intparam_platform)
        sim.getInt32Param(sim.intparam_platform)
        sim.getInt32Param(sim.intparam_platform)
        totTime2 += time.time() - startTime
        sim.step()
        itCnt += 1
    assert itCnt == 101
    sim.stopSimulation(True)
    print(f"Average sim. step time (sim2): {(time.time() - startATime)/100} secs, {itCnt} iterations")
        
    startATime = time.time()
    sim.setStepping(False)
    sim.startSimulation()
    itCnt = 0
    while sim.getSimulationTime() < 5:
        itCnt += 1
    sim.stopSimulation(True)
    print(f"Average sim. step time (sim3): {(time.time() - startATime)/100} secs, {itCnt} iterations")

    startATime = time.time()
    sim.setStepping(True)
    sim.startSimulation()
    itCnt = 0
    while sim.getSimulationTime() < 5:
        sim.step()
        itCnt += 1
    assert itCnt == 101
    sim.stopSimulation(True)
    print(f"Average sim. step time (sim4): {(time.time() - startATime)/100} secs, {itCnt} iterations")

    print(f"Average sim.testCB: {totTime1 / totCnt} secs")
    print(f"Average sim.getInt32Param: {totTime2 / (totCnt * 10)} secs")

print('Program ended')


