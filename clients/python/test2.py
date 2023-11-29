# Make sure to have the add-on "ZMQ remote API" running in
# CoppeliaSim. Do not launch simulation, but run this script

import threading
import math
import time

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
    print(str(ui))
    #ret1, ret2 = sim.testCB("Paul", reentranceCallbackFirst, "Jeanine")
    #assert ret1 == 'Paul' and ret2 == 'Jeanine'
    dt = time.time() - startTime
    print("will block now...")
    sim.getInt32Param(sim.intparam_platform)
    print(f"Elapsed time: {dt} secs")
    
def blueRobot():
    robotColor = 'blue'
    client = RemoteAPIClient()
    sim = client.require('sim')
    sims[robotColor] = sim 
    sim.setStepping(True)

    jointHandles = []
    for i in range(7):
        jointHandles.append(sim.getObject('/' + robotColor + 'Robot/joint', {'index': i}))

    vel = 110 * math.pi / 180
    accel = 40 * math.pi / 180
    jerk = 80 * math.pi / 180

    maxVel = [vel, vel, vel, vel, vel, vel, vel]
    maxAccel = [accel, accel, accel, accel, accel, accel, accel]
    maxJerk = [jerk, jerk, jerk, jerk, jerk, jerk, jerk]

    for i in range(1):
        targetPos1 = [90 * math.pi / 180, 90 * math.pi / 180, 170 * math.pi / 180, -90 * math.pi / 180, 90 * math.pi / 180, 90 * math.pi / 180, 0]
        moveToConfig(robotColor, jointHandles, maxVel, maxAccel, maxJerk, targetPos1)

        targetPos2 = [-90 * math.pi / 180, 90 * math.pi / 180, 180 * math.pi / 180, -90 * math.pi / 180, 90 * math.pi / 180, 90 * math.pi / 180, 0]
        moveToConfig(robotColor, jointHandles, maxVel, maxAccel, maxJerk, targetPos2)

        targetPos3 = [0, 0, 0, 0, 0, 0, 0]
        moveToConfig(robotColor, jointHandles, maxVel, maxAccel, maxJerk, targetPos3)

    sim.setStepping(False)

def redRobot():
    robotColor = 'red'
    client = RemoteAPIClient()
    sim = client.require('sim')
    sims[robotColor] = sim 
    sim.setStepping(True)

    jointHandles = []
    for i in range(7):
        jointHandles.append(sim.getObject('/' + robotColor + 'Robot/joint', {'index': i}))

    vel = 110 * math.pi / 180
    accel = 100 * math.pi / 180
    jerk = 200 * math.pi / 180

    maxVel = [vel, vel, vel, vel, vel, vel, vel]
    maxAccel = [accel, accel, accel, accel, accel, accel, accel]
    maxJerk = [jerk, jerk, jerk, jerk, jerk, jerk, jerk]

    for i in range(1):
        targetPos1 = [90 * math.pi / 180, 90 * math.pi / 180, 170 * math.pi / 180, -90 * math.pi / 180, 90 * math.pi / 180, 90 * math.pi / 180, 0]
        moveToConfig(robotColor, jointHandles, maxVel, maxAccel, maxJerk, targetPos1)

        targetPos2 = [-90 * math.pi / 180, 90 * math.pi / 180, 180 * math.pi / 180, -90 * math.pi / 180, 90 * math.pi / 180, 90 * math.pi / 180, 0]
        moveToConfig(robotColor, jointHandles, maxVel, maxAccel, maxJerk, targetPos2)

        targetPos3 = [0, 0, 0, 0, 0, 0, 0]
        moveToConfig(robotColor, jointHandles, maxVel, maxAccel, maxJerk, targetPos3)

    sim.setStepping(False)

def greenRobot():
    robotColor = 'green'
    client = RemoteAPIClient()
    sim = client.require('sim')
    sims[robotColor] = sim 
    sim.setStepping(True)

    targetHandle = sim.getObject('/' + robotColor + 'Robot/target')

    vel = 0.5
    accel = 0.1
    jerk = 3

    maxVel = [vel, vel, vel, vel]
    maxAccel = [accel, accel, accel, accel * 10]
    maxJerk = [jerk, jerk, jerk, jerk]
    
    initTr = sim.getObjectPose(targetHandle)

    for i in range(1):
        goalTr = initTr.copy()
        goalTr[2] = goalTr[2] + 0.2
        sim.moveToPose(-1, initTr, maxVel, maxAccel, maxJerk, goalTr, poseCallback, {'robotColor': robotColor, 'handle': targetHandle})
        
        startTr = sim.getObjectPose(targetHandle)
        goalTr[2] = goalTr[2] - 0.2
        sim.moveToPose(-1, startTr, maxVel, maxAccel, maxJerk, goalTr, poseCallback, {'robotColor': robotColor, 'handle': targetHandle})
        
        startTr = sim.getObjectPose(targetHandle)
        goalTr = sim.rotateAroundAxis(goalTr, [1, 0, 0], [startTr[0], startTr[1], startTr[2]], 90 * math.pi / 180)
        sim.moveToPose(-1, startTr, maxVel, maxAccel, maxJerk, goalTr, poseCallback, {'robotColor': robotColor, 'handle': targetHandle})

        startTr = sim.getObjectPose(targetHandle)
        sim.moveToPose(-1, startTr, maxVel, maxAccel, maxJerk, initTr, poseCallback, {'robotColor': robotColor, 'handle': targetHandle})
    
    sim.setStepping(False)
    
def findConfigRobot():
    class test:
        def configurationValidationCallback(self, config, auxData):
            global verif
            verif = verif + 1
            #print("test CB", config)
            return True

    def configurationValidationCallback(config, auxData):
        global verif
        verif = verif + 1
        #print("CB", config)
        return True
            
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK=client.require('simIK')
    simJointHandles = []
    for i in range(1, 8):  # Python ranges are zero-indexed and exclusive on the upper bound
        simJointHandles.append(sim.getObject('/LBR4p/j', {'index': i-1}))

    simTip = sim.getObject('/LBR4p/tip')
    simBase = sim.getObject('/LBR4p')
    simTarget = sim.getObject('/LBR4p/target')

    targets = [
        sim.getObject('/LBR4p/testTarget5'),
        sim.getObject('/LBR4p/testTarget6'),
        sim.getObject('/LBR4p/testTarget7'),
        sim.getObject('/LBR4p/testTarget8')
    ]

    cnt1 = 0
    cnt2 = 0

    ikEnv = simIK.createEnvironment()

    ikGroup = simIK.createGroup(ikEnv)
    ikElement, simToIkObjectMapping, *_ = simIK.addElementFromScene(ikEnv, ikGroup, simBase, simTip, simTarget, simIK.constraint_pose)

    simIK.setElementPrecision(ikEnv, ikGroup, ikElement, [0.00005, 0.1 * math.pi / 180])

    ikJointHandles = []
    for i in range(len(simJointHandles)):
        ikJointHandles.append(simToIkObjectMapping[simJointHandles[i]])

    ikTarget = simToIkObjectMapping[simTarget]
    ikBase = simToIkObjectMapping[simBase]

    bla = test()
    cc = False
    while not sim.getSimulationStopping():
        cc = not cc
        dummy_handle = targets[cnt1]
        object_matrix = sim.getObjectMatrix(dummy_handle, simBase)
        simIK.setObjectMatrix(ikEnv, ikTarget, object_matrix, ikBase)
        
        #sim.acquireLock()
        if cc:
            cb = bla.configurationValidationCallback
        else:
            cb = configurationValidationCallback
        global verif
        verif = 0
        state = simIK.findConfig( # generates a callback across c-boundary
            ikEnv, ikGroup, ikJointHandles, 0.25, 0.1, [1, 1, 1, 0.11],
            cb, simJointHandles
        )
        assert verif > 0
        #sim.releaseLock()
        
        if state:
           for i in range(len(simJointHandles)):
                sim.setJointPosition(simJointHandles[i], state[i])
        
        cnt2 += 1
        if cnt2 > 20:
            cnt2 = 0
            cnt1 += 1
            if cnt1 > 3:
                cnt1 = 0
    
def findConfig2Robot():
    pathPlanningThread = threading.Thread(target=omplRobot)
    pathPlanningThread.start()
    pathPlanningThread.join()

    class test:
        def configurationValidationCallback(self, config, auxData):
            global verif2
            verif2 = verif2 + 1
            #print("test CB", config)
            return True

    def configurationValidationCallback(config, auxData):
        global verif2
        verif2 = verif2 + 1
        #print("CB", config)
        return True
            
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK=client.require('simIK')
    simJointHandles = []
    for i in range(1, 8):  # Python ranges are zero-indexed and exclusive on the upper bound
        simJointHandles.append(sim.getObject('/LBR4p2/j', {'index': i-1}))

    simTip = sim.getObject('/LBR4p2/tip')
    simBase = sim.getObject('/LBR4p2')
    simTarget = sim.getObject('/LBR4p2/target')

    targets = [
        sim.getObject('/LBR4p2/testTarget5'),
        sim.getObject('/LBR4p2/testTarget6'),
        sim.getObject('/LBR4p2/testTarget7'),
        sim.getObject('/LBR4p2/testTarget8')
    ]

    cnt1 = 0
    cnt2 = 0

    ikEnv = simIK.createEnvironment()

    ikGroup = simIK.createGroup(ikEnv)
    ikElement, simToIkObjectMapping, *_ = simIK.addElementFromScene(ikEnv, ikGroup, simBase, simTip, simTarget, simIK.constraint_pose)

    simIK.setElementPrecision(ikEnv, ikGroup, ikElement, [0.00005, 0.1 * math.pi / 180])

    ikJointHandles = []
    for i in range(len(simJointHandles)):
        ikJointHandles.append(simToIkObjectMapping[simJointHandles[i]])

    ikTarget = simToIkObjectMapping[simTarget]
    ikBase = simToIkObjectMapping[simBase]

    bla = test()
    cc = False
    while not sim.getSimulationStopping():
        cc = not cc
        dummy_handle = targets[cnt1]
        object_matrix = sim.getObjectMatrix(dummy_handle, simBase)
        simIK.setObjectMatrix(ikEnv, ikTarget, object_matrix, ikBase)
        
        #sim.acquireLock()
        if cc:
            cb = bla.configurationValidationCallback
        else:
            cb = configurationValidationCallback
        global verif2
        verif2 = 0
        state = simIK.findConfig( # generates a callback across c-boundary
            ikEnv, ikGroup, ikJointHandles, 0.25, 0.1, [1, 1, 1, 0.11],
            cb, simJointHandles
        )
        assert verif2 > 0
        #sim.releaseLock()
        
        if state:
           for i in range(len(simJointHandles)):
                sim.setJointPosition(simJointHandles[i], state[i])
        
        cnt2 += 1
        if cnt2 > 20:
            cnt2 = 0
            cnt1 += 1
            if cnt1 > 3:
                cnt1 = 0
    
def omplRobot():
    def stateValidation(state):
        global verif3
        verif3 += 1
        #print("stateValidationCallback", state)
        #sim.addLog(sim.verbosity_scriptinfos, str(state))
        savedState = simOMPL.readState(omplTask)
        simOMPL.writeState(omplTask, state)
        simOMPL.writeState(omplTask, savedState)
        return True
            
    client = RemoteAPIClient()
    sim = client.require('sim')
    simOMPL=client.require('simOMPL')
    maxDistance = 0.05  # max allowed distance
    minDistance = 0.01  # min allowed distance
    robotHandle = sim.getObject('/StartConf*')
    targetHandle = sim.getObject('/GoalConf*')
    initPos = sim.getObjectPosition(robotHandle)
    initOrient = sim.getObjectOrientation(robotHandle)
    omplTask = simOMPL.createTask('omplTask')
    ss = [simOMPL.createStateSpace('2d', simOMPL.StateSpaceType.pose2d, robotHandle, [-0.5, -0.5], [0.5, 0.5], 1)]
    simOMPL.setStateSpace(omplTask, ss)
    simOMPL.setAlgorithm(omplTask, simOMPL.Algorithm.RRTConnect)
    simOMPL.setStateValidationCallback(omplTask, stateValidation)
    startPos = sim.getObjectPosition(robotHandle)
    startOrient = sim.getObjectOrientation(robotHandle)
    startPose = [startPos[0], startPos[1], startOrient[2]]
    simOMPL.setStartState(omplTask, startPose)
    goalPos = sim.getObjectPosition(targetHandle, -1)
    goalOrient = sim.getObjectOrientation(targetHandle, -1)
    goalPose = [goalPos[0], goalPos[1], goalOrient[2]]
    simOMPL.setGoalState(omplTask, goalPose)
    simOMPL.setup(omplTask)
    global verif3
    verif3 = 0
    simOMPL.solve(omplTask, 0.01)
    assert verif3 > 0
    
    
def poseCallback(tr, vel, accel, data):
    sim = sims[data['robotColor']]
    handle = data['handle']
    sim.setObjectPose(handle, tr)

def confCallback(config, vel, accel, data):
    sim = sims[data['robotColor']]
    handles = data['handles']
    for i in range(len(handles)):
        if sim.isDynamicallyEnabled(handles[i]):
            sim.setJointTargetPosition(handles[i], config[i])
        else:
            sim.setJointPosition(handles[i], config[i])

def moveToConfig(robotColor, handles, maxVel, maxAccel, maxJerk, targetConf):
    sim = sims[robotColor]
    currentConf = []
    for i in range(len(handles)):
        currentConf.append(sim.getJointPosition(handles[i]))
    sim.moveToConfig(-1, currentConf, None, None, maxVel, maxAccel, maxJerk, targetConf, None, confCallback, {'robotColor': robotColor, 'handles': handles}, None)

print('Program started')

client = RemoteAPIClient()
sim = client.require('sim')
simIK = client.require('simIK')

sim.loadScene(sim.getStringParam(sim.stringparam_scenedefaultdir) + '/../test2Scene.ttt')

simUI = client.require('simUI')
xml ='<ui title="Threaded"> <button text=" ***** Click me! (will then block/crash)***** " on-click="systemCallback"/> </ui>'
ui=simUI.create(xml)

global sims
sims = {}

for cnt in range(1000):
    print(f"Total iteration {cnt}/1000")

    blueRobotThread = threading.Thread(target=blueRobot)
    redRobotThread = threading.Thread(target=redRobot)
    greenRobotThread = threading.Thread(target=greenRobot)
    ikThread = threading.Thread(target=findConfigRobot)
    ik2Thread = threading.Thread(target=findConfig2Robot)

    blueRobotThread.start()
    redRobotThread.start()
    greenRobotThread.start()
    ikThread.start()
    ik2Thread.start()
    sim.startSimulation()


    blueRobotThread.join()
    redRobotThread.join()
    greenRobotThread.join()
    sim.stopSimulation(True)
    ikThread.join()
    ik2Thread.join()

    while sim.getSimulationState() != sim.simulation_stopped:
        pass

simUI.destroy(ui)

print('Program ended')


