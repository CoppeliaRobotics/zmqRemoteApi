# Make sure to have the add-on "ZMQ remote API" running in
# CoppeliaSim. Do not launch simulation, but run this script

import threading
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

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

sim.loadScene(sim.getStringParam(sim.stringparam_scenedefaultdir) + '/messaging/movementViaRemoteApi.ttt')

global sims
sims = {}

blueRobotThread = threading.Thread(target=blueRobot)
redRobotThread = threading.Thread(target=redRobot)
greenRobotThread = threading.Thread(target=greenRobot)

blueRobotThread.start()
redRobotThread.start()
greenRobotThread.start()

sim.startSimulation()

blueRobotThread.join()
redRobotThread.join()
greenRobotThread.join()

sim.stopSimulation()

print('Program ended')


