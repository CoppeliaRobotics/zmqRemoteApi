# Make sure to have CoppeliaSim running, with followig scene loaded:
#
# scenes/messaging/movementViaRemoteApi.ttt
#
# Do not launch simulation, then run this script

import math

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

print('Program started')

executedMovId1 = 'notReady'
executedMovId2 = 'notReady'

client = RemoteAPIClient()
sim = client.getObject('sim')

targetArm1 = '/blueArm'
targetArm2 = '/redArm'

stringSignalName1 = targetArm1 + '_executedMovId'
stringSignalName2 = targetArm2 + '_executedMovId'

objHandle1 = sim.getObject(targetArm1)
scriptHandle1 = sim.getScript(sim.scripttype_childscript,objHandle1)
objHandle2 = sim.getObject(targetArm2)
scriptHandle2 = sim.getScript(sim.scripttype_childscript,objHandle2)

def waitForMovementExecuted1(id_):
    global executedMovId1
    while executedMovId1 != id_:
        s = sim.getStringSignal(stringSignalName1)
        executedMovId1 = s


def waitForMovementExecuted2(id_):
    global executedMovId2
    while executedMovId2 != id_:
        s = sim.getStringSignal(stringSignalName2)
        executedMovId2 = s


# Set-up some movement variables:
mVel = 100 * math.pi / 180
mAccel = 150 * math.pi / 180
maxVel = [mVel, mVel, mVel, mVel, mVel, mVel]
maxAccel = [mAccel, mAccel, mAccel, mAccel, mAccel, mAccel]
targetVel = [0, 0, 0, 0, 0, 0]

# Start simulation:
sim.startSimulation()

# Wait until ready:
waitForMovementExecuted1('ready')
waitForMovementExecuted1('ready')

# Send first movement sequence:
targetConfig = [
    90 * math.pi / 180, 90 * math.pi / 180, -90 * math.pi / 180,
    90 * math.pi / 180, 90 * math.pi / 180, 90 * math.pi / 180
]
movementData = {
    'id': 'movSeq1',
    'type': 'mov',
    'targetConfig': targetConfig,
    'targetVel': targetVel,
    'maxVel': maxVel,
    'maxAccel': maxAccel
}
sim.callScriptFunction('remoteApi_movementDataFunction',scriptHandle1,movementData)
sim.callScriptFunction('remoteApi_movementDataFunction',scriptHandle2,movementData)

# Execute first movement sequence:
sim.callScriptFunction('remoteApi_executeMovement',scriptHandle1,'movSeq1')
sim.callScriptFunction('remoteApi_executeMovement',scriptHandle2,'movSeq1')

# Wait until above movement sequence finished executing:
waitForMovementExecuted1('movSeq1')
waitForMovementExecuted1('movSeq1')

# Send second and third movement sequence, where third one should execute
# immediately after the second one:
targetConfig = [
    -90 * math.pi / 180, 45 * math.pi / 180, 90 * math.pi / 180,
    135 * math.pi / 180, 90 * math.pi / 180, 90 * math.pi / 180
]
targetVel = [-60 * math.pi / 180, -20 * math.pi / 180, 0, 0, 0, 0]
movementData = {
    'id': 'movSeq2',
    'type': 'mov',
    'targetConfig': targetConfig,
    'targetVel': targetVel,
    'maxVel': maxVel,
    'maxAccel': maxAccel
}
sim.callScriptFunction('remoteApi_movementDataFunction',scriptHandle1,movementData)
sim.callScriptFunction('remoteApi_movementDataFunction',scriptHandle2,movementData)
targetConfig = [0, 0, 0, 0, 0, 0]
targetVel = [0, 0, 0, 0, 0, 0]
movementData = {
    'id': 'movSeq3',
    'type': 'mov',
    'targetConfig': targetConfig,
    'targetVel': targetVel,
    'maxVel': maxVel,
    'maxAccel': maxAccel
}
sim.callScriptFunction('remoteApi_movementDataFunction',scriptHandle1,movementData)
sim.callScriptFunction('remoteApi_movementDataFunction',scriptHandle2,movementData)

# Execute second and third movement sequence:
sim.callScriptFunction('remoteApi_executeMovement',scriptHandle1,'movSeq2')
sim.callScriptFunction('remoteApi_executeMovement',scriptHandle2,'movSeq2')
sim.callScriptFunction('remoteApi_executeMovement',scriptHandle1,'movSeq3')
sim.callScriptFunction('remoteApi_executeMovement',scriptHandle2,'movSeq3')

# Wait until above 2 movement sequences finished executing:
waitForMovementExecuted1('movSeq3')
waitForMovementExecuted1('movSeq3')

sim.stopSimulation()

print('Program ended')
