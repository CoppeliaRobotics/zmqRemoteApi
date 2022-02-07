# Make sure to have CoppeliaSim running, with followig scene loaded:
#
# scenes/messaging/movementViaRemoteApi.ttt
#
# Do not launch simulation, then run this script

import math

from zmqRemoteApi import RemoteAPIClient

print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')

executedMovId = 'notReady'

targetArm = '/blueArm'
# targetArm = '/redArm'

stringSignalName = targetArm + '_executedMovId'


def waitForMovementExecuted(id_):
    global executedMovId, stringSignalName
    while executedMovId != id_:
        s = sim.getStringSignal(stringSignalName)
        executedMovId = s


# Set-up some movement variables:
mVel = 100 * math.pi / 180
mAccel = 150 * math.pi / 180
maxVel = [mVel, mVel, mVel, mVel, mVel, mVel]
maxAccel = [mAccel, mAccel, mAccel, mAccel, mAccel, mAccel]
targetVel = [0, 0, 0, 0, 0, 0]

# Start simulation:
sim.startSimulation()

# Wait until ready:
waitForMovementExecuted('ready')

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
sim.callScriptFunction(
    'remoteApi_movementDataFunction' + '@' + targetArm,
    sim.scripttype_childscript,
    movementData)

# Execute first movement sequence:
sim.callScriptFunction(
    'remoteApi_executeMovement' + '@' + targetArm,
    sim.scripttype_childscript,
    'movSeq1')

# Wait until above movement sequence finished executing:
waitForMovementExecuted('movSeq1')

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
sim.callScriptFunction(
    'remoteApi_movementDataFunction' + '@' + targetArm,
    sim.scripttype_childscript,
    movementData)
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
sim.callScriptFunction(
    'remoteApi_movementDataFunction' + '@' + targetArm,
    sim.scripttype_childscript,
    movementData)

# Execute second and third movement sequence:
sim.callScriptFunction(
    'remoteApi_executeMovement' + '@' + targetArm,
    sim.scripttype_childscript,
    'movSeq2')
sim.callScriptFunction(
    'remoteApi_executeMovement' + '@' + targetArm,
    sim.scripttype_childscript,
    'movSeq3')

# Wait until above 2 movement sequences finished executing:
waitForMovementExecuted('movSeq3')
sim.stopSimulation()

print('Program ended')
