# Make sure to have CoppeliaSim running, with followig scene loaded:
#
# scenes/messaging/ikMovementViaRemoteApi.ttt
#
# Do not launch simulation, then run this script

from zmqRemoteApi import RemoteAPIClient

print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')

executedMovId = 'notReady'
targetArm = '/LBR4p'
stringSignalName = targetArm + '_executedMovId'


def waitForMovementExecuted(id_):
    global executedMovId, stringSignalName
    while executedMovId != id_:
        s = sim.getStringSignal(stringSignalName)
        executedMovId = s


# Set-up some movement variables:
maxVel = 0.1
maxAccel = 0.01

# Start simulation:
sim.startSimulation()

# Wait until ready:
waitForMovementExecuted('ready')

# Get initial pose:
initialPose, initialConfig = sim.callScriptFunction(
    'remoteApi_getPoseAndConfig' + '@' + targetArm,
    sim.scripttype_childscript)

# Send first movement sequence:
targetPose = [0, 0, 0.85, 0, 0, 0, 1]
movementData = {
    'id': 'movSeq1',
    'type': 'mov',
    'targetPose': targetPose,
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
targetPose = [
    0, 0, 0.85,
    -0.7071068883, -6.252754758e-08, -8.940695295e-08, -0.7071067691
]
movementData = {
    'id': 'movSeq2',
    'type': 'mov',
    'targetPose': targetPose,
    'maxVel': maxVel,
    'maxAccel': maxAccel
}
sim.callScriptFunction(
    'remoteApi_movementDataFunction' + '@' + targetArm,
    sim.scripttype_childscript,
    movementData)
movementData = {
    'id': 'movSeq3',
    'type': 'mov',
    'targetPose': initialPose,
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
