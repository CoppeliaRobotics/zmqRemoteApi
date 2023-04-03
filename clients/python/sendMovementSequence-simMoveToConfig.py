# Make sure to have CoppeliaSim running, with followig scene loaded:
#
# scenes/messaging/movementViaRemoteApi.ttt
#
# Do not launch simulation, then run this script

import math

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def movCallback(config,vel,accel,handles):
    for i in range(len(handles)):
        if sim.getJointMode(handles[i])[0]==sim.jointmode_force and sim.isDynamicallyEnabled(handles[i]):
            sim.setJointTargetPosition(handles[i],config[i])
        else:
            sim.setJointPosition(handles[i],config[i])

print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')

targetArm = '/blueArm'
# targetArm = '/redArm'

jointHandles=[]
for i in range(6):
    jointHandles.append(sim.getObject(targetArm+'/joint',{'index':i}))

# Set-up some movement variables:
mVel = 100 * math.pi / 180
mAccel = 150 * math.pi / 180
mJerk = 100 * math.pi / 180
maxVel=[mVel,mVel,mVel,mVel,mVel,mVel]
maxAccel=[mAccel,mAccel,mAccel,mAccel,mAccel,mAccel]
maxJerk=[mJerk,mJerk,mJerk,mJerk,mJerk,mJerk]

# Start simulation:
sim.startSimulation()

# Send 3 movement sequences:
currentConf = [ 0, 0, 0, 0, 0, 0 ]
targetConfig = [
    90 * math.pi / 180, 90 * math.pi / 180, -90 * math.pi / 180,
    90 * math.pi / 180, 90 * math.pi / 180, 90 * math.pi / 180
]
sim.moveToConfig(-1,currentConf,None,None,maxVel,maxAccel,maxJerk,targetConfig,None,movCallback,jointHandles)

currentConf = targetConfig
targetConfig = [
    -90 * math.pi / 180, 45 * math.pi / 180, 90 * math.pi / 180,
    135 * math.pi / 180, 90 * math.pi / 180, 90 * math.pi / 180
]
targetVel = [-60 * math.pi / 180, -20 * math.pi / 180, 0, 0, 0, 0]
sim.moveToConfig(-1,currentConf,None,None,maxVel,maxAccel,maxJerk,targetConfig,targetVel,movCallback,jointHandles)

currentConf = targetConfig
targetConfig = [0, 0, 0, 0, 0, 0]
sim.moveToConfig(-1,currentConf,None,None,maxVel,maxAccel,maxJerk,targetConfig,None,movCallback,jointHandles)

sim.stopSimulation()

print('Program ended')
