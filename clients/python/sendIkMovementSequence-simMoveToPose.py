# Make sure to have CoppeliaSim running, with followig scene loaded:
#
# scenes/messaging/ikMovementViaRemoteApi.ttt
#
# Do not launch simulation, then run this script

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')

tipHandle = sim.getObject('/LBR4p/tip')
targetHandle = sim.getObject('/LBR4p/target')

# Set-up some movement variables:
maxVel = 0.1
maxAccel = 0.01
maxJerk = 80

# Start simulation:
sim.startSimulation()

def cb(pose,vel,accel,handle):
    sim.setObjectPose(handle,-1,pose)


# Send movement sequences:
initialPose = sim.getObjectPose(tipHandle,-1)
targetPose = [0, 0, 0.85, 0, 0, 0, 1]
sim.moveToPose(-1,initialPose,[maxVel],[maxAccel],[maxJerk],targetPose,cb,targetHandle,[1,1,1,0.1])

targetPose = [
    0, 0, 0.85,
    -0.7071068883, -6.252754758e-08, -8.940695295e-08, -0.7071067691
]
sim.moveToPose(-1,sim.getObjectPose(tipHandle,-1),[maxVel],[maxAccel],[maxJerk],targetPose,cb,targetHandle,[1,1,1,0.1])

sim.moveToPose(-1,sim.getObjectPose(tipHandle,-1),[maxVel],[maxAccel],[maxJerk],initialPose,cb,targetHandle,[1,1,1,0.1])

sim.stopSimulation()

print('Program ended')
