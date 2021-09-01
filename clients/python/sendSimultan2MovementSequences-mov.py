# Make sure to have CoppeliaSim running, with followig scene loaded:
#
# scenes/messaging/movementViaRemoteApi.ttt
#
# Do not launch simulation, then run this script

from zmqRemoteApi import RemoteAPIClient
import math

class Client:
    def __enter__(self):
        self.executedMovId1='notReady'
        self.executedMovId2='notReady'
        self.client = RemoteAPIClient('localhost',23000)
        self.sim = self.client.getobject('sim')
        print ('Program started')
        return self
    
    def __exit__(self,*err):
        print ('Program ended')

with Client() as client:
    targetArm1='/blueArm'
    targetArm2='/redArm'

    client.stringSignalName1=targetArm1+'_executedMovId'
    client.stringSignalName2=targetArm2+'_executedMovId'

    def waitForMovementExecuted1(id):
        while client.executedMovId1!=id:
            s=client.sim.getStringSignal(client.stringSignalName1)
            if type(s)==bytearray:
                s=s.decode('ascii') # python2/python3 differences
            client.executedMovId1=s

    def waitForMovementExecuted2(id):
        while client.executedMovId2!=id:
            s=client.sim.getStringSignal(client.stringSignalName2)
            if type(s)==bytearray:
                s=s.decode('ascii') # python2/python3 differences
            client.executedMovId2=s

    # Set-up some movement variables:
    mVel=100*math.pi/180
    mAccel=150*math.pi/180
    maxVel=[mVel,mVel,mVel,mVel,mVel,mVel]
    maxAccel=[mAccel,mAccel,mAccel,mAccel,mAccel,mAccel]
    targetVel=[0,0,0,0,0,0]

    # Start simulation:
    client.sim.startSimulation()

    # Wait until ready:
    waitForMovementExecuted1(b'ready') 
    waitForMovementExecuted1(b'ready') 

    # Send first movement sequence:
    targetConfig=[90*math.pi/180,90*math.pi/180,-90*math.pi/180,90*math.pi/180,90*math.pi/180,90*math.pi/180]
    movementData={"id":"movSeq1","type":"mov","targetConfig":targetConfig,"targetVel":targetVel,"maxVel":maxVel,"maxAccel":maxAccel}
    client.sim.callScriptFunction('remoteApi_movementDataFunction'+'@'+targetArm1,client.sim.scripttype_childscript,movementData)
    client.sim.callScriptFunction('remoteApi_movementDataFunction'+'@'+targetArm2,client.sim.scripttype_childscript,movementData)

    # Execute first movement sequence:
    client.sim.callScriptFunction('remoteApi_executeMovement'+'@'+targetArm1,client.sim.scripttype_childscript,'movSeq1')
    client.sim.callScriptFunction('remoteApi_executeMovement'+'@'+targetArm2,client.sim.scripttype_childscript,'movSeq1')
    
    # Wait until above movement sequence finished executing:
    waitForMovementExecuted1(b'movSeq1') 
    waitForMovementExecuted1(b'movSeq1') 

    # Send second and third movement sequence, where third one should execute immediately after the second one:
    targetConfig=[-90*math.pi/180,45*math.pi/180,90*math.pi/180,135*math.pi/180,90*math.pi/180,90*math.pi/180]
    targetVel=[-60*math.pi/180,-20*math.pi/180,0,0,0,0]
    movementData={"id":"movSeq2","type":"mov","targetConfig":targetConfig,"targetVel":targetVel,"maxVel":maxVel,"maxAccel":maxAccel}
    client.sim.callScriptFunction('remoteApi_movementDataFunction'+'@'+targetArm1,client.sim.scripttype_childscript,movementData)
    client.sim.callScriptFunction('remoteApi_movementDataFunction'+'@'+targetArm2,client.sim.scripttype_childscript,movementData)
    targetConfig=[0,0,0,0,0,0]
    targetVel=[0,0,0,0,0,0]
    movementData={"id":"movSeq3","type":"mov","targetConfig":targetConfig,"targetVel":targetVel,"maxVel":maxVel,"maxAccel":maxAccel}
    client.sim.callScriptFunction('remoteApi_movementDataFunction'+'@'+targetArm1,client.sim.scripttype_childscript,movementData)
    client.sim.callScriptFunction('remoteApi_movementDataFunction'+'@'+targetArm2,client.sim.scripttype_childscript,movementData)

    # Execute second and third movement sequence:
    client.sim.callScriptFunction('remoteApi_executeMovement'+'@'+targetArm1,client.sim.scripttype_childscript,'movSeq2')
    client.sim.callScriptFunction('remoteApi_executeMovement'+'@'+targetArm2,client.sim.scripttype_childscript,'movSeq2')
    client.sim.callScriptFunction('remoteApi_executeMovement'+'@'+targetArm1,client.sim.scripttype_childscript,'movSeq3')
    client.sim.callScriptFunction('remoteApi_executeMovement'+'@'+targetArm2,client.sim.scripttype_childscript,'movSeq3')
    
    # Wait until above 2 movement sequences finished executing:
    waitForMovementExecuted1(b'movSeq3')
    waitForMovementExecuted1(b'movSeq3')
    
    client.sim.stopSimulation()

