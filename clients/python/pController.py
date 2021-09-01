# Make sure to have CoppeliaSim running, with followig scene loaded:
#
# scenes/messaging/pControllerViaRemoteApi.ttt
#
# Do not launch simulation, but run this script

from zmqRemoteApi import RemoteAPIClient
import math

class Client:
    def __enter__(self):
        self.intSignalName='legacyRemoteApiStepCounter'
        self.stepCounter=0
        self.maxForce=100
        self.client = RemoteAPIClient('localhost',23000)
        self.sim = self.client.getobject('sim')
        return self

    def __exit__(self,*err):
        a=1
        

with Client() as client:
    print("running")
    
    def moveToAngle(targetAngle):
        while abs(client.jointAngle-targetAngle)>0.1*math.pi/180:
            vel=computeTargetVelocity(client.jointAngle,targetAngle)
            client.sim.setJointTargetVelocity(client.jointHandle,vel)
            client.sim.setJointMaxForce(client.jointHandle,client.maxForce)
            client.client.step()
            client.jointAngle=client.sim.getJointPosition(client.jointHandle)

    def computeTargetVelocity(jointAngle,targetAngle):
        dynStepSize=0.005
        velUpperLimit=360*math.pi/180
        PID_P=0.1
        errorValue=targetAngle-jointAngle
        sinAngle=math.sin(errorValue)
        cosAngle=math.cos(errorValue)
        errorValue=math.atan2(sinAngle,cosAngle)
        ctrl=errorValue*PID_P
        
        # Calculate the velocity needed to reach the position in one dynamic time step:
        velocity=ctrl/dynStepSize
        if (velocity>velUpperLimit):
            velocity=velUpperLimit
            
        if (velocity<-velUpperLimit):
            velocity=-velUpperLimit
        
        return velocity
    
    client.jointHandle=client.sim.getObjectHandle('/Cuboid[0]/joint')
    client.jointAngle=client.sim.getJointPosition(client.jointHandle)
    client.sim.setJointTargetVelocity(client.jointHandle,360*math.pi/180)
    
    # enable the stepping mode on the client:
    client.client.setstepping(True)
    
    client.sim.startSimulation()
    
    moveToAngle(45*math.pi/180)
    moveToAngle(90*math.pi/180)
    moveToAngle(-89*math.pi/180) #no -90, to avoid passing below
    moveToAngle(0*math.pi/180)
    
    client.sim.stopSimulation()
