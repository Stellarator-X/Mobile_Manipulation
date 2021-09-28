import pybullet as p
import pybullet_data
import time
import math
import numpy as np
from math import *

from datetime import datetime
from datetime import datetime

from camera_for_base import get_image, cam_pose_to_world_pose
from sphere_fitting import sphereFit
from cam_ik import accurateIK


clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
	p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(enableConeFriction=0)

time_step = 0.001



fov = 50
aspect = 1
near = 0.01
far = 10
width = 200
height = 200
pi=3.14159265359
f_x = width/(2*tan(fov*pi/360))
f_y = height/(2*tan(fov*pi/360))

lastTime = time.time()
lastControlTime = time.time()

def pix_to_ee_frame(x_pix,y_pix,d):

	#pixel to cam frame
	Zc = d
	Xc = (x_pix - (width/2))*(d/(f_x))
	Yc = (y_pix - (height/2))*(d/(f_y))

	#cam to ee frame
	Xe = Zc
	Ye = -Xc
	Ze = -Yc
	return(Xe,Ye,Ze)


def mask_points(Sbuf,Dbuf,appleId):
    depthImg_buf = np.reshape(Dbuf, [width, height])
    D = far * near / (far - (far - near) * depthImg_buf)
    S = np.reshape(Sbuf,[width,height])
    Xl = []
    Yl = []
    Zl = []
    for i in range(width):
        for j in range(height):
            if S[j][i] == appleId:
                x,y,z = pix_to_ee_frame(i,j,D[j][i])
                Xl.append(x)
                Yl.append(y)
                Zl.append(z)
    return Xl,Yl,Zl

dpos = np.array((0.2,0.0,0.0))
def visual_servo_control(pos,ort):
    lmd = 5
    Vc = -lmd*((dpos-pos)+np.cross(pos,ort))
    Wc = -lmd*(ort)
    V = np.concatenate((Vc,Wc))
    return V

def relative_ee_pose_to_ee_world_pose1(robotId,eeTargetPos,eeTargetOrn):
    ee_link_state = p.getLinkState(robotId,linkIndex=7,computeForwardKinematics=1)
    ee_pos_W=ee_link_state[-2]
    ee_ort_W=ee_link_state[-1]
    return p.multiplyTransforms(ee_pos_W,ee_ort_W,eeTargetPos,eeTargetOrn)


p.resetSimulation()

p.setTimeStep(time_step)
p.setGravity(0.0, 0.0, -9.81)

cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
p.loadURDF("plane.urdf", [0, 0, 0.0])


gripper_left = p.addUserDebugParameter('Gripper_left', -0.5, 0.5, 0)
gripper_right = p.addUserDebugParameter('Gripper_right', -0.5, 0.5, 0)
targetVelocitySlider = p.addUserDebugParameter("wheelVelocity",-50,50,0)
maxForceSlider = p.addUserDebugParameter("maxForce",0,50,20)
steeringSlider = p.addUserDebugParameter("steering",-1,1,0)
front_left = p.addUserDebugParameter('front_left', 0, 20, 0)
front_right = p.addUserDebugParameter('front_right', 0, 20, 0)
rear_left = p.addUserDebugParameter('rear_left', 0, 20, 0)
rear_right = p.addUserDebugParameter('rear_right', 0, 20, 0)
# appleId = p.loadURDF("urdf/apple1/apple.urdf",[0.3,1.0,0.0],useFixedBase=True)
# appleId = p.loadURDF("urdf/apple1/apple.urdf",[0.5,0.50,0.0],useFixedBase=True)
# appleId = p.loadURDF("urdf/apple1/apple.urdf",[0.5,-1.0,0.0],useFixedBase=True)
# appleId = p.loadURDF("urdf/apple1/apple.urdf",[0.5,-2.0,0.0],useFixedBase=True)
# appleId = p.loadURDF("urdf/apple1/apple.urdf",[0.0,1.0,1],useFixedBase=False)


#importing kuka arm at
kukaCenter = [0,0,0.30250]
kukaOrientation = p.getQuaternionFromEuler([0,0,0])
scale = 0.7
kukaId = p.loadURDF("ur_description/urdf/arm_with_gripper.urdf", kukaCenter, kukaOrientation, globalScaling= scale,useFixedBase=False)

number_of_joints = p.getNumJoints(kukaId)
for joint_number in range(number_of_joints):
    info = p.getJointInfo(kukaId, joint_number)
    print(info[0], ": ", info[1])

# importing the base at 
baseCenter = [0.0,0.0,0.25]
baseOrientation = p.getQuaternionFromEuler([0,0,0])
baseId = p.loadURDF("ur_description/urdf/mobile_base_without_arm.urdf",baseCenter , cubeStartOrientation,useFixedBase=False)
number_of_joints = p.getNumJoints(baseId)
for joint_number in range(number_of_joints):
    info = p.getJointInfo(baseId, joint_number)
    print(info[0], ": ", info[1])
"""
# setting kuka initialy 0 
curr_joint_value = [0,0.5,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
p.setJointMotorControlArray( kukaId, range(16), p.POSITION_CONTROL, targetPositions=curr_joint_value)
"""

# puting kuka on baseId
cid = p.createConstraint(baseId,-1, kukaId, -1, p.JOINT_FIXED, [0.0, 0.0, 0.0], [0,0,0.10], [0.0, 0.0, 0.0])
number_of_joints = p.getNumJoints(cid)

print("cid "+str(p.getNumJoints(cid)))
print(cid)

for joint_number in range(number_of_joints):
    info = p.getJointInfo(cid, joint_number)
    print(info[0], ": ", info[1])

required_joints = [0,-1.9,1.9,-1.57,-1.57,0,0]
for i in range(1,7):
    p.resetJointState(bodyUniqueId=kukaId,
                            jointIndex=i,
                            targetValue=required_joints[i-1])
p.resetDebugVisualizerCamera( cameraDistance=2.2, cameraYaw=140, cameraPitch=-60, cameraTargetPosition=[0,0,0])


# activating real time simulation
useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)




t = 0.
#prevPose = [0, 0, 0]
#prevPose1 = [0, 0, 0]
#hasPrevPose = 0
useNullSpace = 0

useOrientation = 0
#If we set useSimulation=0, it sets the arm pose to be the IK result directly without using dynamic control.
#This can be used to test the IK result accuracy.
useSimulation = 1
useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)
#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
trailDuration = 15
#basepos = []
#basepos[1] = baseCenter
ang = 0


wheels = [0, 1, 2, 3]
wheelVelocities = [0, 0, 0, 0]
wheelDeltasTurn = [1, -1, 1, -1]
wheelDeltasFwd = [1, 1, 1, 1]

I,Dbuf,Sbuf = get_image(kukaId)

while (True):
	p.stepSimulation()
	time.sleep(1./240.)
	# try:
	# a, b, c = get_image(baseId)
	# print(type(a), " ", type(b), " ", type(c))
	# except:
		# print("can't get image")
	I,Dbuf,Sbuf = get_image(baseId)
	keys = p.getKeyboardEvents()
	shift = 0.01
	speed = 0.240
	for k in keys:
		if ord('s') in keys:
			p.saveWorld("state.py")
		

		if p.B3G_LEFT_ARROW in keys:
			for i in range(len(wheels)):
				wheelVelocities[i] = wheelVelocities[i] - speed * wheelDeltasTurn[i]
		if p.B3G_RIGHT_ARROW in keys:
			for i in range(len(wheels)):
				wheelVelocities[i] = wheelVelocities[i] + speed * wheelDeltasTurn[i]
		if p.B3G_UP_ARROW in keys:
			for i in range(len(wheels)):
				wheelVelocities[i] = wheelVelocities[i] + speed * wheelDeltasFwd[i]
		if p.B3G_DOWN_ARROW in keys:
			for i in range(len(wheels)):
				wheelVelocities[i] = wheelVelocities[i] - speed * wheelDeltasFwd[i]

	baseorn = p.getQuaternionFromEuler([0, 0, ang])
	for i in range(len(wheels)):
		p.setJointMotorControl2(baseId,wheels[i],p.VELOCITY_CONTROL,targetVelocity=wheelVelocities[i],force=1000)
		
	#p.resetBasePositionAndOrientation(baseId,basepos,baseorn)#[0,0,0,1])
	if (useRealTimeSimulation):
		t = time.time()  #(dt, micro) = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f').split('.')
		#t = (dt.second/60.)*2.*math.pi
	else:
		t = t + 0.1

	if (useSimulation and useRealTimeSimulation != 0):
		p.stepSimulation()

