import pybullet as p
import pybullet_data
import time
import math
import numpy as np
from math import *

from datetime import datetime
from datetime import datetime

from camera import get_image, cam_pose_to_world_pose
from sphere_fitting import sphereFit
from cam_ik import accurateIK
from vs_rotation_conversion import eulerAnglesToRotationMatrix

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

roboEndEffectorIndex = 6

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

def getJointStates(robot):
  joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
  joint_positions = [state[0] for state in joint_states]
  joint_velocities = [state[1] for state in joint_states]
  joint_torques = [state[3] for state in joint_states]
  return joint_positions, joint_velocities, joint_torques

def getMotorJointStates(robot):
  joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
  joint_infos = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
  joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
  joint_states = joint_states[0:6]
  joint_positions = [state[0] for state in joint_states]
  joint_velocities = [state[1] for state in joint_states]
  joint_torques = [state[3] for state in joint_states]
  return joint_positions, joint_velocities, joint_torques

def run_ee_link_relative(roboId,eevel):
  pos, vel, torq = getJointStates(roboId)
  mpos, mvel, mtorq = getMotorJointStates(roboId)

  result = p.getLinkState(roboId,
                          roboEndEffectorIndex,
                          computeLinkVelocity=1,
                          computeForwardKinematics=1)
  link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
  ee_link_state = p.getLinkState(roboId,linkIndex=roboEndEffectorIndex,computeForwardKinematics=1)
  ee_pos_W=ee_link_state[-2]
  ee_ort_W=ee_link_state[-1]
  bRe = eulerAnglesToRotationMatrix(p.getEulerFromQuaternion(ee_ort_W))
  eRb = np.transpose(bRe)
  jR = np.zeros((6,6))
  jR[0:3,0:3] = eRb
  jR[3:6,3:6] = eRb

  zero_vec = [0.0] * len(mpos)
  jac_t, jac_r = p.calculateJacobian(roboId, roboEndEffectorIndex, com_trn, mpos, zero_vec, zero_vec)
 
  J = jac_t + jac_r  # Jacobian in base frame
  J = np.array(J)[:,6:12]
  Je = np.matmul(jR,J) # Jacobian in ee_link frame
  Ja = np.linalg.inv(np.array(Je))
  Q = np.matmul(Ja,eevel)
  print(Q)
  maxForce = 500
  for i in range(0,6):
    p.setJointMotorControl2(bodyUniqueId=roboId, 
    jointIndex=i, 
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity = Q[i], #check if mapping is proper
    force = maxForce)



dpos = np.array((0.2,0.0,0.0))
def visual_servo_control(pos,ort):
    lmd = 5
    Vc = -lmd*((dpos-pos)+np.cross(pos,ort))
    Wc = -lmd*(ort)
    V = np.concatenate((Vc,Wc))
    return V

if __name__=="__main__":
	p.resetSimulation()

	p.setTimeStep(time_step)
	p.setGravity(0.0, 0.0, -9.81)

	cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
	p.loadURDF("plane.urdf", [0, 0, 0.0])

	appleId = p.loadURDF("urdf/apple1/apple.urdf",[0.6,0.0,0.1],useFixedBase=False)
	
	#importing robo arm at
	roboCenter = [0,0,0.30250]
	roboOrientation = p.getQuaternionFromEuler([0,0,0])
	scale = 0.7
	roboId = p.loadURDF("ur_description/urdf/arm_without_gripper.urdf", roboCenter, roboOrientation)#, globalScaling= scale,useFixedBase=False)
	# input()
	print('joints info in UR')
	number_of_joints = p.getNumJoints(roboId)
	for joint_number in range(number_of_joints):
	    info = p.getJointInfo(roboId, joint_number)
	    print(info[0], ": ", info[1])

	# importing the base at 
	baseCenter = [0.0,0.0,0.25]
	baseOrientation = p.getQuaternionFromEuler([0,0,0])
	baseId = p.loadURDF("ur_description/urdf/mobile_base_without_arm.urdf",baseCenter , cubeStartOrientation,useFixedBase=False)
	print('joint info in mobile base')
	number_of_joints = p.getNumJoints(baseId)
	for joint_number in range(number_of_joints):
	    info = p.getJointInfo(baseId, joint_number)
	    print(info[0], ": ", info[1])

	# puting robo on baseId
	cid = p.createConstraint(baseId,-1, roboId, -1, p.JOINT_FIXED, [0.0, 0.0, 0.0], [0,0,0.10], [0.0, 0.0, 0.0])

	required_joints = [0,-1.4,1.4,-1.57,-1.57,0,0]
	for i in range(0,6):
		p.resetJointState(bodyUniqueId=roboId,
							jointIndex=i,
							targetValue=required_joints[i])
	p.resetDebugVisualizerCamera( cameraDistance=2.2, cameraYaw=140, cameraPitch=-60, cameraTargetPosition=[0,0,0])

	for i in range (1000):
	    p.stepSimulation()
	# activating real time simulation
	useRealTimeSimulation = 1
	p.setRealTimeSimulation(useRealTimeSimulation)

	I,Dbuf,Sbuf = get_image(roboId)
	sx,sy,sz = mask_points(Sbuf,Dbuf,appleId)
	r,cx,cy,cz = sphereFit(sx,sy,sz)
	pos = np.array((cx,cy,cz))[:,0]
	ort = np.array((0.0,0.0,0.0))

	Ve = visual_servo_control(pos,ort)

	for i in range (10000):
	    I,Dbuf,Sbuf = get_image(roboId)
	    sx,sy,sz = mask_points(Sbuf,Dbuf,appleId)
	    r,cx,cy,cz = sphereFit(sx,sy,sz)
	    pos = np.array((cx,cy,cz))[:,0]
	    error = np.sqrt(np.mean((pos-dpos)**2))
	    if error < 0.00045:
	      break
	    print(pos,error)
	    ort = np.array((0.0,0.0,0.0))
	    Ve = visual_servo_control(pos,ort)
	    run_ee_link_relative(roboId,0.1*Ve)
	    
	c = input('press enter to end')
