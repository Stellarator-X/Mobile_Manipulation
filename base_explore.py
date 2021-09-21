import pybullet as p
import pybullet_data
import time
import math
import numpy as np
from math import *

from datetime import datetime

from camera import get_image, cam_pose_to_world_pose
from sphere_fitting import sphereFit
from cam_ik import accurateIK
from camera_for_base import get_image
from diff_drive.controller import Agent, Pose

import matplotlib.pyplot as plt 

fov = 50
aspect = 1
near = 0.01
far = 10
width = 200
height = 200
pi=3.14159265359
f_x = width/(2*tan(fov*pi/360))
f_y = height/(2*tan(fov*pi/360))

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

if __name__ == "__main__":
	
	if True: # Launch Simulation Server(s)
		client_id = p.connect(p.SHARED_MEMORY)
		if (client_id < 0):
			print("connecting to GUI")
			p.connect(p.GUI)
		p.setAdditionalSearchPath(pybullet_data.getDataPath())
		p.setPhysicsEngineParameter(enableConeFriction=0)
		time_step = 0.001

		useRealTimeSimulation = 1
		p.setRealTimeSimulation(useRealTimeSimulation)

		lastTime = time.time()
		lastControlTime = time.time()

		p.resetSimulation()
		p.setTimeStep(time_step)
		p.setGravity(0.0, 0.0, -9.81)

	if True: # Set Camera Parameters

		fov = 50
		aspect = 1
		near = 0.01
		far = 10
		width = 200
		height = 200
		pi=3.14159265359
		f_x = width/(2*tan(fov*pi/360))
		f_y = height/(2*tan(fov*pi/360))

	if True: # Set up the environment
		import env
		cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])

		cubeStartPos = [5,0,-21.5]
		treeStartOrientation = p.getQuaternionFromEuler([np.pi/2,0,0])
		# treeId = p.loadURDF("arbre/arbre.urdf",cubeStartPos, treeStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE)
		target_IDs = []
		target_IDs.append(p.loadURDF("arbre/arbre.urdf",[7., 7., -21.5], treeStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE))
		target_IDs.append(p.loadURDF("arbre/arbre.urdf",[-5., 0., -21.5], treeStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE))
		target_IDs.append(p.loadURDF("arbre/arbre.urdf",[7., -7., -21.5], treeStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE))
		
		
		# target_id = p.loadURDF("cube.urdf", [4., 4., 0.5])
		# target_id = p.loadURDF("cube.urdf", [-1., 0., 0.5])
		# target_id = p.loadURDF("cube.urdf", [4., -4., 0.5])
		

		baseCenter = [0.0,0.0,0.25]
		baseOrientation = p.getQuaternionFromEuler([0,0,0])
		baseId = p.loadURDF("ur_description/urdf/mobile_base_without_arm.urdf",baseCenter , cubeStartOrientation,useFixedBase=False)
		
		# try:
		# 	pineId = p.loadSDF("robot_description/worlds/obstacles.world")
		# except:
		# 	print("Can't import SDF")
		# 	input()

		number_of_joints = p.getNumJoints(baseId)
		for joint_number in range(number_of_joints):
			info = p.getJointInfo(baseId, joint_number)
			print(info[0], ": ", info[1])

		base_length = abs(p.getJointInfo(baseId, 0)[-3][1] - p.getJointInfo(baseId, 1)[-3][1])
		print(f"Base Length = {base_length:.2f}")
		print(f"Link coordinates : {p.getJointInfo(baseId, 0)[-3]}")	
	
	
	
	GOAL_IDS = [1, 2, 3]
	base_pose = Pose(0, 0, 0)
	base_bot = Agent(start_pose = base_pose)
	wheels = [0, 1, 2, 3]
	wheelVelocities = [0, 0, 0, 0]
	prev_t = time.time()
	est_pose = Pose(0, 0, 0)

	err_d = []
	err_theta = []

	for target_id in GOAL_IDS : 
		print()
		target = None
		prev_t = time.time()
		p.stepSimulation()
		pose, orientation  = p.getBasePositionAndOrientation(baseId)
		while target == None:
			time.sleep(1/240.)
			I,Dbuf,Sbuf = get_image(baseId)
			
			pose, orientation  = p.getBasePositionAndOrientation(baseId)
			orientation = p.getEulerFromQuaternion(orientation)
			_, _, theta = orientation
			base_pose.x, base_pose.y, base_pose.theta = pose[0], pose[1], theta 
			base_bot.set_pose(base_pose)

			target = base_bot.find_target(Sbuf, Dbuf, target_id)
			if target is not None: break

			wheelVelocities = base_bot.get_wheel_velocities(0, base_bot.w_max/2)

			for i in range(len(wheels)):
				p.setJointMotorControl2(baseId,wheels[i],p.VELOCITY_CONTROL,targetVelocity=wheelVelocities[i],force=100)

			if (useRealTimeSimulation):
				t = time.time() 
			else:
				t += time_step

			p.stepSimulation()
		print(f"Target Found at ({target.x:.3f}, {target.y:.3f})")
		GOAL_POSE = target
		START_POSE = base_pose
		
		wheels = [0, 1, 2, 3]
		wheelVelocities = [0, 0, 0, 0]

		base_bot = Agent(start_pose = base_pose)
		
		prev_t = time.time()
		p.stepSimulation()
		pose, orientation  = p.getBasePositionAndOrientation(baseId)
		# prev_x = pose[0]
		# velocities = []
		done = False
		while not done: # Simulation Loop
			time.sleep(1/240.)	
			I,Dbuf,Sbuf = get_image(baseId)
			# print(np.unique(Dbuf))
			# print(np.max(Dbuf), np.min(Dbuf))
			
			target = base_bot.find_target(Sbuf, Dbuf, target_id)
			if target!=None:
				GOAL_POSE = target
				# print("target at di")
			# Getting odometry 
			# TODO @stellarator-x : Change all make all odometry simulator independent. 
			pose, orientation  = p.getBasePositionAndOrientation(baseId)
			orientation = p.getEulerFromQuaternion(orientation)
			_, _, theta = orientation
			base_pose.x, base_pose.y, base_pose.theta = pose[0], pose[1], theta 
			base_bot.set_pose(base_pose)

			target_v, target_w, done = base_bot.get_target_velocity(GOAL_POSE, START_POSE, )
			wheelVelocities = base_bot.get_wheel_velocities(target_v, target_w)



			for i in range(len(wheels)):	
				p.setJointMotorControl2(baseId,wheels[i],p.VELOCITY_CONTROL,targetVelocity=wheelVelocities[i],force=100)

			if (useRealTimeSimulation):
				t = time.time() 
				dt = t - prev_t
			else:
				t += time_step
				dt = time_step

			est_pose = base_bot.estimate(est_pose, target_v, target_w, dt)

			err_d.append(np.linalg.norm([est_pose.x - base_pose.x, est_pose.y - base_pose.y]))
			err_theta.append(np.cos(est_pose.theta - base_pose.theta)); 

			p.stepSimulation()

	time.sleep(3)

	_, (a1, a2) = plt.subplots(1, 2)
	a1.plot(err_d, label = "err_distance")
	a1.legend()
	a2.plot(err_theta, label="cos_err_theta")
	a2.legend()
	plt.legend()
	plt.show()

