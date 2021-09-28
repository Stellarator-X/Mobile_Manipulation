import pybullet as p 
import pybullet_data
import time
import math 
import numpy as np 

from datetime import datetime

from camera import get_image, cam_pose_to_world_pose
from sphere_fitting import sphereFit
from cam_ik import accurateIK
from camera_for_base import get_image
from diff_drive.controller import Agent, Pose

client_id = p.connect(p.GUI)
p.setPhysicsEngineParameter(enableConeFriction=0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

target_id = p.loadURDF("cube.urdf", [4., 4., 0.5])
try: p.loadSDF("treeworld.sdf")
except: print("NO")

while True:
    continue