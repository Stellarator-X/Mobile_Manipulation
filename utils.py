import numpy as np 
from math import *

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