from math import atan2
import numpy as np
from numpy.core.numeric import True_ 
from utils import mask_points

class Pose():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class Agent():

    def __init__(self, wheel_radius = 0.034, base_length = 0.50, start_pose = Pose(0, 0, 0), V_MAX = 1.5, W_MAX = 5):
        self.base_length = base_length
        self.pose = start_pose
        self.wheel_radius = wheel_radius
        self.v_max = V_MAX
        self.max_depth = 3
        self.w_max = W_MAX

    def update(self, left_vel, right_vel, dt):
        # Updates odometry
        v = self.wheel_radius*(left_vel+right_vel)/2
        w = self.wheel_radius*(right_vel - left_vel)/self.base_length
        self.pose.x += v*np.cos(self.pose.theta)*dt
        self.pose.y += v*np.sin(self.pose.theta)*dt
        self.pose.theta += w*dt

        return self.pose 

    def estimate(self, start_pose, left_vel, right_vel, dt):
        v = self.wheel_radius*(left_vel+right_vel)/2
        w = self.wheel_radius*(right_vel - left_vel)/self.base_length
        new_pose = Pose(0, 0, 0)
        new_pose.x += v*np.cos(start_pose.theta)*dt
        new_pose.y += v*np.sin(start_pose.theta)*dt
        new_pose.theta += w*dt

        return new_pose


    def set_pose(self, pose):
        self.pose = pose

    def sgn(self, theta):
        cosine = np.cos(theta)
        if cosine < 0: return -1
        else: return 1

    def set_pose2(self, x, y, theta):
        self.pose.x = x
        self.pose.y = y
        self.pose.theta = theta

    def get_distance(self, goal_pose):
        return np.linalg.norm([self.pose.x-goal_pose.x, self.pose.y - goal_pose.y])

    def get_distance2(self, pose1, pose2):
        return np.linalg.norm([pose1.x-pose2.x, pose1.y - pose2.y])

    def get_wheel_velocities(self, v, w):
        v_r = (2*v + w*self.base_length)/(2*self.wheel_radius)
        v_l = (2*v - w*self.base_length)/(2*self.wheel_radius)
        return [v_l, v_r, v_l, v_r]

    def find_target(self, segmentation_mask, depth_mask, target_mask_id):
        """
        # Things we need to do
        1. Find whether the seg mask has the target id.
        2. Find the nearest point of target blob, if 1. returns true : Find argmin of mask*depth maybe.
        3. Find the distance of the blob from the base.
        """
        
        ids = np.unique(segmentation_mask)
        if target_mask_id in ids:
            X, Y, Z = mask_points(segmentation_mask, depth_mask, target_mask_id)
            idx = np.argmin(X)
            a, b = X[idx], Y[idx]
            # g_x = self.pose.x + a*np.cos(self.pose.theta) - b*np.sin(self.pose.theta)
            # g_y = self.pose.y + a*np.sin(self.pose.theta) + b*np.cos(self.pose.theta)
            depth = min(self.max_depth, np.linalg.norm([a, b]))
            alpha = atan2(b, a) + self.pose.theta
            g_x = self.pose.x + depth*np.cos(alpha)
            g_y = self.pose.y + depth*np.sin(alpha)
            
            return Pose(g_x, g_y, 0)
        else :
            return None
               


    def get_target_velocity(self, goal_pose, start_pose):
        # Returns v, w required as per unicycle model.
        epsilon = 0.08
        reached = False
        
        max_dist = self.get_distance2(start_pose, goal_pose)
        linear_distance = self.get_distance(goal_pose)
        target_angle = atan2(goal_pose.y-self.pose.y, goal_pose.x - self.pose.x)
        angular_offset = target_angle - self.pose.theta

        if angular_offset<-np.pi:
            angular_offset += 2*np.pi 
        elif angular_offset>np.pi:
            angular_offset -= 2*np.pi 

        assert (-np.pi <= angular_offset <= np.pi), f"Angular Offset  = {angular_offset}"

        print(f"\rTarget_Angle : {angular_offset:.3f}, Target_Distance : {linear_distance:.3f}", end = "")

        
        if (linear_distance<epsilon*10):
            v = w = 0
            reached = True
            print("Target reached.")
        elif (abs(angular_offset) > epsilon):
            w = self.w_max*(angular_offset)/np.pi
            v = self.sgn(angular_offset)*self.v_max/10
        elif (linear_distance > epsilon*10):
            w = self.sgn(angular_offset)*self.w_max/10
            v = self.sgn(angular_offset)*self.v_max*(linear_distance)/max_dist

        return v, w, reached
