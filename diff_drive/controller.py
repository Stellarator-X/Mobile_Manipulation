from math import atan2
import numpy as np 

class Pose():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class Agent():

    def __init__(self,wheel_radius = 0.034, base_length = 0.50, start_pose = Pose(0, 0, 0), V_MAX = 2, W_MAX = 3):
        self.base_length = base_length
        self.pose = start_pose
        self.wheel_radius = wheel_radius
        self.v_max = V_MAX
        self.w_max = W_MAX

    def update(self, left_vel, right_vel, dt):
        # Updates odometry
        v = self.wheel_radius*(left_vel+right_vel)/2
        w = self.wheel_radius*(right_vel - left_vel)/self.base_length
        self.pose.x += v*np.cos(self.pose.theta)*dt
        self.pose.y += v*np.sin(self.pose.theta)*dt
        self.pose.theta += w*dt

        return self.pose 

    def set_pose(self, pose):
        self.pose = pose

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

    def get_target_velocity(self, goal_pose, start_pose):
        # Returns v, w required as per unicycle model.
        epsilon = 0.01
        max_dist = self.get_distance2(start_pose, goal_pose)
        linear_distance = self.get_distance(goal_pose)
        target_angle = atan2(goal_pose.y-self.pose.y, goal_pose.x - self.pose.x)
        angular_offset = target_angle - self.pose.theta
        print(f"\rTarget_Angle : {angular_offset:.3f}, Target_Distance : {linear_distance:.3f}", end = "")

        # Step 1 - rotate to desired orientation
        if (abs(angular_offset) > epsilon):
            w = self.w_max*(angular_offset)/np.pi
            # v = self.v_max*(linear_distance)/max_dist
            v = 0
        elif (linear_distance > epsilon*50):
            w = 0
            v = np.cos(angular_offset)/abs(np.cos(angular_offset))*self.v_max*(linear_distance)/max_dist
        else : 
            w = v = 0
        return v, w
