#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import tf
import tf2_ros
import numpy as np 
import math
import utils.visualization_functions as visual
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class CollisionAvoidance:
    def __init__(self):
        
        self.move_goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.get_goal,queue_size=1)
        self.odom_sub = rospy.Subscriber("/red/odometry",Odometry,self.get_odom, queue_size=1)
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.get_scan, queue_size=1)
        self.sub_status = rospy.Subscriber('/red/tracker/status',String, self.get_status, queue_size=1)
        self.pub_goal = rospy.Publisher("/red/tracker/input_pose",PoseStamped,queue_size=1)
        
        self.publish_collision = rospy.Publisher('collision_point', Marker, queue_size=1)
        self.publish_radius    = rospy.Publisher('collision_radius', Marker, queue_size=1)
        self.publish_scape     = rospy.Publisher('scape_point', Marker, queue_size=1)
        self.publish_goal      = rospy.Publisher('goal_point', Marker, queue_size=1)
        self.max_height        = 30
        self.pose  = np.zeros(6)
        self.goal  = np.zeros(6)
        self.p0k   = np.zeros(3)
        self.p0k_1 = np.zeros(3)
        self.obs  = np.zeros(3)
        self.obs_w  = np.zeros(3)
        self.min = 0 
        self.ang = 0
        self.collision = False
        self.moving = False
        self.status = False
        self.radius = 0 
        self.velocity = 0 
        self.count = 1
        self.tempo = 0 
        self.scape = np.zeros(3)
        self.scape_w = np.zeros(3)
        self.prev_goal = np.zeros(6)
        self.block = 0
        self.confirm = 0 
        print('Please set a goal point from Rviz')


    def get_odom(self,odom):

        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        self.pose[0] = odom.pose.pose.position.x
        self.pose[1] = odom.pose.pose.position.y
        self.pose[2] = odom.pose.pose.position.z
        self.pose[3] = yaw
        
        if self.count ==1:
            self.p0k_1 = 0
            self.goal = np.zeros(6)
            self.count =2

        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.x
        self.p0k = np.linalg.norm(self.pose[:2]-self.obs_w[:2])
        
        self.velocity = (self.p0k - self.p0k_1)/0.01

        self.p0k_1 = self.p0k
        self.radius = 4 + (1+3*np.tanh(-0.5*self.velocity)) 
        dist_to_goal = np.linalg.norm(self.pose[:2]-self.prev_goal[:2])
        visual.publish_scape(self.prev_goal, self.publish_goal, frame ='world' )
        if dist_to_goal<1 and self.confirm == 0:
            print(f'Goal reached at {self.pose[2]:.2f}m height\n')
            self.confirm = 1
        

    def get_goal(self,goal):
        self.confirm = 0
        diff = self.goal[:2]-self.pose[:2]
        angle = np.arctan2(diff[1],diff[0]) + 1.57
        self.goal = np.array([goal.pose.position.x,goal.pose.position.y,self.pose[2],angle,0,0])

        print(f"Goal -> ({self.goal[0]:.2f},{self.goal[1]:.2f},{self.goal[2]:.2f}) received")
        self.prev_goal = self.goal
        self.send_pose_commnd(self.goal)
        
            

    def get_scan(self,scan):
        scans = []
        angles = []
        for i,r in enumerate(scan.ranges):
            if r>0.5:
                scans.append(r)
                angles.append(scan.angle_min + i*scan.angle_increment)
                
        ranges = np.array(scans)
        self.min = np.min(ranges)

        # # rges = [r for r in scan.ranges if r>0.5]
        
        
        
        
        
        # ranges = np.array(rges)
        # rangemin = np.min(ranges)
        # self.min = rangemin
        indexmin = np.argmin(ranges)
        self.ang = angles[indexmin]

        if self.min<self.radius:
            self.collision=True
            self.block = 1


        # VERTICAL AND HORIZONTAL SCAPE STRATEGY
        if self.collision == True:
            
            if self.tempo == 0:
                self.goal = self.scape_w
                if self.pose[2] < 30:
                    self.goal[2] = self.goal[2] + 10
                if self.pose[2] > 30:
                    print('Maximum height reached')

                self.send_pose_commnd(self.goal)
                print(f'Obstacle detected at {np.linalg.norm(self.obs[:2]):.2f}m from me!')
                print(f'Moving to a safe position')
                self.tempo = 1
        
        
        # angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)

        # indexmin = np.argmin(ranges)
        
        Trw = np.array([[np.cos(self.pose[3]),-np.sin(self.pose[3]),0,self.pose[0]],
                        [np.sin(self.pose[3]),np.cos(self.pose[3]),0,self.pose[1]],
                        [0,0,1,self.pose[2]],
                        [0,0,0,1]])
        
        if not math.isinf(self.min):
            self.obs = np.array([self.min*np.cos(self.ang), self.min*np.sin(self.ang),-0.1,1])
            self.obs_w = Trw@self.obs                                                                           #Closest obstacle in World Frame
            alpha = np.arctan2(self.obs[1],self.obs[0])
            
            
            if self.obs[0]>0 and self.obs[1]>0:
                gamma = -np.deg2rad(90)+alpha
                scape = np.array([self.radius*np.cos(gamma), self.radius*np.sin(gamma)])
            
            if self.obs[0]<0 and self.obs[1]>0:
                gamma = np.deg2rad(90) + alpha
                scape = np.array([self.radius*np.cos(gamma), self.radius*np.sin(gamma)])
            
            if self.obs[0]<0 and self.obs[1]<0:
                gamma = -np.deg2rad(90) + alpha
                scape = np.array([self.radius*np.cos(gamma), self.radius*np.sin(gamma)])
            
            if self.obs[0]>0 and self.obs[1]<0:
                gamma = np.deg2rad(90) + alpha
                scape = np.array([self.radius*np.cos(gamma), self.radius*np.sin(gamma)])
            
            self.scape = np.array([scape[0],scape[1], -0.1,gamma])
            self.scape_w = Trw@np.array([self.scape[0],self.scape[1],-0.1,1])
            

            visual.publish_point(self.obs, self.publish_collision,scale= 0.3,frame ='red/base_link_inertia',color = [1, 0, 0, 1])
            visual.publish_point(self.scape, self.publish_scape, scale = 0.3, frame ='red/base_link_inertia',color = [1, 1, 0, 1])

        visual.publish_radius(self.pose, self.publish_radius,scale=self.radius,frame="world")
        
        if math.isinf(self.min):
            visual.publish_point(self.obs, self.publish_collision,scale=0,frame ='red/base_link',color = [1, 0, 0, 1])
            visual.publish_point(self.scape, self.publish_scape, scale = 0, frame ='red/base_link',color = [1, 1, 0, 1])


        if np.all(ranges>self.radius):
            self.collision = False
            
        if self.block == 1 and self.collision == False and self.tempo == 0:
            self.send_pose_commnd(self.prev_goal)
            print('Resuming mission..')
            print(f"Goal -> ({self.prev_goal[0]:.2f},{self.prev_goal[1]:.2f},{self.prev_goal[2]:.2f}) received")
            self.block = 2
              
                
    def get_status(self,msg):
        if msg.data == "ACCEPT":
            self.status = False
            self.tempo = 0
            self.prev_goal[2] = self.pose[2]
        if msg.data == "ACTIVE":
            self.status = True 
      

    def send_pose_commnd(self,goal):

        w = 1.57   
        cmd = PoseStamped()
        cmd.pose.position.x = goal[0]*np.cos(w) - goal[1]*np.sin(w)
        cmd.pose.position.y = goal[0]*np.sin(w) + goal[1]*np.cos(w)
        cmd.pose.position.z = goal[2]
        self.pub_goal.publish(cmd)

if __name__ == '__main__':
    rospy.init_node("collision_avoidance")
    collision_avoidance = CollisionAvoidance()
    rospy.spin()