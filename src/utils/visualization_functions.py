#!/usr/bin/python

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import ColorRGBA

def publish_corners(corners, pub, frame='world', ns='none', time=None, color=(1, 0, 0, 1), marker_id=0, scale=0.1):
    crn = Marker()
    crn.header.frame_id = frame
    crn.header.stamp = rospy.Time.now()
    crn.type = crn.SPHERE_LIST
    crn.action = crn.ADD
    crn.pose.position.x = 0.0
    crn.pose.position.y = 0.0
    crn.pose.position.z = 0.0
    crn.pose.orientation.x = 0.0
    crn.pose.orientation.y = 0.0
    crn.pose.orientation.z = 0.0
    crn.pose.orientation.w = 1.0
    crn.scale.x = scale
    crn.scale.y = scale
    crn.scale.z = 0.01
    c = ColorRGBA(color[0], color[1], color[2], color[3])
    crn.color = c
    for i in range(len(corners)):
        crn.points.append(Point(corners[i][0], corners[i][1], 0))
    pub.publish(crn)

def publish_circles(corners, pub, frame='world', ns='none', time=None, color=(1, 0, 0, 1), marker_id=0, scale=1):
    crn = Marker()
    crn.header.frame_id = frame
    crn.header.stamp = rospy.Time.now()
    crn.type = Marker.CYLINDER
    crn.action = crn.ADD
    crn.pose.position.x = 0.0
    crn.pose.position.y = 0.0
    crn.pose.position.z = 0.0
    crn.pose.orientation.x = 0.0
    crn.pose.orientation.y = 0.0
    crn.pose.orientation.z = 0.0
    crn.pose.orientation.w = 1.0
    crn.scale.x = scale
    crn.scale.y = scale
    crn.scale.z = 0.001
    c = ColorRGBA(color[0], color[1], color[2], color[3])
    crn.color = c
    for i in range(len(corners)):
        crn.points.append(Point(corners[i][0], corners[i][1], 0))
    pub.publish(crn)

def publish_arrays(arrays, pub, frame='world', ns='none', time=None, color=(0, 1, 0), marker_id=0, scale=0.1):
    markerArray = MarkerArray()
    for index in range(len(arrays)):
        msg_ellipse = Marker()
        msg_ellipse.id = marker_id
        msg_ellipse.header.frame_id = frame
        msg_ellipse.header.stamp = rospy.Time.now()
        msg_ellipse.type = Marker.CYLINDER
        msg_ellipse.pose.position.x = arrays[index][0]
        msg_ellipse.pose.position.y = arrays[index][1]
        msg_ellipse.pose.position.z = -0.1
        msg_ellipse.pose.orientation.x = 0.0
        msg_ellipse.pose.orientation.y = 0.0
        msg_ellipse.pose.orientation.z = 0.0
        msg_ellipse.pose.orientation.w = 1.0
        msg_ellipse.scale.x = 1 
        msg_ellipse.scale.y = 1
        msg_ellipse.scale.z = 0.01
        msg_ellipse.color.a = 0.6
        msg_ellipse.color.r = 0.0
        msg_ellipse.color.g = 0.7
        msg_ellipse.color.b = 0.7
        markerArray.markers.append(msg_ellipse)
        marker_id = marker_id + 1
    pub.publish(markerArray)

# publish a single agent
def publish_agent(pose, pub, scale):
    ellipse = Marker()
    ellipse.header.frame_id = "world"
    ellipse.header.stamp = rospy.Time.now()
    ellipse.type = Marker.SPHERE
    ellipse.pose.position.x = pose[0]
    ellipse.pose.position.y = pose[1]
    ellipse.pose.position.z = pose[2]
    ellipse.pose.orientation.x = 0
    ellipse.pose.orientation.y = 0
    ellipse.pose.orientation.z = 0
    ellipse.pose.orientation.w = 1
    ellipse.scale.x = scale*2
    ellipse.scale.y = scale*2
    ellipse.scale.z = 0.01
    ellipse.color.a = 0.3
    ellipse.color.r = 0.0
    ellipse.color.g = 1.0
    ellipse.color.b = 1.0
    pub.publish(ellipse)

def publish_goal(pose, pub, frame='world', ns='none', time=None, marker_id=0, color=[1, 0, 0, 1]):
    msg = Marker()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame
    msg.type = msg.ARROW
    msg.action = msg.ADD
    msg.pose.position.x = pose[0]
    msg.pose.position.y = pose[1]
    msg.pose.position.z = pose[2]
    quat = quaternion_from_euler(0, 0, pose[3])
    msg.pose.orientation.x = quat[0]
    msg.pose.orientation.y = quat[1]
    msg.pose.orientation.z = quat[2]
    msg.pose.orientation.w = quat[3]
    msg.scale.x = 0.2
    msg.scale.y = 0.2
    msg.scale.z = 0.2
    c = ColorRGBA(color[0], color[1], color[2], color[3])
    msg.color = c
    pub.publish(msg)


def publish_point(pose, pub, scale,frame,color=[1, 0, 0, 1]):
    ellipse = Marker()
    ellipse.header.frame_id = frame
    ellipse.header.stamp = rospy.Time.now()
    ellipse.type = Marker.SPHERE
    ellipse.pose.position.x = pose[0]
    ellipse.pose.position.y = pose[1]
    ellipse.pose.position.z = pose[2]
    ellipse.pose.orientation.x = 0
    ellipse.pose.orientation.y = 0
    ellipse.pose.orientation.z = 0
    ellipse.pose.orientation.w = 1
    ellipse.scale.x = scale
    ellipse.scale.y = scale
    ellipse.scale.z = scale
    # ellipse.color.a = 1
    # ellipse.color.r = 242/255
    # ellipse.color.g = 146/255
    # ellipse.color.b = 29/255
    c = ColorRGBA(color[0], color[1], color[2], color[3])
    ellipse.color =c
    pub.publish(ellipse)

def publish_radius(pose, pub, scale,frame):
    ellipse = Marker()
    ellipse.header.frame_id = frame
    ellipse.header.stamp = rospy.Time.now()
    ellipse.type = Marker.SPHERE
    ellipse.pose.position.x = pose[0]
    ellipse.pose.position.y = pose[1]
    ellipse.pose.position.z = pose[2]-0.1
    ellipse.pose.orientation.x = 0
    ellipse.pose.orientation.y = 0
    ellipse.pose.orientation.z = 0
    ellipse.pose.orientation.w = 1
    ellipse.scale.x = scale*2
    ellipse.scale.y = scale*2
    ellipse.scale.z = 0.01
    ellipse.color.a = 0.3
    ellipse.color.r = 0
    ellipse.color.g = 1
    ellipse.color.b = 1
    pub.publish(ellipse)


def publish_scape(pose, pub, frame,ns='none', color=[0, 1, 1, 1]):
    msg = Marker()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame
    msg.type = msg.ARROW
    msg.action = msg.ADD
    msg.pose.position.x = pose[0]
    msg.pose.position.y = pose[1]
    msg.pose.position.z = pose[2]+0.75
    quat = quaternion_from_euler(0, np.deg2rad(90),0)
    msg.pose.orientation.x = quat[0]
    msg.pose.orientation.y = quat[1]
    msg.pose.orientation.z = quat[2]
    msg.pose.orientation.w = quat[3]
    msg.scale.x = 0.4
    msg.scale.y = 0.2
    msg.scale.z = 0.2
    c = ColorRGBA(color[0], color[1], color[2], color[3])
    msg.color = c
    pub.publish(msg)