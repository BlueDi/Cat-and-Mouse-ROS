#!/usr/bin/env python

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, MapMetaData
from math import atan2, pow, sqrt, pi
from tf.transformations import euler_from_quaternion
from go_to_goal import move_to_goal_ex, getDistance


map_metadata = MapMetaData(resolution=0.02, width=1000, height=1000)
robot_odom = Odometry()

corner = 1
factor = 100

mouse_linear_speed_unscaled = 100
mouse_linear_speed = mouse_linear_speed_unscaled #will be scaled by callbacks
mouse_angular_speed = pi / 180 * 90

drift_angle = pi / 180 * 30
slow_down_on_arrival = False

def odomCallback(odom_message):
    '''Odometry memory update'''
    global robot_odom
    robot_odom = odom_message

    x = robot_odom.pose.pose.position.x
    y = robot_odom.pose.pose.position.y

    updateCorner(x,y)

def updateCorner(x,y):
    global map_metadata
    global corner
    global factor

    resolution = map_metadata.resolution
    width = map_metadata.width * resolution
    height = map_metadata.height * resolution
    f = factor * resolution    

    if abs(x - f) < f:
        if abs(y - (height - f)) < f and corner == 1:
            corner = 2
        elif abs(y - f) < f and corner == 4:
            corner = 1
    elif abs(x - (width - f)) < f:
        if abs(y - (height - f)) < f and corner == 2:
            corner = 3
        elif abs(y - f) < f and corner == 3:
            corner = 4

def map_metadataCallback(map_metadata_message):
    '''Map metadata memory update'''
    global map_metadata
    global mouse_linear_speed_unscaled
    global mouse_linear_speed

    map_metadata = map_metadata_message

    resolution = map_metadata.resolution
    mouse_linear_speed = resolution * mouse_linear_speed_unscaled

def go_to_goal(x_goal, y_goal):
    '''PID Controller'''
    global robot_odom
    global mouse_linear_speed

    min_dist = mouse_linear_speed * 0.1

    position = robot_odom.pose.pose.position
    distance = getDistance(position.x, position.y, x_goal, y_goal)
    while distance > min_dist and not rospy.is_shutdown():
        velocity_message, distance = move_to_goal_ex(robot_odom, x_goal, y_goal, mouse_linear_speed, mouse_angular_speed, drift_angle, slow_down_on_arrival)
        velocity_publisher.publish(velocity_message)
        rate.sleep()
    
    velocity_message = Twist()
    velocity_publisher.publish(velocity_message)

def corner_movement():
    global map_metadata
    global corner
    global factor

    resolution = map_metadata.resolution
    width = map_metadata.width * resolution
    height = map_metadata.height * resolution
    f = factor * resolution    

    global mouse_linear_speed
    global mouse_angular_speed
    global drift_angle

    if corner == 1:
        go_to_goal(f, height - f)
    elif corner == 2:
        go_to_goal(width - f, height - f)
    elif corner == 3:
        go_to_goal(width - f, f)
    elif corner == 4:
        go_to_goal(f, f)

def subscribers(mouse_name):
    position_topic = '/' + mouse_name + '/odom'
    rospy.Subscriber(position_topic, Odometry, odomCallback)
    
    map_metadata_topic = '/map_metadata'
    rospy.Subscriber(map_metadata_topic, MapMetaData, map_metadataCallback)

if __name__ == '__main__':
    try:    
        mouse_name = rospy.get_param("mouse_name") or sys.argv[1]
    except Exception:
        mouse_name = 'mouse0'

    try:
        rospy.init_node(mouse_name + '_movement')
        rate = rospy.Rate(10)
        rate.sleep()
        
        cmd_vel_topic = '/' + mouse_name + '/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        subscribers(mouse_name)        

        while not rospy.is_shutdown():
            corner_movement()

    except rospy.ROSInterruptException:
        rospy.loginfo(mouse_name + ' terminated.')

