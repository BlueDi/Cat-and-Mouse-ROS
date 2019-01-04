#!/usr/bin/env python

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, MapMetaData
from math import atan2, pow, sqrt, pi
from tf.transformations import euler_from_quaternion


map_metadata = MapMetaData(resolution=0.02, width=1000, height=1000)
robot_odom = Odometry()

corner = 1
factor = 100

mouse_linear_speed_unscaled = 100
mouse_linear_speed = mouse_linear_speed_unscaled #will be scaled by callbacks
mouse_angular_speed = pi / 180 * 90

drift_angle = pi / 180 * 30
slow_down_on_arrival = True

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
        velocity_message, distance = move_to_goal(robot_odom, x_goal, y_goal)
        velocity_publisher.publish(velocity_message)
        rate.sleep()
    
    velocity_message = Twist()
    velocity_publisher.publish(velocity_message)

def getDistance(x1,y1,x2,y2):
    '''Euclidean distance between two points'''
    return sqrt(pow((x1-x2),2) + pow((y1-y2),2))

def move_to_goal(robot_odom, goal_x, goal_y):
    global mouse_linear_speed
    global mouse_angular_speed
    global drift_angle

    velocity_message = Twist()
    position = robot_odom.pose.pose.position

    orientation = robot_odom.pose.pose.orientation
    orientation_array = [orientation.x, orientation.y, orientation.z, orientation.w]
    roll, pitch, yaw = euler_from_quaternion(orientation_array)

    bot_x = position.x
    bot_y = position.y
    
    if(yaw >= 0):
        bot_angle = yaw
    else:
        bot_angle = 2 * pi + yaw

    linear_speed = mouse_linear_speed
    angular_speed = mouse_angular_speed

    distance = abs(getDistance(bot_x, bot_y, goal_x, goal_y))  
    if distance < linear_speed and slow_down_on_arrival:
        linear_speed = distance

    angle_aux = atan2(goal_y - position.y, goal_x - position.x)
    if angle_aux >= 0:
        target_angle = angle_aux
    else:
        target_angle = 2 * pi + angle_aux

    angle_diff = 0
    if target_angle > bot_angle:
        s1 = abs(target_angle - bot_angle)
        s2 = abs(bot_angle - (target_angle - 2 * pi))

        if s1 <= s2:
            angle_diff = s1
        else:
            angle_diff = -s2
    else:
        s1 = abs(bot_angle - target_angle)
        s2 = abs(target_angle - (bot_angle - 2 * pi))

        if s1 <= s2:
            angle_diff = -s1
        else:
            angle_diff = s2
    
    if abs(angle_diff) > angular_speed:
        if angle_diff > 0:
            angular_speed = angular_speed
        else:
            angular_speed = -angular_speed
    else:
        angular_speed = angle_diff
    
    if abs(angular_speed) < 0.005:
        angular_speed = 0
    if abs(linear_speed) < 0.005:
        linear_speed = 0

    if abs(angular_speed) < drift_angle:
        velocity_message.linear.x = linear_speed
    else:
        velocity_message.linear.x = 0
    velocity_message.angular.z = angular_speed

    print("Robot Angle: ", bot_angle, " Target Angle: ", target_angle, "Angle_Diff", angle_diff)
    print("Robot Speed: ", velocity_message.linear.x, " Robot Angular: ", velocity_message.angular.z)
    return velocity_message, distance


def corner_movement():
    global map_metadata
    global corner
    global factor

    resolution = map_metadata.resolution
    width = map_metadata.width * resolution
    height = map_metadata.height * resolution
    f = factor * resolution    

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

