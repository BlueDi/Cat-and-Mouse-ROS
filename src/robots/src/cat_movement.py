#!/usr/bin/env python

import sys
from math import degrees, cos, sin
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, MapMetaData
from cat_mouse_world.msg import RoboSpotted
from go_to_goal import move_to_goal
from angular_movement import *
from linear_movement import *


mouse_position = RoboSpotted()
robot_odom = Odometry()


def odomCallback(odom_message):
    '''Odometry memory update'''
    global robot_odom
    robot_odom = odom_message


def sightCallback(sight_message):
    '''Mouse Sight memory update'''
    global mouse_position
    mouse_position = sight_message
    cat_movement()


def stop():
    mouse_position = RoboSpotted()
    velocity_publisher.publish(Twist())


def go_to_goal(x_goal, y_goal):
    '''PID Controller'''
    global robot_odom
    distance = 0.2 + 1
    while distance > 0.2 and not rospy.is_shutdown():
        velocity_message, distance = move_to_goal(robot_odom, x_goal, y_goal)
        velocity_publisher.publish(velocity_message)
        rate.sleep()
    stop()


def subscribers(cat_name):
    position_topic = '/' + cat_name + '/odom'
    rospy.Subscriber(position_topic, Odometry, odomCallback)

    sight_topic = '/' + cat_name + '/sight'
    rospy.Subscriber(sight_topic, RoboSpotted, sightCallback)


def cat_movement():
    global mouse_position, robot_odom
    saw_mouse = mouse_position != RoboSpotted()
    if saw_mouse:
        position = robot_odom.pose.pose.position
        x_goal = position.x + mouse_position.dist * cos(mouse_position.angle)
        y_goal = position.y + mouse_position.dist * sin(mouse_position.angle)
        print x_goal, y_goal
        go_to_goal(x_goal, y_goal)
    else:
        move(velocity_publisher, 3, 0.5, True)


if __name__ == '__main__':
    try:    
        cat_name = rospy.get_param("cat_name") or sys.argv[1]
    except Exception:
        cat_name = 'cat0'

    try:
        rospy.init_node(cat_name + '_movement')
        rate = rospy.Rate(10)
        
        cmd_vel_topic = '/' + cat_name + '/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        subscribers(cat_name)

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo(cat_name + ' terminated.')

