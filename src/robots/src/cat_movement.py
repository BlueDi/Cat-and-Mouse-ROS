#!/usr/bin/env python

import sys
from math import degrees, cos, sin
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, MapMetaData
from cat_mouse_world.msg import RobotsSpotted
from go_to_goal import move_to_goal
from angular_movement import *
from linear_movement import *


mouse_position = []
robot_odom = Odometry()


def odomCallback(odom_message):
    '''Odometry memory update'''
    global robot_odom
    robot_odom = odom_message


def sightCallback(sight_message):
    '''Mouse Sight memory update'''
    global mouse_position
    mouse_position = closest_mouse(sight_message)
    if mouse_position == []:
        rospy.loginfo('No mouse near')
    elif 0 <= mouse_position.dist < 0.1:
        rospy.loginfo('Caught a mouse')


def stop():
    mouse_position = []
    velocity_publisher.publish(Twist())


def go_to_goal(x_goal, y_goal):
    '''PID Controller'''
    global robot_odom
    distance = 1
    while distance > 0.1 and not rospy.is_shutdown():
        velocity_message, distance = move_to_goal(robot_odom, x_goal, y_goal)
        velocity_publisher.publish(velocity_message)
        rate.sleep()
    stop()


def subscribers():
    position_topic = '/' + CAT_NAME + '/odom'
    rospy.Subscriber(position_topic, Odometry, odomCallback)

    sight_topic = '/' + CAT_NAME + '/sight'
    rospy.Subscriber(sight_topic, RobotsSpotted, sightCallback)


def closest_mouse(sight_message):
    visible_mice = sight_message.robotsSpotted
    if len(visible_mice) > 0:
        closest_mouse = min(visible_mice, key=lambda x: x.dist)
    return closest_mouse if closest_mouse.dist > 0 else []


def cat_chase():
    '''Moves the cat in the direction of the closest mouse'''
    global mouse_position, robot_odom
    position = robot_odom.pose.pose.position
    x_goal = position.x + mouse_position.dist * cos(mouse_position.angle)
    y_goal = position.y + mouse_position.dist * sin(mouse_position.angle)
    go_to_goal(x_goal, y_goal)


def cat_roam():
    '''Make the cat roam the map to search for a mouse'''
    move(velocity_publisher, 3, 0.5, True)


def cat_movement():
    '''Control the cat movement'''
    global mouse_position
    saw_mouse = mouse_position != []
    if saw_mouse:
        rospy.loginfo('Saw a mouse!')
        cat_chase()
    else:
        cat_roam()


if __name__ == '__main__':
    try:
        CAT_NAME = rospy.get_param("cat_name")
    except KeyError:
        try:
            CAT_NAME = sys.argv[1]
        except IndexError:
            CAT_NAME = 'cat0'

    try:
        rospy.init_node(CAT_NAME + '_movement')
        rate = rospy.Rate(10)
        
        cmd_vel_topic = '/' + CAT_NAME + '/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=2)
        
        subscribers()

        while not rospy.is_shutdown():
            cat_movement()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo(CAT_NAME + ' terminated.')

