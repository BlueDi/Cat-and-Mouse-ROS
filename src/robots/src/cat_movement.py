#!/usr/bin/env python

import sys
from math import degrees, cos, sin, pi
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, MapMetaData
from cat_mouse_world.msg import RobotsSpotted
from go_to_goal import move_to_goal_ex, getDistance
from angular_movement import *
from linear_movement import *
import time


mouse_position = []
robot_odom = Odometry()
map_metadata = MapMetaData(resolution=0.02, width=1000, height=1000)

cat_linear_speed_unscaled = 125
cat_linear_speed = cat_linear_speed_unscaled #will be scaled by callbacks
cat_angular_speed = pi / 180 * 180

drift_angle = pi / 180 * 45
slow_down_on_arrival = False

clear_distance_unscaled = 10
clear_distance = clear_distance_unscaled
wall_factor = 100

giveUpMilis = 1000 * 10

def map_metadataCallback(map_metadata_message):
    '''Map metadata memory update'''
    global map_metadata
    global cat_linear_speed_unscaled, clear_distance_unscaled
    global cat_linear_speed, clear_distance

    map_metadata = map_metadata_message

    resolution = map_metadata.resolution
    cat_linear_speed = resolution * cat_linear_speed_unscaled
    clear_distance = resolution * clear_distance_unscaled

def odomCallback(odom_message):
    '''Odometry memory update'''
    global robot_odom
    robot_odom = odom_message


def sightCallback(sight_message):
    '''Mouse Sight memory update'''
    global mouse_position, chasing
    mouse_position = closest_mouse(sight_message)
    
    if mouse_position != []:
        if abs(mouse_position.dist) < 0.1:
            rospy.loginfo('Caught a mouse')
        else:
            chasing = True

def stop():
    global mouse_position
    mouse_position = []

    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0

    velocity_publisher.publish(msg)    

def subscribers():
    position_topic = '/' + CAT_NAME + '/odom'
    rospy.Subscriber(position_topic, Odometry, odomCallback)

    sight_topic = '/' + CAT_NAME + '/sight'
    rospy.Subscriber(sight_topic, RobotsSpotted, sightCallback)

    map_metadata_topic = '/map_metadata'
    rospy.Subscriber(map_metadata_topic, MapMetaData, map_metadataCallback)

def closest_mouse(sight_message):
    visible_mice = sight_message.robotsSpotted
    if len(visible_mice) > 0:
        closest_mouse = min(visible_mice, key=lambda x: x.dist)
    return closest_mouse if closest_mouse.dist > 0 else []

def cat_movement():
    '''Control the cat movement'''
    global mouse_position, robot_odom, map_metadata, wall_factor
    global cat_linear_speed, cat_angular_speed, drift_angle
    global slow_down_on_arrival, clear_distance

    position = robot_odom.pose.pose.position
    
    # Set Roam Destiny

    resolution = map_metadata.resolution
    f = wall_factor * resolution 

    use_width = map_metadata.width * resolution - f
    use_height = map_metadata.height * resolution - f

    x_roam = np.random.randint(f, high=use_width)
    y_roam = np.random.randint(f, high=use_height)

    tries = 10
    while getDistance(position.x, position.y, x_roam, y_roam) < f and tries > 0:
        x_roam = np.random.randint(f, high=use_width)
        y_roam = np.random.randint(f, high=use_height)
        tries -= 1
    
    distance = map_metadata.width + map_metadata.height # Just a number to high

    # Give up on movement after a set ammount of time, avoid getting stuck
    startTime = now()
    running = True

    # Move
    while distance > clear_distance and not rospy.is_shutdown() and running:
        if mouse_position != []: # Chasing
            print("Chasing...")

            position = robot_odom.pose.pose.position

            x_mouse = position.x + mouse_position.dist * cos(mouse_position.angle)
            y_mouse = position.y + mouse_position.dist * sin(mouse_position.angle)
            velocity_message, distance = move_to_goal_ex(robot_odom, x_mouse, y_mouse, cat_linear_speed, cat_angular_speed, drift_angle, slow_down_on_arrival)
            velocity_publisher.publish(velocity_message)
            rate.sleep()
            startTime = now()
        else:
            print("Roaming...")

            velocity_message, distance = move_to_goal_ex(robot_odom, x_roam, y_roam, cat_linear_speed, cat_angular_speed, drift_angle, slow_down_on_arrival)
            velocity_publisher.publish(velocity_message)
            rate.sleep()

            currentTime = now()
            if currentTime - startTime >= giveUpMilis:
                running = False
                print("Movement timed out")
    stop()

def now():
    return int(round(time.time() * 1000))

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

