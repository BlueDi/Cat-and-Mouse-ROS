#!/usr/bin/env python

import sys
from math import degrees, cos, sin, pi
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, MapMetaData
from cat_mouse_world.msg import RobotsSpotted, Noises
from go_to_goal import move_to_goal_ex, getDistance
from angular_movement import *
from linear_movement import *
import time


mouse_position = []
noise_position = []

robot_odom = Odometry()
map_metadata = MapMetaData(resolution=0.02, width=1000, height=1000)

cat_linear_speed_unscaled = 100
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
    
    if mouse_position == []:
        rospy.logdebug('No mouse near')
    elif 0 <= mouse_position.dist < 0.1:
        rospy.loginfo('Caught a mouse')


def noiseCallback(noise_message):
    '''Mouse Noise memory update'''
    global noise_position, chasing
    noise_position = closest_noise(noise_message)


def closest_mouse(sight_message):
    visible_mice = sight_message.robotsSpotted
    if len(visible_mice) > 0:
        closest_mouse = min(visible_mice, key=lambda x: x.dist)
    return closest_mouse if closest_mouse.dist > 0 else []


def closest_noise(noise_message):
    noises = noise_message.noises
    if len(noises) > 0:
        closest_noise = max(noises, key=lambda x: x.volume)
    return closest_noise if closest_noise.volume > 0 else []


def stop():
    global mouse_position
    mouse_position = []
    velocity_publisher.publish(Twist())    


def subscribers():
    position_topic = '/' + CAT_NAME + '/odom'
    rospy.Subscriber(position_topic, Odometry, odomCallback)

    sight_topic = '/' + CAT_NAME + '/sight'
    rospy.Subscriber(sight_topic, RobotsSpotted, sightCallback)
    
    noise_topic = '/' + CAT_NAME + '/noise'
    rospy.Subscriber(noise_topic, Noises, noiseCallback)

    map_metadata_topic = '/map_metadata'
    rospy.Subscriber(map_metadata_topic, MapMetaData, map_metadataCallback)


def generate_random_coords(position):
    '''Generate random coords inside the walls'''
    global map_metadata
    x_roam = position.x
    y_roam = position.y
    resolution = map_metadata.resolution
    f = wall_factor * resolution
    width = map_metadata.width * resolution - f
    height = map_metadata.height * resolution - f
    while getDistance(position.x, position.y, x_roam, y_roam) < f:
        x_roam = np.random.randint(1, high=width)
        y_roam = np.random.randint(1, high=height)
    return x_roam, y_roam


def cat_movement():
    '''Control the cat movement'''
    global mouse_position, robot_odom, map_metadata, wall_factor
    global cat_linear_speed, cat_angular_speed, drift_angle
    global slow_down_on_arrival, clear_distance

    position = robot_odom.pose.pose.position
    x_roam, y_roam = generate_random_coords(position)
    
    distance = map_metadata.width + map_metadata.height # Just a number to high

    # Give up on movement after a set ammount of time, avoid getting stuck
    startTime = now()

    # Move
    while distance > clear_distance and not rospy.is_shutdown():
        if mouse_position != []:
            rospy.loginfo_throttle(5, "Chasing a Mouse!")

            position = robot_odom.pose.pose.position

            x_mouse = position.x + mouse_position.dist * cos(mouse_position.angle)
            y_mouse = position.y + mouse_position.dist * sin(mouse_position.angle)
            velocity_message, distance = move_to_goal_ex(robot_odom, x_mouse, y_mouse, cat_linear_speed, cat_angular_speed, drift_angle, slow_down_on_arrival)
            velocity_publisher.publish(velocity_message)
            rate.sleep()
            startTime = now()
        elif noise_position != []:
            rospy.loginfo_throttle(5, "Following a Noise!")

            position = robot_odom.pose.pose.position

            x_noise = position.x + noise_position.volume  * cos(noise_position.angle)
            y_noise = position.y + noise_position.volume * sin(noise_position.angle)
            velocity_message, distance = move_to_goal_ex(robot_odom, x_noise, y_noise, cat_linear_speed, cat_angular_speed, drift_angle, slow_down_on_arrival)
            velocity_publisher.publish(velocity_message)
            rate.sleep()
            startTime = now()
        else:
            rospy.loginfo_throttle(5, "Roaming")

            velocity_message, distance = move_to_goal_ex(robot_odom, x_roam, y_roam, cat_linear_speed, cat_angular_speed, drift_angle, slow_down_on_arrival)
            velocity_publisher.publish(velocity_message)
            rate.sleep()

            currentTime = now()
            if currentTime - startTime >= giveUpMilis:
                rospy.logwarn("Movement timed out")
                break
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
        rospy.logwarn(CAT_NAME + ' terminated.')

