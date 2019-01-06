#!/usr/bin/env python

import sys
from math import degrees, cos, sin, pi, pow, atan, sqrt
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, MapMetaData
from cat_mouse_world.msg import RobotsSpotted, RobotSpotted
from go_to_goal import move_to_goal
from linear_movement import move


cat_position = []
map_metadata = MapMetaData(resolution=0.02, width=1000, height=1000)
robot_odom = Odometry()


def odomCallback(odom_message):
    '''Odometry memory update'''
    global robot_odom
    robot_odom = odom_message


def sightCallback(sight_message):
    '''Cat Sight memory update'''
    global cat_position
    cat_position = cat_pseudo_position(sight_message)
    if cat_position == []:
        rospy.loginfo_throttle(5, 'No cat nearby.')
    elif 0 <= cat_position.dist < 0.1:
        rospy.loginfo('Died.')


def map_metadataCallback(map_metadata_message):
    '''Map metadata memory update'''
    global map_metadata
    map_metadata = map_metadata_message


def cat_pseudo_position(sight_message):
    '''
    Using all the visible cats positions,
    calculate a pseudo position of
    the sum of all cat positions
    '''
    closest_cat = RobotSpotted()
    visible_cats = sight_message.robotsSpotted
    if len(visible_cats) > 1:
        x = 0
        y = 0
        for cat in visible_cats:
            cat_x = cat.dist * cos(cat.angle)
            cat_y = cat.dist * sin(cat.angle)
            x += cat_x
            y += cat_y
        x /= len(visible_cats)
        y /= len(visible_cats)

        # Temporary Fix
        if y == 0:
            y = 0.001

        closest_cat.angle = atan(x / y)
        closest_cat.dist = sqrt(pow(x, 2) + pow(y, 2))
    return closest_cat if closest_cat.dist > 0 else []


def generate_random_coords():
    global map_metadata
    resolution = map_metadata.resolution
    width = map_metadata.width * resolution
    height = map_metadata.height * resolution
    rng_width = np.random.randint(1, high=width)
    rng_height = np.random.randint(1, high=height)
    return rng_width, rng_height


def go_to_goal(x_goal, y_goal):
    '''PID Controller'''
    global robot_odom
    distance = 0.2 + 1
    while distance > 0.2 and not rospy.is_shutdown():
        velocity_message, distance = move_to_goal(robot_odom, x_goal, y_goal)
        velocity_publisher.publish(velocity_message)
        rate.sleep()
    
    velocity_message = Twist()
    velocity_publisher.publish(velocity_message)


def random_movement():
    rng_x, rng_y = generate_random_coords()
    go_to_goal(rng_x, rng_y)


def mouse_runaway():
    '''Moves the mouse in the oposite direction of the closest cat'''
    global cat_position, robot_odom
    position = robot_odom.pose.pose.position
    x_goal = position.x + cat_position.dist * cos(cat_position.angle + pi)
    y_goal = position.y + cat_position.dist * sin(cat_position.angle + pi)
    go_to_goal(x_goal, y_goal)


def mouse_roam():
    '''Make the mouse roam the map'''
    move(velocity_publisher, 3, 0.5, True)


def mouse_movement():
    '''Control the mouse movement'''
    global mouse_position
    saw_cat = cat_position != []
    if saw_cat:
        rospy.loginfo('Saw a cat! dist:%.2f angle:%.2f', cat_position.dist, cat_position.angle)
        mouse_runaway()
    else:
        random_movement()


def subscribers():
    position_topic = '/' + MOUSE_NAME + '/odom'
    rospy.Subscriber(position_topic, Odometry, odomCallback)
    
    sight_topic = '/' + MOUSE_NAME + '/sight'
    rospy.Subscriber(sight_topic, RobotsSpotted, sightCallback)
    
    map_metadata_topic = '/map_metadata'
    rospy.Subscriber(map_metadata_topic, MapMetaData, map_metadataCallback)


if __name__ == '__main__':
    try:
        MOUSE_NAME = rospy.get_param("mouse_name")
    except KeyError:
        try:
            MOUSE_NAME = sys.argv[1]
        except IndexError:
            MOUSE_NAME = 'mouse0'

    try:
        rospy.init_node(MOUSE_NAME + '_movement')
        rate = rospy.Rate(10)
        
        cmd_vel_topic = '/' + MOUSE_NAME + '/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=2)
        
        subscribers()        

        while not rospy.is_shutdown():
            mouse_movement()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logwarn(MOUSE_NAME + ' terminated.')

