#!/usr/bin/env python

import sys
from math import cos, sin, pi, pow, atan, sqrt
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, MapMetaData
from cat_mouse_world.msg import RobotsSpotted, RobotSpotted
from geometry_msgs.msg import Pose
from coords import add_spherical_to_cart
from go_to_goal import move_to_goal_laser, LaserData
from linear_movement import move
from sensor_msgs.msg import LaserScan

cat_position = []
map_metadata = MapMetaData(resolution=0.02, width=1000, height=1000)
robot_odom = Odometry()

avoiding_wall = False
clear_distance_unscaled = 10
clear_distance = clear_distance_unscaled
laser_proximity_threstold = 1.0 # How far should the robot stay from the walls
laser_data = None

def laserCallback(data):
    global laser_proximity_threstold, laser_data
    laser_data = LaserData(
        data.ranges,
        data.angle_min,
        data.angle_increment,
        laser_proximity_threstold
    )

def odomCallback(odom_message):
    '''Odometry memory update'''
    global robot_odom
    robot_odom = odom_message


def sightCallback(sight_message):
    '''Cat Sight memory update'''
    global cat_position
    cat_position = cat_pseudo_position(sight_message)
    if cat_position == []:
        str = '[%s] No cat nearby.'%MOUSE_NAME
        rospy.loginfo_throttle(5, str)
    elif 0 <= cat_position.dist < 0.1:
        str = '[%s] Died.'%MOUSE_NAME
        rospy.loginfo_throttle(5, str)


def map_metadataCallback(map_metadata_message):
    '''Map metadata memory update'''
    global map_metadata
    map_metadata = map_metadata_message
    resolution = map_metadata.resolution
    clear_distance = resolution * clear_distance_unscaled


def cat_pseudo_position(sight_message):
    '''
    Using all the visible cats positions,
    calculate a pseudo position of
    the sum of all cat positions
    '''
    closest_cat = RobotSpotted()
    visible_cats = filter(lambda cat: cat.dist >= 0, sight_message.robotsSpotted)

    if len(visible_cats) > 1:
        cat_temp = Pose().position
        for cat in visible_cats:
            cat_temp.x, cat_temp.y = add_spherical_to_cart(cat_temp, cat)
        cat_temp.x /= len(visible_cats)
        if cat_temp.y == 0:
            closest_cat.angle = 0
        else:
            cat_temp.y /= len(visible_cats)
            closest_cat.angle = atan(cat_temp.x / cat_temp.y) + pi
        closest_cat.dist = sqrt(pow(cat_temp.x, 2) + pow(cat_temp.y, 2))
    return closest_cat if closest_cat.dist > 0 else []


def stop():
    global cat_position
    cat_position = []
    velocity_publisher.publish(Twist())  


def generate_random_coords():
    global map_metadata
    resolution = map_metadata.resolution
    width = map_metadata.width * resolution
    height = map_metadata.height * resolution
    rng_width = np.random.randint(1, high=width)
    rng_height = np.random.randint(1, high=height)
    return rng_width, rng_height


def mouse_runaway():
    '''Moves the mouse in the oposite direction of the cats'''
    global cat_position, robot_odom
    str = '[%s] Running away from a cat!'%MOUSE_NAME
    rospy.loginfo_throttle(5, str)

    position = robot_odom.pose.pose.position
    x_goal, y_goal = add_spherical_to_cart(position, cat_position)
    mouse_move_to(x_goal, y_goal)


def mouse_roam():
    '''Moves the mouse in a random direction'''
    str = '[%s] Roaming.'%MOUSE_NAME
    rospy.loginfo_throttle(5, str)
    rng_x, rng_y = generate_random_coords()
    mouse_move_to(rng_x, rng_y)


def mouse_move_to(x_goal, y_goal):
    '''Moves the mouse in the direction of (x,y)'''
    global robot_odom, avoiding_wall
    velocity_message, distance, avoiding_wall = move_to_goal_laser(robot_odom, x_goal, y_goal, laser_data, avoiding_wall)
    velocity_publisher.publish(velocity_message)


def mouse_movement():
    '''Control the mouse movement'''
    global mouse_position
    saw_cat = cat_position != []
    distance = clear_distance + 1
    while distance > clear_distance and not rospy.is_shutdown():
        if saw_cat:
            mouse_runaway()
        else:
            mouse_roam()
    stop()


def subscribers():
    laser_topic = '/' + MOUSE_NAME + '/laser_0'
    rospy.Subscriber(laser_topic, LaserScan, laserCallback)

    position_topic = '/' + MOUSE_NAME + '/odom'
    rospy.Subscriber(position_topic, Odometry, odomCallback)
    
    sight_topic = '/' + MOUSE_NAME + '/sight'
    rospy.Subscriber(sight_topic, RobotsSpotted, sightCallback)
    
    map_metadata_topic = '/map_metadata'
    rospy.Subscriber(map_metadata_topic, MapMetaData, map_metadataCallback)


if __name__ == '__main__':
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

