#!/usr/bin/env python

import sys
from math import degrees, cos, sin, pi
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, MapMetaData
from cat_mouse_world.msg import RobotsSpotted, Noises
from coords import add_spherical_to_cart
from go_to_goal import move_to_goal_ex, getDistance, LaserData
from angular_movement import *
from linear_movement import *
from sensor_msgs.msg import LaserScan


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

now = rospy.Time()
giveUp = rospy.Duration(10)

laser_proximity_threstold = 1.5 # How far should the robot stay from the walls
laser_data = None

avoiding_wall = False


def laserCallback(data):
    global laser_proximity_threstold, laser_data
    laser_data = LaserData(
        data.ranges,
        data.angle_min,
        data.angle_increment,
        laser_proximity_threstold
    )


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
    global mouse_position
    mouse_position = closest_mouse(sight_message)
    
    if mouse_position == []:
        str = '[%s] No mouse near'%CAT_NAME
        rospy.loginfo_throttle(5, str)
    elif 0 <= mouse_position.dist < 0.1:
        str = '[%s] Caught a mouse'%CAT_NAME
        rospy.loginfo_throttle(5, str)


def noiseCallback(noise_message):
    '''Mouse Noise memory update'''
    global noise_position, now
    
    new_noise_position = closest_noise(noise_message)
    if new_noise_position != [] or rospy.get_rostime() > now + rospy.Duration(10):
        now = rospy.get_rostime()
        noise_position = new_noise_position


def closest_mouse(sight_message):
    visible_mice = filter(lambda mouse: mouse.dist > 0, sight_message.robotsSpotted)

    if len(visible_mice) > 0:
        return min(visible_mice, key=lambda x: x.dist)
    return []


def closest_noise(noise_message):
    noises = filter(lambda noise: noise.volume > 0, noise_message.noises)

    if len(noises) > 0:
        return max(noises, key=lambda x: x.volume)
    return []


def stop():
    global mouse_position
    mouse_position = []
    velocity_publisher.publish(Twist())    


def subscribers():
    laser_topic = '/' + CAT_NAME + '/laser_1'
    rospy.Subscriber(laser_topic, LaserScan, laserCallback)

    position_topic = '/' + CAT_NAME + '/odom'
    rospy.Subscriber(position_topic, Odometry, odomCallback)

    sight_topic = '/' + CAT_NAME + '/sight'
    rospy.Subscriber(sight_topic, RobotsSpotted, sightCallback)
    
    noise_topic = '/' + CAT_NAME + '/noise'
    rospy.Subscriber(noise_topic, Noises, noiseCallback)

    map_metadata_topic = '/map_metadata'
    rospy.Subscriber(map_metadata_topic, MapMetaData, map_metadataCallback)


def generate_random_coords():
    '''Generate random coords inside the walls'''
    global map_metadata, robot_odom
    position = robot_odom.pose.pose.position
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


def cat_chase():
    '''Moves the cat in the direction of the closest mouse'''
    global mouse_position, robot_odom
    str = '[%s] Chasing a Mouse!'%CAT_NAME
    rospy.loginfo_throttle(5, str)
    
    position = robot_odom.pose.pose.position
    x_mouse, y_mouse = add_spherical_to_cart(position, mouse_position)
    cat_move_to(x_mouse, y_mouse)


def cat_search():
    '''Moves the cat in the direction of the strongest noise'''
    global noise_position, robot_odom
    str = '[%s] Following a Noise!'%CAT_NAME
    rospy.loginfo_throttle(5, str)

    position = robot_odom.pose.pose.position
    x_noise, y_noise = add_spherical_to_cart(position, noise_position)
    cat_move_to(x_noise, y_noise)


def cat_roam(x, y):
    '''Moves the cat in a random direction'''
    str = '[%s] Roaming.'%CAT_NAME
    rospy.loginfo_throttle(5, str)
    cat_move_to(x, y)


def cat_move_to(x, y):
    '''Moves the cat in the direction of (x,y)'''
    global robot_odom
    global cat_linear_speed, cat_angular_speed, drift_angle, slow_down_on_arrival
    global avoiding_wall
    velocity_message, distance, avoiding_wall = move_to_goal_ex(robot_odom, x, y, cat_linear_speed, cat_angular_speed, drift_angle, slow_down_on_arrival, laser_data, avoiding_wall)
    velocity_publisher.publish(velocity_message)


def cat_movement():
    '''Control the cat movement'''
    global mouse_position, noise_position, robot_odom, map_metadata, wall_factor
    global clear_distance

    x_roam, y_roam = generate_random_coords()

    # Give up on movement after a set ammount of time, avoid getting stuck
    startTime = rospy.get_rostime()

    # Move
    distance = clear_distance + 1
    while distance > clear_distance and not rospy.is_shutdown():
        saw_mouse = mouse_position != []
        heard_mouse = noise_position != []

        if saw_mouse:
            cat_chase()
            startTime = rospy.get_rostime()
        elif heard_mouse:
            cat_search()
            startTime = rospy.get_rostime()
        else:
            cat_roam(x_roam, y_roam)
            currentTime = rospy.get_rostime()
            if currentTime - startTime >= giveUp:
                rospy.logwarn('[%s] Movement timed out', CAT_NAME)
                break
    stop()


if __name__ == '__main__':
    try:
        CAT_NAME = sys.argv[1]
    except IndexError:
        CAT_NAME = 'cat0'

    try:
        rospy.init_node(CAT_NAME + '_movement')
        now = rospy.get_rostime()
        rate = rospy.Rate(10)
        
        cmd_vel_topic = '/' + CAT_NAME + '/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=2)
        
        subscribers()

        while not rospy.is_shutdown():
            cat_movement()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logwarn(CAT_NAME + ' terminated.')

