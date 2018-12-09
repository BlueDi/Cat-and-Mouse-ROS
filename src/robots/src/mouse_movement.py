#!/usr/bin/env python

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, MapMetaData
from go_to_goal import move_to_goal


map_metadata = MapMetaData(resolution=0.02, width=1000, height=1000)
robot_odom = Odometry()


def odomCallback(odom_message):
    '''Odometry memory update'''
    global robot_odom
    robot_odom = odom_message


def map_metadataCallback(map_metadata_message):
    '''Map metadata memory update'''
    global map_metadata
    map_metadata = map_metadata_message


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
    rate = rospy.Rate(10)
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


def subscribers(mouse_name):
    position_topic = '/' + mouse_name + '/odom'
    rospy.Subscriber(position_topic, Odometry, odomCallback)
    
    map_metadata_topic = '/map_metadata'
    rospy.Subscriber(map_metadata_topic, MapMetaData, map_metadataCallback)

if __name__ == '__main__':
    mouse_name = rospy.get_param("mouse_name")
    try:
        rospy.init_node(mouse_name + '_movement')
        
        cmd_vel_topic = '/' + mouse_name + '/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        subscribers(mouse_name)        

        i = 10
        while i > 0 and not rospy.is_shutdown():
            i -= 1
            random_movement()

    except rospy.ROSInterruptException:
        rospy.loginfo(mouse_name + ' terminated.')

