#!/usr/bin/env python

import rospy
from math import atan2, pow, sqrt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


robot_odom = Odometry()


def odomCallback(odom_message):
    '''Odometry memory update'''
    global robot_odom
    robot_odom = odom_message


def getDistance(x1,y1,x2,y2):
    '''Euclidean distance between two points'''
    return sqrt(pow((x1-x2),2) + pow((y1-y2),2))


def go_to_goal(x_goal, y_goal):
    '''PID Controller'''
    global robot_odom
    velocity_message = Twist()
    
    while(True):
        position = robot_odom.pose.pose.position
        orientation = robot_odom.pose.pose.orientation
        orientation_array = [orientation.x, orientation.y, orientation.z, orientation.w]
        k_linear = 0.5 #Proportional Controller
        distance = abs(getDistance(x_goal, y_goal, position.x, position.y))
        
        linear_speed = distance * k_linear

        k_angular = 4.0
        roll, pitch, yaw = euler_from_quaternion(orientation_array)
        desired_angle_goal = atan2(y_goal - position.y, x_goal - position.x)
        angular_speed = (desired_angle_goal - yaw) * k_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)
        print(distance)
        if(distance < 0.02):
            break


if __name__ == '__main__':
    try:
        rospy.init_node('go_to_goal', anonymous=True)
        
        cmd_vel_topic = '/robot1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = '/robot1/odom'
        pose_subscriber = rospy.Subscriber(position_topic, Odometry, odomCallback)
        
        go_to_goal(7.0, 5.0)

    except rospy.ROSInterruptException:
        rospy.loginfo('node terminated.')

