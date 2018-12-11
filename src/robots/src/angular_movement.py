#!/usr/bin/env python

import rospy
from math import radians
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


robot_odom = Odometry()


def odomCallback(odom_message):
    '''Odometry memory update'''
    global robot_odom
    robot_odom = odom_message


def rotate(velocity_publisher, angular_speed, relative_angle, isClockwise):
    '''
    Rotation of the robot to desired relative angle.
    This rotation is in the z axis.
    
    @param angular_speed desired angular velocity in meters per second
    @param relative_angle radian angle to rotate in z
    @param isClockwise boolean set orientation of the rotation
    '''
    rate = rospy.Rate(10)
    velocity_message = Twist()
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0

    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = -abs(angular_speed) if isClockwise else abs(angular_speed)

    t0 = rospy.get_time()
    curr_ang = 0.0
    while curr_ang < relative_angle:
        t1 = rospy.get_time()
        curr_ang = angular_speed * (t1 - t0)
        velocity_publisher.publish(velocity_message)
        rate.sleep()

    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)


def get_rotation(robot_odom, desired_angle):
    '''
    Calculates the rotation of the robot to the desired angle.
    It returns a recomended angular velocity, the angle in radians, and the orientation of the turn as clockwise or counter-clockwise.
        
    @param desired_angle angle in degrees
    @return angular_speed, relative_angle, isClockwise
    '''
    position = robot_odom.pose.pose.position
    orientation = robot_odom.pose.pose.orientation
    orientation_array = [orientation.x, orientation.y, orientation.z, orientation.w]
    
    k_angular = 0.4
    roll, pitch, yaw = euler_from_quaternion(orientation_array)
    relative_angle = radians(desired_angle) - yaw
    angular_speed = relative_angle * k_angular
    
    isClockwise = relative_angle < 0
    return abs(angular_speed), abs(relative_angle), isClockwise


if __name__ == '__main__':
    try:
        rospy.init_node('robot_angular_movement', anonymous=True)
        
        cmd_vel_topic = '/robot0/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = '/robot0/odom'
        odom_subscriber = rospy.Subscriber(position_topic, Odometry, odomCallback)
        
        angular_speed, relative_angle, isClockwise = get_rotation(robot_odom, 90)
        rotate(velocity_publisher, angular_speed, relative_angle, isClockwise)
        
        rotate(0.4, 3.14/2, True)

    except rospy.ROSInterruptException:
        rospy.loginfo('node terminated.')

