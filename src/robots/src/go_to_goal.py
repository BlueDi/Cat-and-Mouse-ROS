#!/usr/bin/env python

import rospy
from math import atan2, pow, sqrt, pi
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


MIN_DISTANCE = 0.2
robot_odom = Odometry()

class LaserData:
    def __init__(self, firstAngle, angleIncrement, ranges, proximityThrestold):
        self.ranges = ranges
        self.proximityThrestold = proximityThrestold
        self.angles = []

        for i in range(0, len(ranges)):
            angle = firstAngle + (i * angleIncrement)
            if angle > 2 * pi:
                angle -= 2 * pi
            self.angles.append(angle)

def odomCallback(odom_message):
    '''Odometry memory update'''
    global robot_odom
    robot_odom = odom_message


def getDistance(x1,y1,x2,y2):
    '''Euclidean distance between two points'''
    return sqrt(pow((x1-x2),2) + pow((y1-y2),2))


def go_to_goal(velocity_publisher, robot_odom, x_goal, y_goal):
    '''PID Controller'''    
    distance = MIN_DISTANCE + 1
    while distance > MIN_DISTANCE and not rospy.is_shutdown():
        velocity_message, distance = move_to_goal(robot_odom, x_goal, y_goal)
        velocity_publisher.publish(velocity_message)

    return True


def move_to_goal(robot_odom, x_goal, y_goal):
    return move_to_goal_laser(robot_odom, x_goal, y_goal, None)

def move_to_goal_laser(robot_odom, x_goal, y_goal, lasers):
    position = robot_odom.pose.pose.position
    distance = abs(getDistance(x_goal, y_goal, position.x, position.y))
    linear_speed = distance * 0.5 #Proportional Controller
    k_angular = 4.0
    drift_angle = pi / 180 * 30

    velocity_message, distance = move_to_goal_ex(robot_odom, x_goal, y_goal, linear_speed, k_angular, drift_angle, True, lasers)

    return velocity_message, distance

def move_to_goal_ex(robot_odom, goal_x, goal_y, bot_linear_speed, bot_angular_speed, drift_angle, brake, lasers):
    velocity_message = Twist()
    position = robot_odom.pose.pose.position

    orientation = robot_odom.pose.pose.orientation
    orientation_array = [orientation.x, orientation.y, orientation.z, orientation.w]
    roll, pitch, yaw = euler_from_quaternion(orientation_array)

    bot_x = position.x
    bot_y = position.y
    
    if(yaw >= 0):
        bot_angle = yaw
    else:
        bot_angle = 2 * pi + yaw

    linear_speed = bot_linear_speed
    angular_speed = bot_angular_speed

    distance = abs(getDistance(bot_x, bot_y, goal_x, goal_y))  
    if distance < linear_speed and brake:
        linear_speed = distance

    angle_aux = atan2(goal_y - position.y, goal_x - position.x)
    if angle_aux >= 0:
        target_angle = angle_aux
    else:
        target_angle = 2 * pi + angle_aux

    angle_diff = 0
    if target_angle > bot_angle:
        s1 = abs(target_angle - bot_angle)
        s2 = abs(bot_angle - (target_angle - 2 * pi))

        if s1 <= s2:
            angle_diff = s1
        else:
            angle_diff = -s2
    else:
        s1 = abs(bot_angle - target_angle)
        s2 = abs(target_angle - (bot_angle - 2 * pi))

        if s1 <= s2:
            angle_diff = -s1
        else:
            angle_diff = s2
    
    if abs(angle_diff) > angular_speed:
        if angle_diff > 0:
            angular_speed = angular_speed
        else:
            angular_speed = -angular_speed
    else:
        angular_speed = angle_diff
    
    if abs(angular_speed) < 0.005:
        angular_speed = 0
    if abs(linear_speed) < 0.005:
        linear_speed = 0

    if abs(angular_speed) < drift_angle:
        if abs(angular_speed) > 0.05:
            velocity_message.linear.x = linear_speed * (1 - abs(angular_speed) / drift_angle)
        else:
            velocity_message.linear.x = linear_speed        
    else:
        velocity_message.linear.x = 0
    velocity_message.angular.z = angular_speed

    return velocity_message, distance


if __name__ == '__main__':
    try:
        rospy.init_node('go_to_goal', anonymous=True)
        
        cmd_vel_topic = '/robot1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = '/robot1/odom'
        pose_subscriber = rospy.Subscriber(position_topic, Odometry, odomCallback)
        
        go_to_goal(velocity_publisher, robot_odom, rospy.get_param("x_goal"), rospy.get_param("y_goal"))

    except rospy.ROSInterruptException:
        rospy.loginfo('node terminated.')

