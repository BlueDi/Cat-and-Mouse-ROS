#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist


def move(speed, distance, is_forward):
    rate = rospy.Rate(10)
    velocity_message = Twist()

    velocity_message.linear.x = abs(speed) if is_forward else -abs(speed)
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0

    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0

    t0 = rospy.get_time()
    curr_distance = 0.0
    while curr_distance < distance:
        t1 = rospy.get_time()
        curr_distance = speed * (t1 - t0)
        velocity_publisher.publish(velocity_message)
        rate.sleep()

    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)


if __name__ == '__main__':
    try:
        rospy.init_node('robot_linear_movement', anonymous=True)
        
        cmd_vel_topic = '/robot0/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        time.sleep(2)
        
        move(1, 2, True)

    except rospy.ROSInterruptException:
        rospy.loginfo('node terminated.')

