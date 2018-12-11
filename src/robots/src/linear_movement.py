#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist


def move(velocity_publisher, speed, distance, is_forward):
    velocity_message = Twist()
    velocity_message.linear.x = abs(speed) if is_forward else -abs(speed)

    t0 = rospy.get_time()
    curr_distance = 0.0
    while curr_distance < distance:
        t1 = rospy.get_time()
        curr_distance = speed * (t1 - t0)
        velocity_publisher.publish(velocity_message)

    velocity_message = Twist()
    velocity_publisher.publish(velocity_message)


if __name__ == '__main__':
    try:
        rospy.init_node('robot_linear_movement', anonymous=True)
        
        cmd_vel_topic = '/robot0/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        time.sleep(2)
        
        move(velocity_publisher, 1, 2, True)

    except rospy.ROSInterruptException:
        rospy.loginfo('node terminated.')

