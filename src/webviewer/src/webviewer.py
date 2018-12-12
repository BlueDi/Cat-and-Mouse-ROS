#!/usr/bin/env python

import re
import rospy
import tf
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry, MapMetaData

map_metadata = MapMetaData(resolution=0.02, width=1000, height=1000)
map_resolution = map_metadata.resolution
map_width = map_metadata.width * map_resolution
map_height = map_metadata.height * map_resolution

robots = {}

def mouse_callback(data):
    id = data.child_frame_id
    t = "mouse"
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    q = [data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
    a = tf.transformations.euler_from_quaternion(q)[2]

    global robots
    robots[id] = [id, t, x, y, a]

    fire()

def cat_callback(data):
    id = data.child_frame_id
    t = "cat"
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    q = [data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
    a = tf.transformations.euler_from_quaternion(q)[2]

    global robots
    robots[id] = [id, t, x, y, a]

    fire()

def fire():
    global map_width, map_height, map_resolution, robots

    rl = "["
    ri = 0

    for r in robots:
        rs = '{ "id" : "%s", "t" : "%s", "x" : %s, "y" : %s, "a" : %s }' % (robots[r][0], robots[r][1], robots[r][2], robots[r][3], robots[r][4])
        if ri == 0:
            rl += rs
            ri = 1
        else:
            rl += ", " + rs
    rl += "]"

    print('{ "map" : { "width": %s, "height": %s, "resolution": %s }, "robots" : %s }', map_width, map_height, map_resolution, rl)

def listener():
    rospy.init_node('listener', anonymous=True)

    pmouse = re.compile("/mouse.*/odom")
    pcat = re.compile("/cat.*/odom")

    for topic in rospy.get_published_topics():
        if pmouse.match(topic[0]):
            rospy.Subscriber(topic[0], Odometry, mouse_callback)
        elif pcat.match(topic[0]):
            rospy.Subscriber(topic[0], Odometry, cat_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

