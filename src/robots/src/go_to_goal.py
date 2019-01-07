#!/usr/bin/env python

import rospy
from math import atan2, pow, sqrt, pi
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


MIN_DISTANCE = 0.2
robot_odom = Odometry()

class LaserData:
    def __init__(self, ranges, angle_min, angle_increment, proximity_threstold):
        self.ranges = ranges
        self.count = len(ranges)
        self.angle_min = angle_min
        self.angle_increment = angle_increment
        self.proximity_threstold = proximity_threstold

    def getAngle(self, i):
        return self.angle_min + (i * self.angle_increment)

    def getRange(self, i):
        return self.ranges[i]

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
    velocity_message, distance, _ = move_to_goal_laser(robot_odom, x_goal, y_goal, None, False)
    return velocity_message, distance

def move_to_goal_laser(robot_odom, x_goal, y_goal, lasers, avoiding_wall):
    position = robot_odom.pose.pose.position
    distance = abs(getDistance(x_goal, y_goal, position.x, position.y))
    linear_speed = distance * 0.5 #Proportional Controller
    k_angular = 4.0
    drift_angle = pi / 180 * 30

    velocity_message, distance, avoiding_wall = move_to_goal_ex(robot_odom, x_goal, y_goal, linear_speed, k_angular, drift_angle, True, lasers, avoiding_wall)

    return velocity_message, distance, avoiding_wall

def move_to_goal_ex(robot_odom, goal_x, goal_y, bot_linear_speed, bot_angular_speed, driftang, brake, lasers, avoiding_wall):
    velocity_message = Twist()
    position = robot_odom.pose.pose.position

    orientation = robot_odom.pose.pose.orientation
    orientation_array = [orientation.x, orientation.y, orientation.z, orientation.w]
    roll, pitch, yaw = euler_from_quaternion(orientation_array)

    bot_x = position.x
    bot_y = position.y
    
    drift_angle = driftang

    bot_angle = normalizeAngle(yaw)

    linear_speed = bot_linear_speed
    angular_speed = bot_angular_speed

    distance = abs(getDistance(bot_x, bot_y, goal_x, goal_y))  
    if distance < linear_speed and brake:
        linear_speed = distance

    angle_aux = atan2(goal_y - position.y, goal_x - position.x)
    target_angle = normalizeAngle(angle_aux)
    angle_diff = getAngleDifference(target_angle, bot_angle) if not avoiding_wall else 0
    
    # Laser -------------------------------------
    blocked = False
    if lasers != None: # Consider laser data if it exists
        dodge = pi / 180 * 30
        deadzone = pi / 180 * 20
        lt = -dodge + angle_diff
        gt = dodge + angle_diff
        dlt = -deadzone + angle_diff
        dgt = deadzone + angle_diff
        nonBlocked = []
        front_range = []

        # Determine if there's a wall in the path of the robot
        for i in range(0, lasers.count):
            r = lasers.getRange(i) # Range
            a = lasers.getAngle(i) # Angle
            b = r < lasers.proximity_threstold # Blocked

            if not b and (a < dlt or a > dgt) and r > lasers.proximity_threstold * 1.1 : # Collected non blocked angles
                nonBlocked.append((i, a, r))

            if a >= dlt and a <= dgt and r != float('Inf'):
                front_range.append(r)

            if a >= lt and a <= gt and b:
                blocked = True

        # If there's a wall in front of the robot then
        if blocked:
            # Find the least blocked half
            nonBlocked.sort(key=lambda tuple: tuple[0], reverse=False)

            front_range_avg = sum(front_range) / len(front_range) if len(front_range) > 0 else lasers.proximity_threstold

            left_half = nonBlocked[:len(nonBlocked)//2][2]
            right_half = nonBlocked[len(nonBlocked)//2:][2]

            right_avg = sum(right_half) / len(right_half)
            left_avg = sum(left_half) / len(left_half)

            right = right_avg < left_avg

            # Pick the first item from the best half
            middle = len(nonBlocked) / 2

            nonBlocked.sort(key=lambda tuple: tuple[2], reverse=True)
            nonBlocked.sort(key=lambda tuple: abs(tuple[1]), reverse=False)

            langle = 0
            for tup in nonBlocked:
                if (right and tup[0] <= middle) or (not right and tup[0] >= middle):
                    langle = tup[1]
                    break

            target_angle = normalizeAngle(bot_angle + langle)

            angle_diff = getAngleDifference(target_angle, bot_angle)

            drift_angle *= (front_range_avg / lasers.proximity_threstold)
            linear_speed *= (front_range_avg / lasers.proximity_threstold)

    # Laser -------------------------------------

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

    return velocity_message, distance, blocked

def normalizeAngle(angle):
    a = angle
    twopi = 2 * pi
    
    if a > twopi:
        a -= twopi
    elif a < 0:
        a += twopi

    return a

def getAngleDifference(target, origin):
    angle_diff = 0
    if target > origin:
        s1 = abs(target - origin)
        s2 = abs(origin - (target - 2 * pi))

        if s1 <= s2:
            angle_diff = s1
        else:
            angle_diff = -s2
    else:
        s1 = abs(origin - target)
        s2 = abs(target - (origin - 2 * pi))

        if s1 <= s2:
            angle_diff = -s1
        else:
            angle_diff = s2
    return angle_diff

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

