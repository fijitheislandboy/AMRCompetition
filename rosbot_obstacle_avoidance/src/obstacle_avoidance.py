#!/usr/bin/env python
import rospy
import random
import math
import numpy as np
import os
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


class RobotStates:
    def __init__(self):
        # robot rate at which commands get run
        self.running_rate = 10

        # go-to-goal target positions and angle
        self.target_goal_x = 2
        self.target_goal_y = 0
        self.theta_init = 0

        # robot output velocity
        self.velocity = 0
        self.angular_velocity = 0

        # robot measured velocity based on odom
        self.measured_linear_velocity = 0

        # robot current position and heading
        self.x = 0
        self.y = 0
        self.theta = 0

        # proportional controller constants
        self.kv = 0.3
        self.kp = 1

        # lidar measurements
        self.angle_range = []
        self.distance_range = []
        self.lidar_distances = []
        self.intensities = []

        # robot obstacle avoidance constants
        self.nu = 0.5
        self.rho_0 = 0.2
        self.eta = 0.4



robot_states = RobotStates()


def odom_info(msg):
    global robot_states
    robot_states.measured_linear_velocity = msg.twist.twist.linear.x
    robot_states.x = msg.pose.pose.position.x
    robot_states.y = msg.pose.pose.position.y
    _, _, robot_states.theta = euler_from_quaternion(
        [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
         msg.pose.pose.orientation.w])
    # print("new theta: " + str(robot_states.theta))


def pose_info(msg):
    global robot_states
    # print(msg)

def lidar_info(msg):
    global robot_states
    robot_states.angle_range = [msg.angle_min, msg.angle_max]
    robot_states.distance_range = [msg.range_min, msg.range_max]
    robot_states.lidar_distances = msg.ranges
    print(msg.ranges)
    robot_states.intensities = msg.intensities
    # print(msg)

def frep():
    global robot_states
    frep_sum = [0, 0]
    for i in range(0, len(robot_states.lidar_distances)):

        if robot_states.lidar_distances[i] < float('inf'):
            obstacle_angle_from_robot = robot_states.angle_range[0] + \
                                       (robot_states.angle_range[1] - robot_states.angle_range[0]) * \
                                       i / len(robot_states.lidar_distances) - math.pi
            obstacle_x = robot_states.lidar_distances[i] * math.cos(obstacle_angle_from_robot)
            obstacle_y = robot_states.lidar_distances[i] * math.sin(obstacle_angle_from_robot)

            # print(obstacle_x)
            # print(obstacle_y)
            # print(robot_states.x)
            # print(robot_states.y)
            frep_point = frep_for_point(robot_states.x, robot_states.y, obstacle_x, obstacle_y, robot_states.rho_0, robot_states.nu)

            # print(frep_point)
            frep_sum[0] += frep_point[0]
            frep_sum[1] += frep_point[1]

    print(frep_sum)
    return frep_sum



def frep_for_point(current_x, current_y, target_x, target_y, rho_0, nu):
    target_distance = distance(current_x, current_y, target_x, target_y)
    q = [current_x, current_y]
    b = [target_x, target_y]

    # print(q)
    # print(b)
    delta = [(q[0] - b[0])/target_distance, (q[1] - b[1])/target_distance]

    # print(delta)
    multiplier = nu*((1/target_distance) - (1/rho_0)) * (1/target_distance**2)
    return [delta[0] * multiplier, delta[1]* multiplier]

def distance(x1, x2, y1, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

"""
return an index for open path based on lidar closest to center robot path in the given direction
"""
def find_open_path(lidar_distances, direction, threshold):
    center_index = len(lidar_distances) // 2
    path_index = center_index

    if direction == -1:
        end = -1
    else:
        end = len(lidar_distances)

    for i in range(center_index, end, direction):
        if lidar_distances[i] > threshold:
            return i

    return -1




def main():
    global robot_states
    rospy.init_node("rosbot_gotogoal")
    odom_sub = rospy.Subscriber("/odom", Odometry, odom_info)

    # reset pose
    reset_pose = PoseWithCovarianceStamped()
    reset_pose.pose.pose.position = {0, 0, 0}
    reset_pose.pose.pose.orientation = {0,0,0,0}

    reset_pub = rospy.Publisher("/set_pose", PoseWithCovarianceStamped, queue_size=1)
    reset_pub.publish(reset_pose)
    # reset_pose.position.x = 0.0
    # reset_pose.position.y = 0.0
    # reset_pose.position.z = 0.0
    pose_sub = rospy.Subscriber("/pose", PoseStamped, pose_info)
    lidar_sub = rospy.Subscriber("/scan", LaserScan, lidar_info)
    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    rate = rospy.Rate(robot_states.running_rate)
    velocity = Twist()
    velocity.linear.x = 0.0
    count = 0

    while not rospy.is_shutdown():
        # set target goal to be relative to robot's initial position
        if count == 0:
            robot_states.target_goal_x = robot_states.target_goal_x + robot_states.x
            robot_states.target_goal_y = robot_states.target_goal_y + robot_states.y
            robot_states.theta_init = robot_states.theta

        count += 1

        # get updated distance from robot to goal
        distance_from_goal = math.sqrt((robot_states.target_goal_x - robot_states.x) ** 2 +
                                       (robot_states.target_goal_y - robot_states.y) ** 2)

        # print(robot_states.lidar_distances)
        if distance_from_goal < 0.01:
            robot_states.velocity = 0
            robot_states.angular_velocity = 0
        else:
            if len(robot_states.lidar_distances) != 0:
                # calculate f_att, f_rep

                # f_att = -eta * [q - goal]
                f_att = [-robot_states.eta*(robot_states.x - robot_states.target_goal_x),
                         -robot_states.eta*(robot_states.y  - robot_states.target_goal_y)]
                print(f_att)
                f_rep = frep()
                # print(f_rep)

                f_total = f_att + f_rep

                print(f_total)
                velocity_from_force =  math.sqrt(f_total[0]**2 + f_total[1]**2)
                angle_from_force = math.atan2(f_total[1], f_total[0])


                # robot velocity proportional controller: move slow when closer to goal
                robot_states.velocity =  velocity_from_force
                
                
                # robot angle to goal point
                theta_d = math.atan2((robot_states.target_goal_y - robot_states.y),
                                     (robot_states.target_goal_x - robot_states.x))

                # print("robot x:" + str(robot_states.x))
                # print("robot target x:" + str(robot_states.target_goal_x))
                # print("robot y: " + str(robot_states.y))
                # print("robot target y: " + str(robot_states.target_goal_y))
                # print("theta_d: " + str(theta_d))
                # print("robot init theta:" + str(robot_states.theta_init))
                # print("distance from goal: " + str(distance_from_goal))

                # angle error
                e = angle_from_force - robot_states.theta
                # print("robot e: " + str(e))
                # robot angle proportional controller: faster turn movement
                gamma = robot_states.kp * math.atan2(math.sin(e), math.cos(e))

                robot_states.angular_velocity = angle_from_force

        # set new velocity to publish
        velocity.linear.x = robot_states.velocity
        velocity.angular.z = robot_states.angular_velocity
        vel_pub.publish(velocity)
        rate.sleep()


if __name__ == "__main__":
    main()


