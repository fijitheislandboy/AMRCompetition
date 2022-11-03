#!/usr/bin/env python
import rospy
import random
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

class RobotStates:
    def __init__(self):
        # robot rate at which commands get run
        self.running_rate = 10

        # go-to-goal target positions and angle
        self.target_goal_x = 0
        self.target_goal_y = 0
        self.theta_init = 0

        # robot output velocity
        self.velocity = 0
        self.angular_velocity = 0

        # robot measured velocity based on odom
        self.measured_linear_velocity = 0
        # lidar measurements
        self.angle_range = []
        self.distance_range = []
        self.lidar_distances = []
        self.intensities = []
        self.lidar_increment = 0

        # robot current position and heading
        self.x = 0
        self.y = 0
        self.theta = 0

        # proportional controller constants
        self.kv = 0.3
        self.kp = 1


robot_states = RobotStates()


def odom_info(msg):
    global robot_states
    robot_states.measured_linear_velocity = msg.twist.twist.linear.x
    robot_states.x = msg.pose.pose.position.x
    robot_states.y = msg.pose.pose.position.y
    _, _, robot_states.theta = euler_from_quaternion(
        [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
         msg.pose.pose.orientation.w])
    print("new theta: " + str(robot_states.theta))

def lidar_info(msg):
    global robot_states
    robot_states.angle_range = [msg.angle_min, msg.angle_max]
    robot_states.distance_range = [msg.range_min, msg.range_max]
    robot_states.lidar_distances = msg.ranges
    robot_state.lidar_increment = msg.angle_increment # added lidar increment for calculating angle
    print(msg.ranges)
    robot_states.intensities = msg.intensities

def find_location():
    # update the target_goal_x, and target_goal_y
    global robot_states
    d = 0
    for i in range(len(robot_states.lidar_distances)):
        if robot_states.lidar_distances[i] < 0.45:
            d = robot_states.lidar_distances[i]
            break # obtain the index of the obstacle angle
    if d == 0: # nothing is in range
        robot_states.target_goal_x = robot_states.x
        robot_states.target_goal_y = robot_states.y
    else:
        total_angle = i * robot_states.lidar_increment - math.pi
        robot_states.target_goal_x = robot_states.x + math.cos(total_angle)
        robot_states.target_goal_y = robot_states.y + math.sin(total_angle)


def main():
    global robot_states
    rospy.init_node("rosbot_gotogoal")
    odom_sub = rospy.Subscriber("/odom", Odometry, odom_info)
    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    lidar_sub = rospy.Subscriber("/scan", LaserScan, lidar_info)
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
        if len(robot_states.lidar_distances) != 0:
            find_location() # update the target location after the count =0

        # get updated distance from robot to goal
        distance_from_goal = math.sqrt((robot_states.target_goal_x - robot_states.x) ** 2 +
                                       (robot_states.target_goal_y - robot_states.y) ** 2)

        if distance_from_goal < 0.01:
            robot_states.velocity = 0
            robot_states.angular_velocity = 0
        else:

                # calculate f_att, f_rep
            # robot velocity proportional controller: move slow when closer to goal
            robot_states.velocity = robot_states.kv * distance_from_goal

            # robot angle to goal point
            theta_d = math.atan2((robot_states.target_goal_y - robot_states.y),
                                 (robot_states.target_goal_x - robot_states.x))

            print("robot x:" + str(robot_states.x))
            print("robot target x:" + str(robot_states.target_goal_x))
            print("robot y: " + str(robot_states.y))
            print("robot target y: " + str(robot_states.target_goal_y))
            print("theta_d: " + str(theta_d))
            print("robot init theta:" + str(robot_states.theta_init))
            print("distance from goal: " + str(distance_from_goal))

            # angle error
            e = theta_d - (robot_states.theta)
            print("robot e: " + str(e))
            # robot angle proportional controller: faster turn movement
            gamma = robot_states.kp * math.atan2(math.sin(e), math.cos(e))

            robot_states.angular_velocity = gamma

        # set new velocity to publish
        velocity.linear.x = 0#robot_states.velocity
        velocity.angular.z = 0#robot_states.angular_velocity
        vel_pub.publish(velocity)
        rate.sleep()


if __name__ == "__main__":
    main()


