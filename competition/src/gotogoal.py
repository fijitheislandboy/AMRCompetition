#!/usr/bin/env python
import rospy
import random
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


class RobotStates:
    def __init__(self):
        # robot rate at which commands get run
        self.running_rate = 10

        # go-to-goal target positions and angle
        self.target_goal_x = 0.0
        self.target_goal_y = 0.0
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
        self.kp = 0.25

        # info from lidar
        self.angle_range = []
        self.distance_range = []
        self.lidar_distances = []
        self.intensities = []

        self.has_waypoint = False

    # force settings
    self.eta = 2
    self.nu = 10
    self.rho_0 = 0.5

	self.ki = 0.001
	self.kd = 0.0003

	# pid variables
	self.vref = 0.25
	self.integral = 0
	self.previous_error = 0

robot_states = RobotStates()

def odom_info(msg):
    global robot_states
    robot_states.measured_linear_velocity = msg.twist.twist.linear.x
    robot_states.x = msg.pose.pose.position.x
    robot_states.y = msg.pose.pose.position.y
    _, _, robot_states.theta = euler_from_quaternion(
        [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
         msg.pose.pose.orientation.w])

def lidar_info(msg):
    global robot_states
    print(msg)
    robot_states.angle_range = [msg.angle_min, msg.angle_max]
    robot_states.distance_range = [msg.range_min, msg.range_max]
    robot_states.lidar_distances = msg.ranges
    robot_states.intensities = msg.intensities

def waypoint_callback(msg):
    global robot_states
    if not robot_states.has_waypoint:
        robot_states.target_goal_x = msg.point.x
        robot_states.target_goal_y = msg.point.y
        robot_states.has_waypoint = True

def f_att(eta, current_x, current_y, target_x, target_y):
    return [-eta * (target_x - current_x), -eta*(target_y - current_y)]

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

def main():
    global robot_states
    rospy.init_node("rosbot_gotogoal")
    odom_sub = rospy.Subscriber("/odom", Odometry, odom_info)
    waypoint_sub = rospy.Subscriber("/furthest_gap", PointStamped, waypoint_callback) 
    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(robot_states.running_rate)
    velocity = Twist()
    velocity.linear.x = 0.0
    count = 0

    while not rospy.is_shutdown():
        # # set target goal to be relative to robot's initial position
        # if count == 0:
        #     robot_states.target_goal_x = robot_states.target_goal_x + robot_states.x
        #     robot_states.target_goal_y = robot_states.target_goal_y + robot_states.y
        #     robot_states.theta_init = robot_states.theta
        #
        # count += 1
        if not robot_states.has_waypoint:
            continue

        # get updated distance from robot to goal
        distance_from_goal = math.sqrt((robot_states.target_goal_x - robot_states.x)**2 +
                                       (robot_states.target_goal_y - robot_states.y)**2)

        if distance_from_goal < 0.01 :
            robot_states.velocity = 0
            robot_states.angular_velocity = 0
            robot_states.has_waypoint = False
        else:
            # robot velocity pid control
            f_att = [-robot_states.eta * (robot_states.x - robot_states.target_goal_x),
                     -robot_states.eta * (robot_states.y - robot_states.target_goal_y)]
            print(f_att)
            f_rep = frep()

            f_total = f_att + f_rep # force vector representing combined forces

            dt = 1.0 / robot_states.running_rate
            next_point_x = robot_states.x + f_total[0] * dt
            next_point_y = robot_states.y + f_total[1] * dt

            # P controller for now
            error_from_next_point = distance(robot_states.x, robot_states.y, next_point_x, next_point_y)
            robot_states.integral = robot_states.integral + error * dt
            robot_states.velocity = robot_states.kp * error_from_next_point
            # robot_states.velocity = robot_states.kp * error_from_next_point + robot_states.ki * robot_states.integral


            # robot angle to goal point
            theta_d = math.atan2((next_point_y - robot_states.y),
                                 (next_point_x - robot_states.x))

            # angle error
            e  = theta_d - (robot_states.theta)
            # robot angle proportional controller: faster turn movement
            gamma = math.atan2(math.sin(e), math.cos(e))

            robot_states.angular_velocity = gamma
            print("Linear Velocity: ", robot_states.velocity)
            print("Angular Velocity: ", robot_states.angular_velocity)
            print("Distance to Goal: ", distance_from_goal)

        # set new velocity to publish
        velocity.linear.x = robot_states.velocity
        velocity.angular.z = robot_states.angular_velocity
        vel_pub.publish(velocity)
        rate.sleep()


if __name__ == "__main__":
    main()


