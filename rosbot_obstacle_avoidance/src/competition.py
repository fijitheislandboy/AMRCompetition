#!/usr/bin/env python
from __future__ import print_function
import rospy
import random
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray


waypoint_pub = None

def gapcb(msg):
    gapList = msg.data

    stride0 = msg.layout.dim[0].stride
    stride1 = msg.layout.dim[1].stride
    num_gaps = msg.layout.dim[0].size
    cols = msg.layout.dim[1].size

    labels = ["start", "end"]
    for i in range(num_gaps):
        k = 0
        print("gap " + str(i + 1) + " " + ": ")
        for j in range(cols):
            if j % 2 == 1:
                print(str(gapList[stride0 * i + stride1 * j]) + ")")
            else:
                print("\t" + str(labels[k]) + ": (" + str(gapList[stride0 * i + stride1 * j]) + ", ", end="")
                k += 1

def area(x1, y1, x2, y2, x3, y3):
    return math.abs(((x1 * (y2-y3)) + (x2 * (y3-y1)) + (x3 * (y1 - y2))) /2.0))

def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def gap_callback(msg):
    global robot_states, waypoint_pub
    num_gaps = msg.layout.dim[0].size
    stride = msg.layout.dim[0].stride
    robot_states.gap_list = []
    for i in range(num_gaps):
        center_point = PoseStamped()
        center_point.header.frame_id = "odom"

        # point A - edge of gap
        x_1 = msg.data[i * stride]
        y_1 = msg.data[i * stride + 1]

        # point B - other edge of gap
        x_2 = msg.data[i*stride+2]
        y_2 =  msg.data[i*stride+3]

        # point C - robot position
        x_3 = robot_states.x
        y_3 = robot_states.y

        # point P - goal
        x_g = robot_states.target_goal_x
        y_g = robot_states.target_goal_y

        # determine if the robot can move through the gap
        if distance(x1,y1,x2,y2) < max(robot_states.width, robot_states.length):
            continue

        # determine if the goal is within the triangle made by the robot and the edges of the gap
        # first link on geeksforgeeks describing this

        A = area(x1, y1, x2, y2, x3, y3) # area of triangle ABC
        A1 = area(x_g, y_g, x_2, y_2, x_3, y_3) # area of triangle PBC
        A2 = area(x1, y1, x_g, y_g, x3, y3) # area of triangle PAC
        A3 = area(x1, y1, x2, y2, x_g, y_g) # area of triangle PAB

        # check if point inside triangle
        if A == (A1 + A2 + A3):
            center_point.pose.position.x = x_g
            center_point.pose.position.y = y_g
        else:
            center_point.pose.position.x = (msg.data[i * stride] + msg.data[i*stride+2]) / 2
            center_point.pose.position.y = (msg.data[i*stride+1] + msg.data[i*stride+3]) / 2
            gap_yaw = math.atan2(msg.data[i*stride+3] - msg.data[i*stride+1], msg.data[i*stride+2] - msg.data[i*stride]) + math.pi / 2
            center_point.pose.orientation.z = math.sin(gap_yaw / 2)
            center_point.pose.orientation.w = math.cos(gap_yaw / 2)
        waypoint_pub.publish(center_point)

        robot_states.gap_list.append(center_point)


class RobotStates:
    def __init__(self):
        # robot rate at which commands get run
        self.running_rate = 10

        # go-to-goal target positions and angle
        self.target_goal_x = 6.0
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

        # list of gaps
        self.gap_list = []
        self.gap_history_stack = [] # for backtracking in case we need it

        # intermediate goal
        self.intermediate_goal_x = None
        self.intermediate_goal_y = None

        # robot physical info, in (cm)
        self.length = 20
        self.width = 23


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

def gotogoal(x, y):
    return


"""
Game Plan:
1. Find open gaps and determine if they are usable for the robot to go through.
    a. If goal is within gap distance and is open space, go to goal.
    b. If goal is not within gap distance, select gap closest towards the goal. Could either do by:
        i. Distance
        ii. Angle
2. Head towards gap using go to goal on middle of gap, with orientation perpendicular to gap pointing away from robot original position.
3. Repeat.

-- may need stack of poses for backtracking purposese.
"""
def main():
    global robot_states, waypoint_pub
    rospy.init_node("rosbot_obstacle_avoidance")

    # reset pose
    reset_pose = PoseWithCovarianceStamped()
    reset_pub = rospy.Publisher("/set_pose", PoseWithCovarianceStamped, queue_size=1)
    reset_pub.publish(reset_pose)

    gap_sub = rospy.Subscriber("/gaps", Float32MultiArray, gap_callback)
    waypoint_pub = rospy.Publisher("/furthest_gap", PoseStamped, queue_size=1)

    while True:
        gap_center_points = robot_states.gap_list

        best_distance = -1
        best_point_index = -1

        if robot_states.intermediate_goal_x is not None:
            # if the robot has reached the intermediate goal, find a new goal
            if abs(robot_states.x - robot_states.intermediate_goal_x) < 0.1 and
                abs(robot_states.y - robot_states.intermediate_goal_y) < 0.1:

                # if robot has found gaps
                if len(gap_center_points) != -1:

                    # find gap center point closest to goal, orient for that
                    for i in range(0, len(gap_center_points)):
                        x = gap_center_points[i].pose.position.x
                        y = gap_center_points[i].pose.position.y

                        distance_from_goal = distance(x, y, robot_states.target_goal_x, robot_states.target_goal_y)
                        if best_distance == -1:
                            best_distance = distance_from_goal
                            best_point_index = -1
                        else:
                            if distance_from_goal < best_distance:
                                best_distance = distance_from_goal
                                best_point_index = i

                    # gotogoal(gap_center_points[best_point_index].pose.position.x,
                    #          gap_center_points[best_point_index].pose.position.y)
                else:
                    print("backtrack?")



        rospy.spin()

if __name__ == "__main__":
    main()


