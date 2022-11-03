#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
import math
import numpy as np
from tf.transformations import euler_from_quaternion

waypoint_pub = None
final_goal = (20, 0)
current_goal = (0, 0)
publish_goal = False

def gap_callback(msg):
    global waypoint_pub, final_goal, current_goal, publish_goal
    num_gaps = msg.layout.dim[0].size
    stride = msg.layout.dim[0].stride
    for i in range(num_gaps):
        center_point = [-1* (msg.data[i * stride] + msg.data[i*stride+2]) / 2, -1 * (msg.data[i*stride+1] + msg.data[i*stride+3]) / 2] # for some reason lidar is flipped?
        if np.linalg.norm([center_point[0] - final_goal[0], center_point[1] - final_goal[1]]) < np.linalg.norm([current_goal[0] - final_goal[0], current_goal[1] - final_goal[1]]):
            current_goal = center_point
            publish_goal = True
        #gap_yaw = math.atan2(msg.data[i*stride+3] - msg.data[i*stride+1], msg.data[i*stride+2] - msg.data[i*stride]) + math.pi / 2
        #center_point.pose.orientation.z = math.sin(gap_yaw / 2)
        #center_point.pose.orientation.w = math.cos(gap_yaw / 2)
    if publish_goal:
        msg = PointStamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = rospy.Time.now()
        msg.point.x = current_goal[0]
        msg.point.y = current_goal[1]
        waypoint_pub.publish(msg)
        publish_goal = False


def main():
    global waypoint_pub
    rospy.init_node("furthest_gap_publisher")
    gap_sub = rospy.Subscriber("/gaps", Float32MultiArray, gap_callback)
    waypoint_pub = rospy.Publisher("/furthest_gap", PointStamped, queue_size=1)
    rospy.spin()

if __name__ == "__main__":
    main()

