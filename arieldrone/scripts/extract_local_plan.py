#! /usr/bin/python

import rospy
from nav_msgs.msg import Path

def callback(msg):
	a = msg.poses.pose.position.x
	print(a)


rospy.init_node('extract_local_plan_node')
sub = rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan', Path, callback)
rospy.spin()

