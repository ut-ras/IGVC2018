#!/usr/bin/env python
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

def broadcast_tf(msg):
	br = tf2_ros.TransformBroadcaster()
	t = geometry_msgs.msg.TransformStamped()
	odom = Odometry()

	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "chassis"
	t.child_frame_id = "lane_laser_frame"
	t.transform.translation.x = 0.15
	t.transform.translation.y = 0
	t.transform.translation.z = 0.1

	###
	# The following does not work for whatever reason :/
	###

	t.transform.rotation.x = odom.pose.pose.orientation.x
	t.transform.rotation.y = odom.pose.pose.orientation.y
	t.transform.rotation.z = odom.pose.pose.orientation.z
	t.transform.rotation.w = 1

	br.sendTransform(t)

	print('I am working')

def odom_callback(msg):
	# Set data to msg
	data = msg

	# Get scan data
	broadcast_tf(data)

rospy.init_node('lane_laser_broadcaster')

rospy.Subscriber("/odom", Odometry, odom_callback)

try:
	rospy.spin()
except KeyboardInterrupt:
	print("Shutting down")


