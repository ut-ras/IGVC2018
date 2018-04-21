#!/usr/bin/env python
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import numpy as np
from nav_msgs.msg import Odometry

def broadcast_tf(msg):
	br = tf2_ros.TransformBroadcaster()
	t = geometry_msgs.msg.TransformStamped()
	odom = msg

	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "chassis"
	t.child_frame_id = "lane_laser_frame"

	# Update this later depending on position of camera picture IRL
	t.transform.translation.x = 0.15
	t.transform.translation.y = 0
	t.transform.translation.z = 0.1

	###
	# EDIT: The following almost works. Have to read up on quaternions.
	# Might use euler angles first then convert to quats
	###

	# Retrieve orientation data
	orientation = odom.pose.pose.orientation

	# Then convert to numpy array
	quat = np.array([])
	quat = np.append(quat, orientation.x)
	quat = np.append(quat, orientation.y)
	quat = np.append(quat, orientation.z)
	quat = np.append(quat, orientation.w)

	# Convert to Euler
	euler = tf.transformations.euler_from_quaternion(quat)

	# Cause tuples can't receive item assignment apparently
	euler = list(euler)

	# Rotates the exact opposite of RAScal's orientation in order to stay fixed
	#euler[2] = euler[2] * -1
	for i in range(3):
		euler[i] *= -1

	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]

	# Convert back to quaternion
	updated_quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

	# Update transform publisher
	t.transform.rotation.x = updated_quat[0]
	t.transform.rotation.y = updated_quat[1]
	t.transform.rotation.z = updated_quat[2]
	t.transform.rotation.w = updated_quat[3]

	# Send the 1s and 0s
	br.sendTransform(t)

	print('Roll: %.3g' % euler[0], 'Pitch: %.3g' % euler[1], 'Yaw: %.3g' % euler[2])

	#print(updated_quat)
	#print(odom.pose.pose.orientation.z)

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


