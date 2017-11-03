#!/usr/bin/env python

from vn200.msg import vn_200_accel_gyro_compass, vn_200_gps_soln, vn_200_ins_soln
import rospy
import time

def callback(data):
	pass

def listener():
	rospy.init_node('listener', anonymous = True)

	imu_sub = rospy.Subscriber("vn200_accel_gyro_compass", vn_200_accel_gyro_compass, callback)
	gps_sub = rospy.Subscriber("vn200_gps", vn_200_gps_soln, callback)
	ins_sub = rospy.Subscriber("vn200_ins", vn_200_ins_soln, callback)

	rospy.spin()

def visualization():



if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
