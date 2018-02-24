#!/usr/bin/env python

from vn200.msg import vn_200_accel_gyro_compass, vn_200_gps_soln, vn_200_ins_soln
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import rospy
import time
import math

imu_msg = None

def callback(data):
	global imu_msg
	imu_msg = data

def listener():
	lat_avg = float(0)
	long_avg = float(0)

	min_latitude = float(100)
	min_longitude = float(-80)

	max_latitude = float(0)
	max_longitude = float(-100)


	rospy.init_node('gps_testing', anonymous = True)

	#imu_sub = rospy.Subscriber("vn200_accel_gyro_compass", vn_200_accel_gyro_compass, callback)
	#gps_sub = rospy.Subscriber("vn200_gps", vn_200_gps_soln, callback)
	#ins_sub = rospy.Subscriber("vn200_ins", vn_200_ins_soln, callback)

	#imu_msg = vn_200_accel_gyro_compass()

	for i in range(0,5000):
		gps_sub = rospy.Subscriber("vn200_gps", vn_200_gps_soln, callback)

		global imu_msg

		if imu_msg==None:
			return

		latitude = imu_msg.latitude
		longitude = imu_msg.longitude

		lat_avg = lat_avg + latitude
		long_avg = long_avg + longitude

		if (latitude > max_latitude):
			max_latitude = latitude
		if (longitude > max_longitude):
			max_longitude = longitude

		if (latitude < min_latitude):
			min_latitude = latitude
		if (longitude < min_longitude):
			min_longitude = longitude

	lat_avg = lat_avg / 5000
	long_avg = long_avg / 5000

	print("")

	#print("Lat avg: %3.9f" % lat_avg)
	#print("Long avg: %3.9f" % long_avg)
	#print("Max lat: %3.9f" % max_latitude)
	#print("Max long: %3.9f" % max_longitude)
	#print("Min lat: %3.9f" % min_latitude)
	#print("Min Long: %3.9f" % min_longitude)

	#print("")

	# Coordinates in decimal degrees (e.g. 2.89078, 12.79797)
	
	lon1 = min_longitude
	lon2 = max_longitude

	lat1 = min_latitude
	lat2 = max_latitude

	R = 6372880  # radius of Earth in meters
	phi_1 = math.radians(lat1)
	phi_2 = math.radians(lat2)

	delta_phi = math.radians(lat2 - lat1)
	delta_lambda = math.radians(lon2 - lon1)

	a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi_1) * math.cos(phi_2) * math.sin(delta_lambda / 2.0) ** 2
	
	c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

	meters = R * c  # output distance in meters
	km = meters / 1000.0  # output distance in kilometers

	meters = round(meters, 3)
	km = round(km, 3)

	print("Distance: %3.9f m" % meters)
	print("Distance: %3.9f km" % km)

	rospy.signal_shutdown("quitting")

if __name__ == '__main__':
	try:
		while not rospy.is_shutdown():
			listener()
	except rospy.ROSInterruptException:
		pass






