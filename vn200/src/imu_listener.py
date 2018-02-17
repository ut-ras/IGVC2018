#!/usr/bin/env python

from vn200.msg import vn_200_accel_gyro_compass, vn_200_gps_soln, vn_200_ins_soln
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import rospy
import time
import math

imu_msg = None

gyr_xavg = float(0)
gyr_yavg = 0
gyr_zavg = 0

sample_size = 1000

comp_pitch = float(0)
comp_roll = float(0)
comp_yaw = float(0)

previous = 0
previous_yaw = float(0)

def callback(data):
	global imu_msg
	imu_msg = data

def sampling_callback(data):
	global gyr_xavg
	global gyr_yavg
	global gyr_zavg

	global imu_msg
	imu_msg = data

	RAD_TO_DEGREES = float(180 / math.pi)

	if imu_msg == None:
		return

	gyr_x = imu_msg.gyro.x * RAD_TO_DEGREES
	gyr_y = imu_msg.gyro.y * RAD_TO_DEGREES
	gyr_z = imu_msg.gyro.z * RAD_TO_DEGREES

	gyr_xavg = gyr_xavg + gyr_x
	gyr_yavg = gyr_yavg + gyr_y
	gyr_zavg = gyr_zavg + gyr_z

	#print(gyr_x)

def bound0to2pi(angle):
    return (angle%(2*math.pi) + 2*math.pi)%(2*math.pi) - math.pi

def offset_calc():
	global sample_size

	for i in range(sample_size):
		rospy.init_node('imu_listener', anonymous = True)
		imu_sub = rospy.Subscriber("vn200_accel_gyro_compass", vn_200_accel_gyro_compass, sampling_callback)

	global gyr_xavg
	global gyr_yavg
	global gyr_zavg

	gyr_xavg = gyr_xavg / sample_size
	gyr_yavg = gyr_yavg / sample_size
	gyr_zavg = gyr_zavg / sample_size

	print('success')

	#print(gyr_xavg)

def hard_iron_offset():
	buff = 1000

	loop = 1
	counter = [0,0,0,0,0,0]
	mag_appended_list = [[] for i in range(3)]

	current_mag = [0,0,0]
	mag_max = [0,0,0]
	mag_min = [0,0,0]
	previous_max = [0,0,0]
	previous_min = [0,0,0]

	# while not rospy.is_shutdown():
	# 	rospy.init_node('imu_listener', anonymous = True)
	# 	imu_sub = rospy.Subscriber("vn200_accel_gyro_compass", vn_200_accel_gyro_compass, callback)

	# 	global imu_msg

	# 	if imu_msg == None:
	# 		print('failed')
	# 		pass
	# 	else:
	# 		return

	while(loop):
		rospy.init_node('imu_listener', anonymous = True)
		imu_sub = rospy.Subscriber("vn200_accel_gyro_compass", vn_200_accel_gyro_compass, callback)

		global imu_msg

		if imu_msg == None:
			continue

		current_mag[0] = imu_msg.compass.x
		current_mag[1] = imu_msg.compass.y
		current_mag[2] = imu_msg.compass.z


		for j in range(3):
			mag_appended_list[j].append(current_mag[j])

			mag_max[j] = max(mag_appended_list[j])
			mag_min[j] = min(mag_appended_list[j])

		for j in range(3):
			if(mag_max[j] - previous_max[j]) == 0.0:
				counter[j] = counter[j] + 1
			elif(mag_max[j] - previous_max[j] != 0.0):
				counter[j] = 0
			previous_max[j] = mag_max[j]

			if(mag_min[j] - previous_min[j]) == 0.0:
				counter[j+3] = counter[j+3] + 1
			elif(mag_min[j] - previous_min[j] != 0.0):
				counter[j+3] = 0
			previous_min[j] = mag_min[j]

		if counter[0] > buff and counter[1] > buff and counter[2] > buff and counter[3] > buff and counter[4] > buff and counter[5] > buff:
			loop = 0

	print(mag_max)
	print(mag_min)


def listener():
	rospy.init_node('imu_listener', anonymous = True)

	imu_sub = rospy.Subscriber("vn200_accel_gyro_compass", vn_200_accel_gyro_compass, callback)
	#gps_sub = rospy.Subscriber("vn200_gps", vn_200_gps_soln, callback)
	#ins_sub = rospy.Subscriber("vn200_ins", vn_200_ins_soln, callback)

	#imu_msg = vn_200_accel_gyro_compass()
	global imu_msg
	global previous
	global comp_pitch
	global comp_roll
	global comp_yaw
	global previous_yaw
	#global gyr_xavg

	mag_x_correction = float(1.41999375)
	mag_y_correction = float(1.56595)
	mag_z_correction = float(-1.47125)

	RAD_TO_DEGREES = float(180 / math.pi)

	alpha = float(0.1)

	if imu_msg==None:
		return
	
	acc_x = imu_msg.accelerometer.x
	acc_y = imu_msg.accelerometer.y
	acc_z = imu_msg.accelerometer.z

	gyr_x = imu_msg.gyro.x
	gyr_y = imu_msg.gyro.y
	gyr_z = imu_msg.gyro.z

	#print(gyr_x)

	mag_x = imu_msg.compass.x - mag_x_correction
	mag_y = imu_msg.compass.y - mag_y_correction
	mag_z = imu_msg.compass.z - mag_z_correction

	pitch = math.atan2(acc_x, -acc_z)
	roll = math.atan2(acc_y, acc_z)

	roll = bound0to2pi(roll)

	current_time = rospy.get_time()
	dt = current_time - previous

	comp_pitch = alpha * (comp_pitch + gyr_y * dt) + (1-alpha) * pitch
	comp_roll = alpha * (comp_roll + gyr_x * dt) + (1-alpha) * roll


	YAW_CORRECTION = -math.pi/2

	x_h = mag_x * math.cos(comp_pitch) + mag_z * math.sin(comp_pitch)
	y_h = mag_x * math.sin(comp_roll) * math.sin(comp_pitch) + mag_y * math.cos(comp_roll) - mag_z * math.sin(comp_roll) * math.cos(comp_pitch)

	yaw = math.atan2(y_h, x_h)

	comp_yaw = alpha * (comp_yaw + gyr_z * dt) + (1-alpha) * yaw

	print(yaw * RAD_TO_DEGREES)

	#print(comp_pitch)
	previous = current_time
	previous_yaw = comp_yaw
	#print(gyr_xavg)

def visualization():
	pass

def init():
	#offset_calc()

	#hard_iron_offset()

	while not rospy.is_shutdown():
		listener()

if __name__ == '__main__':
	try:
		init()
	except rospy.ROSInterruptException:
		pass
