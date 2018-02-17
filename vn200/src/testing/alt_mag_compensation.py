	# previous_min = 0
	# previous_max = 0

	# while(loop):
	# 	rospy.init_node('imu_listener', anonymous = True)
	# 	imu_sub = rospy.Subscriber("vn200_accel_gyro_compass", vn_200_accel_gyro_compass, sampling_callback)

	# 	global imu_msg

	# 	if imu_msg == None:
	# 		return

	# 	mag_x = imu_msg.compass.x
	# 	mag_y = imu_msg.compass.y
	# 	mag_z = imu_msg.compass.z

	# 	mag = []

	# 	mag[0] = imu_msg.compass.x
	# 	mag[1] = imu_msg.compass.y
	# 	mag[2] = imu_msg.compass.z




	# for i in range(10000):
	# 	rospy.init_node('imu_listener', anonymous = True)
	# 	imu_sub = rospy.Subscriber("vn200_accel_gyro_compass", vn_200_accel_gyro_compass, sampling_callback)

	# 	global imu_msg

	# 	global mag_min
	# 	global mag_max

	# 	if imu_msg==None:
	# 		return

	# 	mag_x = imu_msg.compass.x
	# 	mag_y = imu_msg.compass.y
	# 	mag_z = imu_msg.compass.z

	# 	if mag_x < mag_x_previous_min:
	# 		mag_x_min = mag_x
	# 	elif mag_x >  mag_x_previous_max:
	# 		mag_x_max = mag_x

	# 	if mag_y < mag_y_previous_min:
	# 		mag_y_min = mag_y
	# 	elif mag_y >  mag_y_previous_max:
	# 		mag_y_max = mag_y

	# 	if mag_z < mag_z_previous_min:
	# 		mag_z_min = mag_z
	# 	elif mag_z >  mag_z_previous_max:
	# 		mag_z_max = mag_z

	# 	mag_x_previous_min = mag_x_min
	# 	mag_y_previous_min = mag_y_min
	# 	mag_z_previous_min = mag_z_min

	# 	mag_x_previous_min = mag_x_max
	# 	mag_y_previous_min = mag_y_max
	# 	mag_z_previous_min = mag_z_max