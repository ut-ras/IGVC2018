####
# Copying these here for testing purposes later . . .
####

# # # # # # # # #

	#roll = math.atan2(acc_y, math.sqrt(acc_x**2 + acc_z**2))
	#pitch = math.atan2(acc_x, math.sqrt(acc_y**2 + acc_z**2))

# # # # # # # # #

	#mag_norm = math.sqrt((mag_x * mag_x) + (mag_y * mag_y) + (mag_z * mag_z))
	#mag_x /= mag_norm
	#mag_y /= mag_norm
	#mag_z /= mag_norm

# # # # # # # # #

	#yaw = (math.atan2((-mag_y * math.cos(comp_roll)) + (mag_z * math.sin(comp_roll)),
	#	(mag_x * math.cos(comp_pitch)) + (mag_y * math.sin(comp_pitch)) * math.sin(comp_roll) +
	#	(mag_z * math.sin(comp_pitch) * math.cos(comp_roll)))) * RAD_TO_DEGREES

# # # # # # # # #

	#yaw = math.atan2(mag_y, mag_x)

	#yaw = math.atan2(-mag_y * math.cos(roll) + mag_z * math.sin(roll),
	#	mag_x * math.cos(pitch) + mag_z * math.sin(pitch) * math.sin(roll)
	#	+ mag_z * math.sin(pitch) * math.cos(roll))

	#magnetic_x = (mag_x * math.cos(comp_pitch)) + (mag_y * math.sin(comp_roll) * math.sin(comp_pitch))
	#+ (mag_z * math.cos(comp_roll) * math.sin(comp_pitch)) 
	#magnetic_y = (mag_y * math.cos(comp_roll)) - (mag_z * math.sin(comp_roll))

	#yaw = (math.atan2(mag_x, mag_y) * RAD_TO_DEGREES)