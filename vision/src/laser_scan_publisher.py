#!/usr/bin/env python

import roslib
import rospy
import cv2
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

def append_data_list(image):
	e = float(2.718281828459)
	random_value = 10

	num_readings = 400
	laser_frequency = 40
	x = float(220) # 1 meter = 220 pixels

	pixels_to_meters = 1/x

	current_time = rospy.Time.now()
	rows = image.shape[0]
	cols = image.shape[1]

	scan = LaserScan()

	scan.header.stamp = current_time
	scan.header.frame_id = 'lane_laser_frame'
	scan.angle_min = -1.57
	scan.angle_max = 1.57
	scan.angle_increment = 3.14 / num_readings
	scan.time_increment = 0.0003018708
	scan.range_min = 0.0
	scan.range_max = 100.0

	scan.ranges = []
	# scan.intensities = []

	# cv2.imshow('testing', image)

	# for i in range(0, rows):
	# 	for j in range(previous_pixel_col, cols):
	# 		pixel = image[i,j]
	# 		#print(pixel[0])
	# 		if(pixel == 255):				##### BLUE GREEN RED ENCODING
	# 			#print("Obstacle found!")
	# 			scan.ranges.append((j*pixels_to_meters))
	# 			previous_pixel_col = j
	# 			break
	# 		if(image[i, previous_pixel_col - 1] == 0):
	# 			direction = 1
	# 		elif(image[i, previous_pixel_col - 1] == 255):
	# 			direction = -1
	# 		if(j == cols - 1):
	# 			scan.ranges.append(scan.range_max)
	
	previous_pixel_col = 0
	col = previous_pixel_col

	first_row = True
	obstacle_found = False

	row = 0

	while(row < rows):
		# Counter variables for when obstacle is not found

		col_counter = 1
		sign = 1

		# Main Loop

		while(not first_row and not obstacle_found):
			while(col < cols and col_counter <= 20):
				print(row)		# Debugging
				print(col)		# Debugging
				if(image[row,col] == 255):		# 255 == White
					obstacle_found = True
					scan.ranges.append((col*pixels_to_meters))
					previous_pixel_col = col
					break
				else:
					col = previous_pixel_col + sign * col_counter
					sign = sign * -1

					if(col >= col + 20 or col <= col + 20):		# End of range
						scan.ranges.append(scan.range_max)
				if(sign == 1):
				    col_counter += 1

		#loop to get first obstacle

		while(col < cols and first_row):
			if(image[row,col] == 255):
				previous_pixel_col = col
				first_row = False			# Set first_row condition such that this loop never runs after first row
				break
			col += 1
		row += 1

	scan_pub.publish(scan)

def image_callback(msg):
	# Set timer
	last_time = time.time()

	# Set img to msg data
	img = msg

	# Convert ROS image to CV format
	cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")

	# Get scan data
	append_data_list(cv_image)

	# If 'q' is detected, quit
	if(chr(cv2.waitKey(3)&255) == 'q'):
		rospy.signal_shutdown("Quit detected")

	# Print FPS
	print("{} fps".format(1/(time.time() - last_time)))

bridge = CvBridge()

# Init node
rospy.init_node('lane_laser_publisher')

# Create publisher to publish scan data
scan_pub = rospy.Publisher('/mybot/camera1/laser_scan', LaserScan, queue_size = 50)

# Create subscriber to log polar image topic
rospy.Subscriber("/mybot/camera1/image_log_polar", Image, image_callback)

##### Putting this here so I don't forget:
##### Top to down is angle. Left to Right is radius

## Have to scale pixels -> m

### PSEUDOCODE ###
# for cols in image:
#     last_time = time.time()
#     for rows in image:
#         if found red:
#             append to ranges list as obstacle
#     print(time.time() - last_time)            # Only need to do this once to figure out time increments

# obstacle_ranges = []
# obstacle_angle = []

try:
	rospy.spin()
except KeyboardInterrupt:
	print("Shutting down")

