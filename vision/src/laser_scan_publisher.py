#!/usr/bin/env python

import roslib
import rospy
import cv2
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

def append_data_list(image):
	num_readings = 100
	laser_frequency = 40

	pixels_to_meters = 1/400

	count = 0

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
	scan.intensities = []

	# cv2.imshow('testing', image)

	for i in range(0, rows):
		for j in range(0, cols/4):
			pixel = image[i,j]
			#print(pixel[0])
			if(pixel[2] == 255):				##### BLUE GREEN RED ENCODING
				#print("Obstacle found!")
				scan.ranges.append(j*pixels_to_meters)
				break

	scan_pub.publish(scan)
	count += 1

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

