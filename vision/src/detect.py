#!/usr/bin/env python

import roslib
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# checkerboard dimensions (defined by corners)
# Must be asymmetrical
cbrow = 7
cbcol = 6

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((cbrow*cbcol,3), np.float32)
objp[:,:2] = np.mgrid[0:cbcol,0:cbrow].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Have to calibrate our cameras in real life.

def calibrate_camera(image):
    #last_time = time.time()
    img = image
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (cbcol,cbrow), None)
    print(loop)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (cbcol,cbrow), corners2, ret)

    #print('Frame took {} seconds'.format(time.time() - last_time))
    

    return img

# Add light threshold to select white pixels only
# Increase frames from ~3 FPS to ~20 FPS

def select_white(image):
    # Keep a safe copy of the original image
    original_image = image

    # Convert image to HSL
    processed_img = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

    # white color mask
    lower = np.uint8([0,160,0])
    upper = np.uint8([255,255,255])
    white_mask = cv2.inRange(processed_img, lower, upper)

    # return mask
    return cv2.bitwise_and(original_image, original_image, mask = white_mask)

## Processes image and returns image with drawn lines
#  @type image: Mat
#  @param image input image to process
#  
#  @return image with detected edges
def process_img(image):
    # Track frames per second
    # last_time = time.time()

    # Keep a safe copy of the original image
    original_image = image

    # Retrieve image after applying white masking
    white_img = select_white(image)
    
    # convert the image to Grayscale
    gray_img = cv2.cvtColor(white_img, cv2.COLOR_BGR2GRAY)

    # Edge detection: cv2.Canny(image to run detection, lower bound of intensity to ignore, upper bound of intensity to consider an edge)
    edgeIntensityMin = 100
    edgeIntensityMax = 200
    canny_img = cv2.Canny(gray_img, threshold1 = edgeIntensityMin, threshold2 = edgeIntensityMax)
    
    # Gaussian Blur: cv2.GaussianBlur(image to run blur on, kernel size, sigma)
    processed_img = cv2.GaussianBlur(canny_img, (5,5), 0)

    # Define a blank mask
    masking = np.zeros_like(processed_img)
    
    # Helper variables
    rows = image.shape[0]
    cols = image.shape[1]
    
    # Define points of the region of interest, in this case we defined a trapezoid for the lower half of the screen
    # It needs to be 32 bit for masking to work correctly
    roi = np.array([[[0, rows/2], [cols, rows/2], [cols, rows], [0, rows]]], dtype = np.int32)
    # roi = np.array([[[cols/3, rows/2], [cols*2/3, rows/2], [cols*12, rows/1.465], [cols/12, rows/1.465]]], dtype = np.int32)
    
    # Actually draw the polygon now
    cv2.fillPoly(masking, roi, 255)
    # cv2.imshow("mask", masking) # debugging code

    # Apply mask to the original image we mask here so we don't get weird edges created by masking the original image
    processed_img = cv2.bitwise_and(processed_img, processed_img, mask=masking)

    # Hough Line Transforms
    lines = cv2.HoughLinesP(processed_img, 1, np.pi/180, 180, np.array([]), 10, 20)
    
    # Draw the lines on the image
    draw_lines(original_image, lines)

    # Convert to polar Coordinates
    original_image = convert_to_polar(original_image)

    # Print frame timestamp
    # print("This frame took {} seconds".format(time.time() - last_time))

    return original_image

## Helper function to draw lines on an image
# @param image the image to draw on
# @param lines output from Hough Line Transform to draw onto image
def draw_lines(image, lines):
    # Prevents some errors if there were no lines detected
    if lines is None:
        return

    # Draw detected lines on the image (passed by reference)
    for line in lines:
        coords = line[0]
        cv2.line(image, (coords[0], coords[1]), (coords[2], coords[3]), [0, 0, 255], 3)

def convert_to_polar(image):
    # Save copy of image just in case
    original_image = image

    # Obtain row/column data from image
    rows = image.shape[0]
    cols = image.shape[1]

    # Set source points (trapezoidal)
    src = np.float32([
        [(cols / 2) - 180, (rows / 2)],
        [(cols / 6) - 120, (rows)],
        [(cols * 5/6) + 120, rows],
        [(cols / 2) + 180, (rows / 2)]])

    # Set destination points -- What do you want to transform your image to? 
    # In this case skewed 3D -> Planar 2D
    dst = np.float32([
        [0, 0],
        [0, rows],
        [(cols), rows],
        [(cols), 0]])

    # First retrieve perspective transform matrix
    M = cv2.getPerspectiveTransform(src, dst)

    # Apply matrix to original image
    warp_img = cv2.warpPerspective(image, M, (cols, rows))

    # Convert to log_polar format 
    log_polar_img = cv2.logPolar(warp_img, (rows / 2, cols / 2), 30, cv2.WARP_FILL_OUTLIERS)

    # Used for debugging purposes
    cv2.imshow("Warped Image", warp_img)
    # cv2.imshow("Log Polar image", log_polar_img)

    return log_polar_img


## ROS callback for subscriber image topic
# @param msg the image advertised from another node
# 
def image_callback(msg):
    # Set img to msg data
    img = msg

    # Convert ROS image to CV format
    cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
    cv2.imshow("Image window", process_img(cv_image))

    # Convert back to CV and publish
    ros_image = bridge.cv2_to_imgmsg(process_img(cv_image), encoding="passthrough")
    log_image_pub.publish(ros_image)

    # If 'q' is detected, quit
    if(chr(cv2.waitKey(3)&255) == 'q'):
        rospy.signal_shutdown("Quit detected")

bridge = CvBridge()

# Initialize a node in ROS
rospy.init_node('image_listener')

# Create publisher to publish log polar image
log_image_pub = rospy.Publisher('/mybot/camera1/image_log_polar', Image, queue_size = 10)

# Create a subscriber to read the published camera image
rospy.Subscriber("/mybot/camera1/image_raw", Image, image_callback)

# Loop until we die
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
cv2.destroyAllWindows()
