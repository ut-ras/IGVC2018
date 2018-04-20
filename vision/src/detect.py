#!/usr/bin/env python
#

import roslib
import rospy
import cv2
import timeit
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def process_img_Daniel(img):
    #Set margins for isolation of the color white
    sensitivity = 20  #variable sensitivity value (can be tuned further down the line)
    whiteLower = (0, 0, 255 - sensitivity)
    whiteUpper = (255, sensitivity, 255)

    #Read image
    start = timeit.default_timer()  #clock algorithm start
    img = cv2.resize(img, (640, 360))  #resize image to decrease overhead
    #cv2.imshow('original',img)                         #debugging

    # Create the basic black image
    white_rec = np.zeros(img.shape, dtype="uint8")
    # Draw a white, filled rectangle on the mask image
    cv2.rectangle(white_rec, (0, 100), (720, 500), (255, 255, 255), -1)

    #cv2.imshow('white rectangle',white_rec)            #debugging

    imgROI = cv2.bitwise_and(
        img, white_rec)  #get rid of the top part to decrease overhead (O(1))
    #cv2.imshow('masked img',imgROI)                    #debugging

    imgHSV = cv2.cvtColor(
        imgROI, cv2.COLOR_BGR2HSV
    )  #change color scheme from BGR to HSV for easier color isolation (time complexity O(N^3)))

    # Create a image that filters out every color besides white
    mask = cv2.inRange(imgHSV, whiteLower,
                       whiteUpper)  #time complexity estimate O(N^3)

    #helper variables for masking the part of the image immediately in front of the robot
    rows = imgHSV.shape[0]
    cols = imgHSV.shape[1]
    offset = 50  #offset for drawing the polygon
    mask_immediate_front = np.array(
        [[[cols / 5, rows / 2], [cols * 4 / 5, rows / 2],
          [cols - offset, rows], [offset, rows]]],
        dtype=np.int32)

    cv2.fillPoly(mask, mask_immediate_front,
                 0)  #create the polygon on the mask
    #cv2.imshow('mask',mask)                            #debugging

    res = cv2.bitwise_and(
        imgROI, imgROI, mask=mask
    )  #apply the mask to the pre-processed image (time complexity O(1))
    #cv2.imshow('res',res)                              #debugging code

    #Select hysteresis thresholds for Canny edge detection algorithm
    low_threshold = 100
    high_threshold = 200

    #run canny edge detection algorithm
    edges = cv2.Canny(
        res, low_threshold, high_threshold
    )  #notes: Convolutions using Fast Fourier Transform are O(n*log(n)).
    #       For an m by n image, complexity is O(mn(log(mn))).
    #       This is the bottleneck in the pipeline
    #cv2.imshow('edges',edges)                          #debugging code

    #Select parameters for probabilistic Hough lines transform
    rho = 1  # distance resolution in pixels of the Hough grid
    theta = np.pi / 180  # angular resolution in radians of the Hough grid
    threshold = 15  # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 20  # minimum number of pixels making up a line
    max_line_gap = 50  # maximum gap in pixels between connectable line segments
    line_image = np.copy(res) * 0  # creating a blank to draw lines on

    # Run Hough on edge detected image
    # Output "lines" is an array containing endpoints of detected line segments **** NOTE: "Navigation" will use these data points ****
    lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                            min_line_length, max_line_gap)

    stop = timeit.default_timer()  #clock algorithm end

    #for demo purposes...
    #iterate through to create the Hough lines on top of the data set returned by the Hough transform
    if (lines is not None):
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), (255, 50, 50), 5)

    #add the lines on top of the original image for display
    lines_edges = cv2.addWeighted(img, 0.8, line_image, 1, 0)

    #cv2.imshow('hough image', res)                         # debugging code

    time = str((stop - start) * 1000)
    print(time + ' ms')

    return lines_edges


## Processes image and returns image with drawn lines
#  @type image: Mat
#  @param image input image to process
#
#  @return image with detected edges
def process_img_old(image):
    # Keep a safe copy of the original image
    original_image = image

    # convert the image to Grayscale
    processed_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Edge detection: cv2.Canny(image to run detection, lower bound of intensity to ignore, upper bound of intensity to consider an edge)
    edgeIntensityMin = 200
    edgeIntensityMax = 300
    processed_img = cv2.Canny(
        processed_img,
        threshold1=edgeIntensityMin,
        threshold2=edgeIntensityMax)

    # Gaussian Blur: cv2.GaussianBlur(image to run blur on, kernel size, sigma)
    processed_img = cv2.GaussianBlur(processed_img, (5, 5), 0)

    # Define a blank mask
    masking = np.zeros_like(processed_img)

    # Helper variables
    rows = image.shape[0]
    cols = image.shape[1]

    # Define points of the region of interest, in this case we defined a trapezoid for the lower half of the screen
    # It needs to be 32 bit for masking to work correctly
    roi = np.array(
        [[[cols / 5, rows / 2], [cols * 4 / 5, rows / 2], [cols, rows],
          [0, rows]]],
        dtype=np.int32)

    # Actually draw the polygon now
    cv2.fillPoly(masking, roi, 255)
    # cv2.imshow("mask", masking) # debugging code

    # Apply mask to the original image we mask here so we don't get weird edges created by masking the original image
    processed_img = cv2.bitwise_and(processed_img, processed_img, mask=masking)

    # Hough Line Transforms
    lines = cv2.HoughLinesP(processed_img, 1, np.pi / 180, 180, 20, 15)

    # Draw the lines on the image
    draw_lines(original_image, lines)

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
        cv2.line(image, (coords[0], coords[1]), (coords[2], coords[3]),
                 [0, 0, 255], 3)


## ROS callback for subscriber image topic
# @param msg the image advertised from another node
#
def image_callback(msg):
    img = msg
    cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
    cv2.imshow("Image window", process_img_Daniel(cv_image))
    # If 'q' is detected, quit
    if (chr(cv2.waitKey(3) & 255) == 'q'):
        rospy.signal_shutdown("Quit detected")


bridge = CvBridge()

# Initialize a node in ROS
rospy.init_node('image_listener')

# Create a subscriber to read the published camera image
rospy.Subscriber("/camera/image_raw", Image, image_callback)

# Loop until we die
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
cv2.destroyAllWindows()
