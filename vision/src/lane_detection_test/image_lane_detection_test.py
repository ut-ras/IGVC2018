###This is a test for lane detection using a still image and openCV
## Created by: Daniel Diamont
## Created: 2/15/2018
## Last: 2/16/2018

## TODO:
# 1) Test this on different images
# 2) Select a capture rate that does not produce too much overhead (2-5 Hz)
# 3) Remove line drawing code
# 4) Merge mask operations to reduce time complexity

##import openCV packages, numpy, and timeit for testing
import cv2
import numpy as np
import timeit

#Set margins for isolation of the color white
sensitivity = 20									#variable sensitivity value (can be tuned further down the line)
whiteLower = (0,0,255-sensitivity)
whiteUpper = (255,sensitivity,255)

#Read image
start = timeit.default_timer()						#clock algorithm start
img = cv2.imread('igvc_lanes.jpg')
img = cv2.resize(img,(640,360))						#resize image to decrease overhead
#cv2.imshow('original',img)							#debugging

# Create the basic black image 
white_rec = np.zeros(img.shape, dtype = "uint8")
# Draw a white, filled rectangle on the mask image
cv2.rectangle(white_rec, (0, 100), (720, 500), (255, 255, 255), -1)

#cv2.imshow('white rectangle',white_rec)			#debugging

imgROI = cv2.bitwise_and(img,white_rec)				#get rid of the top part to decrease overhead (O(1))
#cv2.imshow('masked img',imgROI)					#debugging

imgHSV = cv2.cvtColor(imgROI, cv2.COLOR_BGR2HSV)	#change color scheme from BGR to HSV for easier color isolation (time complexity O(N^3)))

# Create a image that filters out every color besides white
mask = cv2.inRange(imgHSV, whiteLower, whiteUpper)	#time complexity estimate O(N^3)

#helper variables for masking the part of the image immediately in front of the robot
rows = imgHSV.shape[0]
cols = imgHSV.shape[1]
offset = 50											#offset for drawing the polygon
mask_immediate_front = np.array([[[cols/5, rows/2], [cols * 4/5, rows/2], [cols-offset, rows], [offset, rows]]], dtype = np.int32)

cv2.fillPoly(mask,mask_immediate_front,0)			#create the polygon on the mask
#cv2.imshow('mask',mask)							#debugging

res = cv2.bitwise_and(imgROI,imgROI, mask= mask)	#apply the mask to the pre-processed image (time complexity O(1))
#cv2.imshow('res',res)								#debugging code

#Select hysteresis thresholds for Canny edge detection algorithm
low_threshold = 100
high_threshold = 200

#run canny edge detection algorithm
edges = cv2.Canny(res,low_threshold,high_threshold)	#notes: Convolutions using Fast Fourier Transform are O(n*log(n)).
													#		For an m by n image, complexity is O(mn(log(mn))).
													#		This is the bottleneck in the pipeline
#cv2.imshow('edges',edges)							#debugging code

#Select parameters for probabilistic Hough lines transform
rho = 1 											 # distance resolution in pixels of the Hough grid
theta = np.pi/180 									 # angular resolution in radians of the Hough grid
threshold = 15  									 # minimum number of votes (intersections in Hough grid cell)
min_line_length = 20 								 # minimum number of pixels making up a line
max_line_gap = 50  									 # maximum gap in pixels between connectable line segments
line_image = np.copy(res) * 0  						 # creating a blank to draw lines on

# Run Hough on edge detected image
# Output "lines" is an array containing endpoints of detected line segments **** NOTE: "Navigation" will use these data points ****
lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
            min_line_length, max_line_gap)

stop = timeit.default_timer()						#clock algorithm end

#for demo purposes...
#iterate through to create the Hough lines on top of the data set returned by the Hough transform
if(lines is not None):
	for line in lines:
		for x1,y1,x2,y2 in line:
			cv2.line(line_image,(x1,y1),(x2,y2),(255,50,50),5)

#add the lines on top of the original image for display
lines_edges = cv2.addWeighted(img, 0.8, line_image, 1, 0)

#cv2.imshow('hough image', res)							# debugging code
cv2.imshow('Lane Detection', lines_edges)				# display original image with lanes detected and overlayed

time = str((stop-start)*1000)
print(time + ' ms')

cv2.waitKey(0)
cv2.destroyAllWindows()

 