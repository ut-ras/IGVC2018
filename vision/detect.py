import roslib
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def image_callback(msg):
    img = msg
    cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

bridge = CvBridge()
rospy.init_node('image_listener')
rospy.Subscriber("/camera/image_raw", Image, image_callback)
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
cv2.destroyAllWindows()
