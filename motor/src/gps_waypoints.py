import rospy
from std_msgs.msg import Float32
from vn200.msg import vn_200_gps_soln

waypoints = [[123.0, 456.0], [456.0, 123.0]]  # obviously replace these

ourPos = [0.0, 0.0]
ourDir = 0.0


def yaw_cb(data):
    ourDir = data
    print "Recieved direction:", ourDir


def gps_cb(data):
    ourPos[0] = data.latitude
    ourPos[1] = data.longitude
    print "Recieved GPS:", ourPos


def driver():
    rospy.init_node('gps_waypoints', log_level=rospy.INFO)

    imu_sub = rospy.Subscriber("yaw_heading", Float32, yaw_cb)
    gps_sub = rospy.Subscriber("vn_200_gps_soln", vn_200_gps_soln, gps_cb)

    left = rospy.Publisher('left', Float32, queue_size=10)
    right = rospy.Publisher('right', Float32, queue_size=10)

    rate = rospy.Rate(10)  # number of times per second we run our loop
    while not rospy.is_shutdown():
        for next_waypoint in waypoints:
            # check if we need to turn and either turn or go straight
            left.publish(123)
            right.publish(456)
            rate.sleep()


if __name__ == '__main__':
    try:
        driver()
    except rospy.ROSInterruptException:
        pass
