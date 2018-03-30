#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from vn200.msg import vn_200_gps_soln
import math

waypoints = [[123.0, 456.0], [456.0, 123.0]]  # obviously replace these

ourPos = [0.0, 0.0]
ourDir = 0.0
angle_deadzone = 10  # Change later
turnspeed = .25
straightspeed = .5
distance_deadzone = .000025


def yaw_cb(msg):
    global ourDir
    ourDir = (msg.data + 360) % 360
    #  print "you just got a letter", ourDir


def gps_cb(msg):
    global ourPos
    ourPos[0] = msg.longitude
    ourPos[1] = msg.latitude


def compass_bearing(pointA, pointB):

    lat1 = math.radians(pointA[0])
    lat2 = math.radians(pointB[0])

    diffLong = math.radians(pointB[1] - pointA[1])

    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
                                           * math.cos(lat2) * math.cos(diffLong))

    initial_bearing = math.atan2(x, y)

    initial_bearing = math.degrees(initial_bearing)

    compass_bearing = (initial_bearing + 360) % 360
    return compass_bearing


def get_distance(pointA, pointB):

    distance = math.sqrt(
        pow((pointA[0] - pointB[0]), 2) + pow((pointA[1] - pointB[1]), 2))
    return distance


def driver():
    global ourPos
    global ourDir
    global angle_deadzone
    global turnspeed
    global straightspeed
    global distance_deadzone

    rospy.init_node('gps_waypoints', log_level=rospy.INFO)

    imu_sub = rospy.Subscriber("yaw_heading", Float32, yaw_cb)
    gps_sub = rospy.Subscriber("vn_200_gps", vn_200_gps_soln, gps_cb)

    print "made subs"

    left = rospy.Publisher('left', Float32, queue_size=10)
    right = rospy.Publisher('right', Float32, queue_size=10)

    print "made pubs"

    rate = rospy.Rate(10)  # number of times per second we run our loop
    print "set rate"
    while not rospy.is_shutdown():
        print "please"
        for next_waypoint in waypoints:
            print get_distance(ourPos, next_waypoint)
            while get_distance(ourPos, next_waypoint) > distance_deadzone:
                # check if we need to turn and either turn or go straight
                currentbearing = compass_bearing(ourPos, next_waypoint)
                dif1 = abs(ourDir - currentbearing)
                print "dif:", dif1, "bear:", currentbearing, "ourDir:", ourDir
                if dif1 > angle_deadzone:
                    if dif1 < 180:
                        left.publish(-turnspeed)
                        right.publish(turnspeed)
                    else:
                        left.publish(turnspeed)
                        right.publish(-turnspeed)
                else:
                    left.publish(straightspeed)
                    right.publish(straightspeed)
                rate.sleep()


if __name__ == '__main__':
    try:
        print "test stuff"
        driver()
    except rospy.ROSInterruptException:
        pass
