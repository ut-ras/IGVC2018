#!/usr/bin/env python
""" Publishes motor speed topics based on joystick input

Requires xboxdrv (sudo apt install xboxdrv)
"""

import xbox
import rospy
from std_msgs.msg import Float32


def joy2speed(joy, tank=False):
    """Converts joystick x and y axis values to left and right motor speeds
    Defaults to arcade drive with optional tank drive"""

    if tank:
        return (joy.leftY(), joy.rightY())
    x = joy.leftX()
    y = joy.leftY()
    #TODO: test/fix arcade drive and motor directions
    return ((y + x) / 2, (y - x) / 2)


def publisher():
    """Publishes /left and /right based on joystick input
    The left and right triggers are treated as enables for the respective motors"""

    rospy.init_node('manual_control', log_level=rospy.INFO)
    joystick = None
    for _ in range(3):
        try:
            joystick = xbox.Joystick()
        except IOError:
            rospy.logerr(
                "Failed to connect to xbox controller. Trying agin in 10 seconds..."
            )
            rospy.sleep(10.)
    if not joystick:
        rospy.logfatal("Xbox controller not found. Shutting down...")
        rospy.signal_shutdown("Couldn't connect to xbox controller")
    rospy.loginfo("Joystick found")
    #TODO measure speed going straght and constrain to 5mph
    max_speed = .5
    left = rospy.Publisher('left', Float32, queue_size=10)
    right = rospy.Publisher('right', Float32, queue_size=10)
    #TODO: test other rates and find a balance between bandwidth usage and fine control
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        left_speed, right_speed = joy2speed(joystick, True)
        left_speed *= max_speed * joystick.leftTrigger()
        right_speed *= max_speed * joystick.rightTrigger()
        left.publish(left_speed)
        right.publish(right_speed)
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
