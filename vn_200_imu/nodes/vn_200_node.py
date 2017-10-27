#!/usr/bin/env python
#
#driver for VN-200 IMU
#datasheet:
#Author: Cruz Monrreal II

import roslib; roslib.load_manifest('vn_200_imu')
import rospy
from vn_200_imu.msg import vn_200_accel_gyro_compass, vn_200_gps_soln, vn_200_ins_soln
import serial
import struct
import time


DO_SCALING = True

IMU_MSG_LEN = 12
GPS_SOLN_MSG_LEN = 16
INS_SOL_MSG_LEN = 16

ser = serial.Serial(port = '/dev/VN-200', baudrate=921600)

imu_pub = rospy.Publisher('vn_200_accel_gyro_compass', vn_200_accel_gyro_compass)
gps_pub = rospy.Publisher('vn_200_gps_soln', vn_200_gps_soln)
ins_pub = rospy.Publisher('vn_200_ins_soln', vn_200_ins_soln)

def read_data() :

    data = "$"

    while not rospy.is_shutdown():
        c = ser.read(1)
        if c == '$':
            break

    while not rospy.is_shutdown():
        c = ser.read(1)
        if c == '\n':
            break
        elif c == '\x00':
            continue
        data += c

    return data

def validate_checksum(msg):
    try:
        xor = 0
        msg = msg.strip()
        #rospy.loginfo("~~~~~~~" + msg + "~~~~~~");
        #rospy.loginfo("~~~~~~~" + str(msg[-2]) + str(msg[-1])+ "!~~~~~~~~");
        chksum = int(str(msg[-2]) + str(msg[-1]), 16)
        #chksum = int(str(msg[-3]) +str([-2]), 16)
        data = msg[1:-3].upper()

        for char in data:
            xor = xor ^ ord(char)
        #rospy.loginfo(">>>>>" + str(xor) + ">>>>>>>>" + str(chksum))
        return xor == chksum
    except ValueError:
        return False

def cmd(string):
    xor = 0

    for char in string.upper():
        xor = xor ^ ord(char)

    return '$' + string + '*' + hex(xor & 0xFF)[2:4].upper() + '\n'

READ_CMDS = [cmd("VNRRG,54"), # read IMU data from the register
             cmd("VNRRG,58"), # read GPS soln from the register
             cmd("VNRRG,63")] # read INS soln from the register

def publish_imu_data (imu_data) :

    global imu_pub

    rospy.loginfo("IMU: " + str(imu_data))

    imu_msg = vn_200_accel_gyro_compass()
    imu_msg.header.stamp = rospy.get_rostime()

    imu_msg.compass.x = float(imu_data[1])
    imu_msg.compass.y = float(imu_data[2])
    imu_msg.compass.z = float(imu_data[3])

    imu_msg.accelerometer.x = float(imu_data[4])
    imu_msg.accelerometer.y = float(imu_data[5])
    imu_msg.accelerometer.z = float(imu_data[6])

    imu_msg.gyro.x = float(imu_data[7])
    imu_msg.gyro.y = float(imu_data[8])
    imu_msg.gyro.z = float(imu_data[9])

    imu_pub.publish(imu_msg)

def publish_gps_data (gps_data) :

    global gps_pub

    rospy.loginfo("GPS: " + str(gps_data))

    gps_msg = vn_200_gps_soln()
    gps_msg.header.stamp = rospy.get_rostime()
    gps_msg.error_string = ''
    gps_msg.error_present = False

    gps_fix = int(gps_data[3])

    if gps_fix == 0:
        rospy.logwarn("GPS: No GPS fix")
        gps_msg.error_present = True
        gps_msg.error_string += "No GPS fix.\n"
    elif gps_fix == 1:
        gps_msg.error_present = True
        rospy.logwarn("GPS: Time only in GPS soln")
        gps_msg.error_string += "Time only in GPS soln\n"
    elif gps_fix == 2:
        rospy.logwarn("GPS: 2D fix. Horizontal position might be impacted.")

    gps_msg.num_satellites = int(gps_data[4])

    if gps_msg.num_satellites < 3:
        gps_msg.error_present = True
        rospy.logwarn("GPS: Solution is has connection to less than 3 satellites. Please move to an open area")
        gps_msg.error_string += "Less than 3 satellites found\n"

    gps_msg.latitude  = float(gps_data[5])
    gps_msg.longitude = float(gps_data[6])
    gps_msg.altitude  = float(gps_data[7])

    gps_msg.ned_velocities.x = float(gps_data[8])
    gps_msg.ned_velocities.y = float(gps_data[9])
    gps_msg.ned_velocities.z = float(gps_data[10])

    gps_msg.ned_acceleration.x = float(gps_data[11])
    gps_msg.ned_acceleration.y = float(gps_data[12])
    gps_msg.ned_acceleration.z = float(gps_data[13])

    gps_msg.speed_accuracy_estimate = float(gps_data[14])

    gps_pub.publish(gps_msg)

def publish_ins_data (ins_data) :

    global ins_pub

    rospy.loginfo("INS: " + str(ins_data) + '\n')

    ins_msg = vn_200_ins_soln()
    ins_msg.header.stamp = rospy.get_rostime()
    ins_msg.error_present = False
    ins_msg.error_string = ''

    ins_status = int(ins_data[2], 16) & 0xFFFF # Make sure we have only 16 bits. No python weirdness

    ins_mode    = ins_status & 0x0003
    ins_gps_fix = ins_status & 0x0004
    ins_error   = ins_status & 0x0078

    if ins_mode == 0:
        ins_msg.error_present = True
        rospy.logwarn("INS: Not tracking. Insufficient dynamic motion")
        ins_msg.error_string += "Not tracking. Insufficient dynamic motion\n"
    elif ins_mode == 1:
        rospy.logwarn("INS: Sufficient dynamic motion, but solution was not within performence specs.")
        ins_msg.error_string += "Sufficient dynamic motion, but solution was not withing perforemnce specs.\n"

    if ins_gps_fix != 1:
        ins_msg.error_present = True
        rospy.logwarn("INS: No GPS fix")
        ins_msg.error_string += "No GPS fix\n"

    if ins_error & 0x0001:
        ins_msg.error_present = True
        rospy.logwarn("INS: Time Error. IMU filter loop has taken more than 5 ms")
        ins_msg.error_string += "Time Error in INS\n"
    if ins_error & 0x0002:
        ins_msg.error_present = True
        rospy.logwarn("INS: IMU Error. IMU communication error detected")
        ins_msg.error_string += "IMU error\n"
    if ins_error & 0x0004:
        ins_msg.error_present = True
        rospy.logwarn("IMU: Magnetometer or Pressure sensor error detected")
        ins_msg.error_string += "Magnetometer or Pressure sensor error detected\n"
    if ins_error & 0x0008:
        ins_msg.error_present = True
        rospy.logwarn("IMU: GPS communication error detected")
        ins_msg.error_string += "GPS communication error detected\n"

    ins_msg.orientation_euler.yaw   = float(ins_data[3])
    ins_msg.orientation_euler.pitch = float(ins_data[4])
    ins_msg.orientation_euler.roll  = float(ins_data[5])

    ins_msg.geodetic_latitude   = float(ins_data[6])
    ins_msg.geodetic_longitude  = float(ins_data[7])
    ins_msg.altitude            = float(ins_data[8])

    ins_msg.ned_velocities.x = float(ins_data[9])       # Velocity X
    ins_msg.ned_velocities.y = float(ins_data[10])       # Velocity Y
    ins_msg.ned_velocities.z = float(ins_data[11])       # Velocity Z

    ins_msg.attitude_uncertainty = float(ins_data[12])   # Altitude Uncertainty
    ins_msg.position_uncertainty = float(ins_data[13])   # Position Uncertainty
    ins_msg.velocity_uncertainty = float(ins_data[14])   # Velocity Uncertainty

    ins_pub.publish(ins_msg)

def strip_tag_and_checksum (data) :
    # remove the $VNRRG tag in front of the message and the checksum behind the message
    return data[7:-4].split(',')

def process_and_publish(data):

    global GPS_SOLN_MSG_LEN
    global INS_SOL_MSG_LEN
    global IMU_MSG_LEN

    if data[7:9] == "54" and data[1:6] == "VNRRG":#data[1:6] == "VNIMU":
        imu_data = strip_tag_and_checksum(data)
        # discard the message of if it does not have all the necessaty fields
        if len(imu_data) is not IMU_MSG_LEN:
            rospy.logwarn("ERROR reading IMU message")
            return
        publish_imu_data(imu_data)

    elif data[7:9] == "58" and data[1:6] == "VNRRG":#data[1:6] == "VNGPE"
        gps_data = strip_tag_and_checksum(data)
        if len(gps_data) is not GPS_SOLN_MSG_LEN:
            rospy.logwarn("ERROR reading GPS message")
            return
        publish_gps_data(gps_data)

    elif  data[7:9] == "63" and data[1:6] == "VNRRG":#data[1:6] == "VNINS":
        ins_data = data[7:-4].split(',')
        if len(ins_data) is not INS_SOL_MSG_LEN:
           rospy.logwarn("ERROR reading INS message")
           return
        rospy.loginfo('!!!!INS' + data)
        publish_ins_data(ins_data)

    else:
        rospy.loginfo("Unknown message: " + str(data) + '\n')


def vn200():

    global READ_CMDS
    global ser

    rospy.init_node('vn_200_imu')
    rospy.loginfo("VN-200 IMU listener is running on " + ser.portstr )

    ser.flush()
    ser.open()

    ser.write(cmd("VNWRG,05,921600"))
    ser.write(cmd("VNWRG,07,50"))
    ser.write(cmd("VNWRG,06,0"))
    #ser.write(cmd("VNWRG,06,247"))

    r = rospy.Rate(20)

    while not rospy.is_shutdown():

        for cmds in READ_CMDS:
            ser.write(cmds)

        # read 3 responses
        for i in range(len(READ_CMDS)):
            # read the data from the serial port
            data = read_data()

            if validate_checksum(data):
                # check if data was an IMU, INS SOLN or GPS SOLN reply,
                # process accordingly and then publish the appropriate message
                try:
                    process_and_publish(data)
                except:
                    rospy.logwarn("vn_200_imu data dropped")
            else:
                rospy.logwarn("Checksum incorrect for %s. Dropping packet", data)

            r.sleep()

if __name__ == "__main__":
    try:
      vn200()
    except rospy.ROSInterruptException: pass
