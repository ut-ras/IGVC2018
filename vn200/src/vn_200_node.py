#!/usr/bin/env python
#
#driver for VN-200 IMU
#datasheet:
#Author: Cruz Monrreal II

#Edited vn_200_gps_soln.msg file: Position accuracy estimate not acceleration

#Edited node.py: Ins status read from 3rd element not 2nd. (Shifts every index down one)
#If above doesn't work try shifting back one

#Edited queue size for publishers. ROS requires this.

#you MUST import vector3 and header packages from corresponding ros messages
#ROS init node must take in this file's name as the name parameter. Use anonymous = True to allow for duplicates

#--- If ctrl + c does not quit use ctrl + shift + \

from vn200.msg import vn_200_accel_gyro_compass, vn_200_gps_soln, vn_200_ins_soln
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header, Float32MultiArray
import rospy
import serial
import time

DO_SCALING = True

IMU_MSG_LEN = 12
GPS_SOLN_MSG_LEN = 16
INS_SOL_MSG_LEN = 16
MAG_COMP_MSG_LEN = 13

ser = serial.Serial(port = '/dev/ttyUSB0', baudrate=921600)

imu_pub = rospy.Publisher('vn200_accel_gyro_compass', vn_200_accel_gyro_compass, queue_size = 10)
gps_pub = rospy.Publisher('vn200_gps', vn_200_gps_soln, queue_size = 10)
ins_pub = rospy.Publisher('vn200_ins', vn_200_ins_soln, queue_size = 10)
mag_comp_c_pub = rospy.Publisher('vn200_mag_comp_c_matrix', Float32MultiArray, queue_size = 10)
mag_comp_b_pub = rospy.Publisher('vn200_mag_comp_b_matrix', Float32MultiArray, queue_size = 10)

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
             cmd("VNRRG,63"), # read INS soln from the register
             cmd("VNRRG,23")] # read mag compensation from the register

def strip_tag_and_checksum (data):
    # remove the $VNRRG tag in front of the message and the checksum behind the message
    return data[7:-4].split(',')

def publish_imu_data(imu_data):
    global imu_pub

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

def publish_gps_data(gps_data):
    global gps_pub

    gps_msg = vn_200_gps_soln()
    gps_msg.header.stamp = rospy.get_rostime()
    gps_msg.err_string = ""
    gps_msg.error_present = False

    gps_fix = int(gps_data[3])

    if gps_fix == 0:
        rospy.logwarn("GPS: No GPS Fix")
        gps_msg.error_present = True
        gps_msg.err_string += "No GPS fix. \n"

    elif gps_fix == 1:
        rospy.logwarn("GPS: Time only in GPS soln")
        gps_msg.error_present = True
        gps_msg.err_string += "Time only in GPS soln \n"
    elif gps_fix == 2:
        rospy.logwarn("GPS: 2D only. Horizontal Position may be affected")

    gps_msg.num_satellites = float(gps_data[4])

    if gps_msg.num_satellites < 3:
        rospy.logwarn("Less than 3 satellites found. Try moving to an open area")
        gps_msg.error_present = True
        gps_msg.err_string += "Less than 3 satellites found \n"

    gps_msg.latitude = float(gps_data[5])
    gps_msg.longitude = float(gps_data[6])
    gps_msg.altitude = float(gps_data[7])

    gps_msg.ned_velocities.x = float(gps_data[8])
    gps_msg.ned_velocities.y = float(gps_data[9])
    gps_msg.ned_velocities.z = float(gps_data[10])

    gps_msg.ned_accuracy.x = float(gps_data[11])
    gps_msg.ned_accuracy.y = float(gps_data[12])
    gps_msg.ned_accuracy.z = float(gps_data[13])

    gps_msg.speed_accuracy_estimate = float(gps_data[14])

    gps_pub.publish(gps_msg)

def publish_ins_data(ins_data):
    global ins_pub

    ins_msg = vn_200_ins_soln()
    ins_msg.header.stamp = rospy.get_rostime()
    ins_msg.error_present = False
    ins_msg.err_string = ""

    ins_status = int(ins_data[3], 16) & 0xFFFF

    ins_mode = ins_status & 0x0003
    ins_gps_fix = ins_status & 0x0004
    ins_error = ins_status & 0x0078

    if ins_mode == 0:
        rospy.logwarn("INS: Not tracking. Insufficient dynamic motion to esimtate attitude.")
        ins_msg.error_present = True
        ins_msg.err_string += "Insufficient dynamic motion. \n"

    elif ins_mode == 1:
        rospy.logwarn("INS: Sufficient dynamic motion, but solution not within performance specs.")
        ins_msg.error_present = True
        ins_msg.err_string += "Sufficient dynamic motion, but solution not within performance specs. \n"

    if ins_gps_fix != 1:
        rospy.logwarn("INS: No GPS fix.")
        ins_msg.error_present = True
        ins_msg.err_string += "No GPS fix found. \n"

    if ins_error == 1:
        rospy.logwarn("INS: Filter loop exceeds 5 ms!")
        ins_msg.error_present = True
        ins_msg.err_string += "INS Filter loop exceds 5 ms. \n"

    elif ins_error == 2:
        rospy.logwarn("INS: IMU communication error detected. Try resetting.")
        ins_msg.error_present = True
        ins_msg.err_string += "IMU communcation error detected. \n"

    elif ins_error == 4:
        rospy.logwarn("INS: Magnetometer or Pressure Sensor Error Detected.")
        ins_msg.error_present = True
        ins_msg.err_string += "Magnetometer or pressure sensor error detected. \n"

    elif ins_error == 8:
        rospy.logwarn("INS: GPS communication error detected.")
        ins_msg.error_present = True
        ins_msg.err_string += "GPS communication error detected. \n"

    ins_msg.yaw_heading = float(ins_data[4])
    ins_msg.pitch = float(ins_data[5])
    ins_msg.roll = float(ins_data[6])

    ins_msg.geodetic_latitude = float(ins_data[7])
    ins_msg.geodetic_longitude = float(ins_data[8])
    ins_msg.altitude = float(ins_data[9])

    ins_msg.ned_velocities.x = float(ins_data[10])
    ins_msg.ned_velocities.y = float(ins_data[11])
    ins_msg.ned_velocities.z = float(ins_data[12])

    ins_msg.attitude_uncertainty = float(ins_data[13])
    ins_msg.position_uncertainty = float(ins_data[14])
    ins_msg.velocity_uncertainty = float(ins_data[15])

    ins_pub.publish(ins_msg)

def publish_mag_comp_data(mag_comp_data):
    global mag_comp_pub

    c_matrix = Float32MultiArray()
    b_matrix = Float32MultiArray()

    for i in range(9):
    	c_matrix.data.append(float(mag_comp_data[i+1]))
    for i in range(3):
    	b_matrix.data.append(float(mag_comp_data[i+10]))

    mag_comp_c_pub.publish(c_matrix)
    mag_comp_b_pub.publish(b_matrix)

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
        #return gps_data[5:6]
        #print(gps_data)

    elif  data[7:9] == "63" and data[1:6] == "VNRRG":#data[1:6] == "VNINS":
        ins_data = data[7:-4].split(',')
        if len(ins_data) is not INS_SOL_MSG_LEN:
           rospy.logwarn("ERROR reading INS message")
           return
        #rospy.loginfo('!!!!INS' + data)
        publish_ins_data(ins_data)

    elif  data[7:9] == "23" and data[1:6] == "VNRRG":#data[1:6] == "VNMAGCOMP":
        mag_compensation_data = data[7:-4].split(',')
        if len(mag_compensation_data) is not MAG_COMP_MSG_LEN:
           rospy.logwarn("ERROR reading Mag compensation message")
           return
        publish_mag_comp_data(mag_compensation_data)

    else:
        rospy.loginfo("Unknown message: " + str(data) + '\n')

def vn200_main():

    global READ_CMDS
    global ser

    rospy.init_node('vn_200_node')
    rospy.loginfo("VN-200 IMU listener is running on " + ser.portstr )

    ser.flush()
    ser.close()
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

            '''
            if validate_checksum(data):
                # check if data was an IMU, INS SOLN or GPS SOLN reply,
                # process accordingly and then publish the appropriate message
                try:
                    process_and_publish(data)
                    #print(imu_data)
                    #print(gps_data)
                    #print(ins_data)
                except:
                    rospy.logwarn("vn_200_imu data dropped")
            else:
                rospy.logwarn("Checksum incorrect for %s. Dropping packet", data)
            '''
            process_and_publish(data)

            '''
            if data[7:9] == "54" and data[1:6] == "VNRRG":#data[1:6] == "VNIMU":
                imu_data = strip_tag_and_checksum(data)
                print(imu_data[4])
            '''

            r.sleep()

if __name__ == "__main__":
    try:
        vn200_main()
    except rospy.ROSInterruptException:
        pass