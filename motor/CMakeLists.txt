cmake_minimum_required(VERSION 2.8.3)
project(motor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  rosserial_python
)

catkin_package()

catkin_install_python(PROGRAMS src/manual_control.py src/xbox.py src/gps_waypoints.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
