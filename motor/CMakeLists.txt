cmake_minimum_required(VERSION 2.8.3)
project(motor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  rosserial_python
)

catkin_package()

install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
