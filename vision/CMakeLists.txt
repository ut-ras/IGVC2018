cmake_minimum_required(VERSION 2.8.3)
project(vision)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  OpenCV REQUIRED
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
