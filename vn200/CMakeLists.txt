cmake_minimum_required(VERSION 2.8.3)
project(vn200)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

  add_message_files(
    FILES
    EulerOrientation.msg
    vn_200_gps_soln.msg
    vn_200_accel_gyro_compass.msg
    vn_200_ins_soln.msg
  )

  generate_messages(
    DEPENDENCIES
    geometry_msgs
  )

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/imu_listener.py src/vn_200_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
