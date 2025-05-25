cmake_minimum_required(VERSION 3.0.2)
project(Homography)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/document_rectifier.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY scripts/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
# )

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )