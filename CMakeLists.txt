cmake_minimum_required(VERSION 3.0.2)
project(elp_camera)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  std_msgs
  message_generation
)

add_message_files(
  FILES
  MotionDetection.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs 
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  sensor_msgs 
  cv_bridge 
  std_msgs 
  message_runtime
  DEPENDS OpenCV  
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(subscriber src/subscriber.cpp)

add_dependencies(subscriber
  ${catkin_EXPORTED_TARGETS}
  elp_camera_generate_messages_cpp
)

target_link_libraries(subscriber
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)