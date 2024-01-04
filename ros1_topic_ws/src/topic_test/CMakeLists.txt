cmake_minimum_required(VERSION 3.0.2)
project(topic_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
)

# OpenCV
find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES topic_test
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(topic_pub src/topic_pub.cpp)
target_link_libraries(topic_pub
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(topic_sub src/topic_sub.cpp)
target_link_libraries(topic_sub
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_INCLUDE_DIRS}
)
