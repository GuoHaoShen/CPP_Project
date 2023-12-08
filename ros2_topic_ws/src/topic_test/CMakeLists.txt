cmake_minimum_required(VERSION 3.8)
project(topic_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(publisher src/topic_publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs sensor_msgs)

add_executable(subscriber src/topic_subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs sensor_msgs)

add_library(publish_component SHARED
  src/publisher_component.cpp)
target_compile_definitions(publish_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(publish_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs")
rclcpp_components_register_nodes(publish_component "topic_test::TopicPublisher")
set(node_plugins "${node_plugins}topic_test::TopicPublisher;$<TARGET_FILE:publish_component>\n")

add_library(subscrib_component SHARED
  src/subscriber_component.cpp)
target_compile_definitions(subscrib_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(subscrib_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs")
rclcpp_components_register_nodes(subscrib_component "topic_test::TopicSubscriber")
set(node_plugins "${node_plugins}topic_test::TopicSubscriber;$<TARGET_FILE:subscrib_component>\n")

set(libs
  publish_component
  subscrib_component
)

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  publish_component
  subscrib_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS 
  publisher
  subscriber
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
