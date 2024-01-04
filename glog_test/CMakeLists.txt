cmake_minimum_required(VERSION 3.5)

project(MyLoggingApp)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 添加可执行文件
add_executable(log_example src/log_example.cpp)
# 链接glog库
target_link_libraries(log_example glog)

# 添加可执行文件
add_executable(Destination_example src/Destination_example.cpp)
# 链接glog库
target_link_libraries(Destination_example glog)
