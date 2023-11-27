cmake_minimum_required(VERSION 3.0)

project(MatplotlibCppExample)

find_package(Python3 COMPONENTS Development NumPy)
include_directories(${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

add_executable(steepest_gd_test steepest_gd_test.cpp)
target_link_libraries(steepest_gd_test Python3::Python)

