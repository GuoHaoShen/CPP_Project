cmake_minimum_required(VERSION 3.0)

project(MatplotlibCppExample)

find_package(Python3 COMPONENTS Development NumPy)
include_directories(${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

add_executable(example example.cpp)
target_link_libraries(example Python3::Python)

add_executable(function_surface function_surface.cpp)
target_link_libraries(function_surface Python3::Python)
