cmake_minimum_required(VERSION 3.0)

project(NumerialExample)

find_package(Python3 COMPONENTS Development NumPy)
include_directories(${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(steepest_gd_test src/steepest_gd_test.cpp)
target_link_libraries(steepest_gd_test Python3::Python)

add_executable(lbfgs_example src/lbfgs_example.cpp)