cmake_minimum_required(VERSION 3.0)
project(g2o_example)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 寻找g2o库
find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIR})

# 寻找Eigen库
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 添加可执行文件
add_executable(quadratic_func_example src/quadratic_func_example.cpp)

# 链接g2o库
target_link_libraries(quadratic_func_example ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} 
                        g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension)
