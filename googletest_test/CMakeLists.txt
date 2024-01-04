cmake_minimum_required(VERSION 3.10)

project(gtestExample)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找 Google Test，它应该也包含了 Google Mock
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# EXPECT_EQ示例
add_executable(example src/example.cpp)
target_link_libraries(example ${GTEST_BOTH_LIBRARIES} gtest_main pthread)
enable_testing()
add_test(NAME example COMMAND example)

# TEST_F示例
add_executable(TEST_F_example src/TEST_F_example.cpp)
target_link_libraries(TEST_F_example ${GTEST_BOTH_LIBRARIES} pthread)
enable_testing()
add_test(NAME TEST_F_example COMMAND TEST_F_example)

# 参数化测试示例
add_executable(Parameter_example src/Parameter_example.cpp)
target_link_libraries(Parameter_example ${GTEST_BOTH_LIBRARIES} pthread)
enable_testing()
add_test(NAME Parameter_example COMMAND Parameter_example)

# EXPECT_DEATH示例
add_executable(EXPECT_DEATH_example src/EXPECT_DEATH_example.cpp)
target_link_libraries(EXPECT_DEATH_example ${GTEST_BOTH_LIBRARIES} pthread)
enable_testing()
add_test(NAME EXPECT_DEATH_example COMMAND EXPECT_DEATH_example)

# TEST示例
add_executable(TEST_example src/TEST_example.cpp)
target_link_libraries(TEST_example ${GTEST_BOTH_LIBRARIES} pthread)
enable_testing()
add_test(NAME TEST_example COMMAND TEST_example)