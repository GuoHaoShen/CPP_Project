cmake_minimum_required(VERSION 3.12)  # 指定所需的CMake最低版本
project(HelloWorld)  # 设置项目名称

set(CMAKE_CXX_STANDARD 14)  # 设置C++标准版本

# 添加可执行文件的源文件
add_executable(HelloWorld main.cpp)

# 添加其他源文件
# add_executable(HelloWorld main.cpp other.cpp)

# 添加头文件搜索路径
# include_directories(include)

# 添加库文件搜索路径
# link_directories(lib)

# 添加链接的库文件
# target_link_libraries(HelloWorld library_name)

# 添加编译选项
# target_compile_options(HelloWorld PRIVATE -Wall -Wextra)

# 添加链接选项
# target_link_options(HelloWorld PRIVATE -static-libgcc -static-libstdc++)

# 添加自定义编译定义
# target_compile_definitions(HelloWorld PRIVATE DEBUG)

# 添加自定义编译标志
# target_compile_features(HelloWorld PRIVATE cxx_std_17)

# 添加自定义安装规则
# install(TARGETS HelloWorld DESTINATION bin)

# 添加自定义测试
# enable_testing()
# add_test(NAME MyTest COMMAND HelloWorld)
