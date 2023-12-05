CPP_Project
------------
本项目为个人c++项目仓库，包含学习过程中各种项目代码，持续更新中...

Environment
------------
Ubuntu20.04

Make\Install
------------
进入对应子模块后执行：
```
cd matplotlib-cpp-master
mkdir build && cd build
cmake ..
make
```

Module List
----------
* matplotlib_test
  
  matplotlibcpp的使用案例，需要先安装matplotlib-cpp

* matplotlib-cpp

  matplotlib-cpp的源码，安装方法见模块中的README.md

* numerical_optimization

  机器人中的数值优化源码

* googletest_test

  Google Test（简称gtest）是Google开发的一个C++单元测试框架，用于编写和运行C++的单元测试。该模块为gtest测试案例

* glog_test

  glog是Google提供的一个C++日志库，它提供了灵活的日志记录功能，可以帮助开发人员在C++应用程序中方便地记录日志信息。该模块为glog测试案例