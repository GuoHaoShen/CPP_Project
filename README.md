CPP_Project
------------
本项目为个人c++项目仓库，包含学习过程中各种项目代码，持续更新中...

Environment
------------
Ubuntu20.04

Make\Install
------------
对于非ros包：
进入对应子模块后执行：
```
cd matplotlib-cpp-master
mkdir build && cd build
cmake ..
make
```

对于ros工作空间：
按照ros版本对应的编译运行方法，请自行学习

Module List
----------
* matplotlib_test
  
  matplotlibcpp的使用案例，包括曲线图、函数3维图案案例，详细见代码，需要先安装matplotlib-cpp

* matplotlib-cpp

  matplotlib-cpp的源码，同时也有不少案例，安装方法见模块中的README.md

* numerical_optimization

  机器人中的数值优化源码，完成最速下降法

* googletest_test

  Google Test（简称gtest）是Google开发的一个C++单元测试框架，用于编写和运行C++的单元测试。该模块为gtest测试案例

* glog_test

  glog是Google提供的一个C++日志库，它提供了灵活的日志记录功能，可以帮助开发人员在C++应用程序中方便地记录日志信息。该模块为glog测试案例

* g2o_test

  g2o优化库测试代码，目前遇到问题停滞，未完成

* ros1_topic_ws

  ros1话题通信测试工作空间，包括不同发送频率、数据大小等因素对通信延迟的影响，测试结果见飞书文档：https://cqo2hqxe68k.feishu.cn/docx/YnuVdvB0hoVNSsxYTxvcR6xnnFb

* ros2_topic_ws

  ros2话题通信测试工作空间，包括不同Qos策略、不同数据大小等对通信的影响，测试结果见飞书文档：https://cqo2hqxe68k.feishu.cn/docx/YnuVdvB0hoVNSsxYTxvcR6xnnFb

* ros2_nav_ws

  ros2与导航功能学习工作空间，完成smooth_local_planner