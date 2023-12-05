#include <glog/logging.h>

int main(int argc, char* argv[]) {
  // 初始化glog
  google::InitGoogleLogging(argv[0]);

  // 设置日志输出到文件
  google::SetLogDestination(google::INFO, "/mnt/d/Project/CPP_Project/glog_test/log/info.log"); // 将INFO级别的日志输出到info.log文件
  google::SetLogDestination(google::WARNING, "/mnt/d/Project/CPP_Project/glog_test/log/warning.log"); // 将WARNING级别的日志输出到warning.log文件
  google::SetLogDestination(google::ERROR, "/mnt/d/Project/CPP_Project/glog_test/log/error.log"); // 将ERROR级别的日志输出到error.log文件

  // 记录日志
  LOG(INFO) << "This is an informational message.";
  LOG(WARNING) << "This is a warning message.";
  LOG(ERROR) << "This is an error message.";

  // 关闭glog
  google::ShutdownGoogleLogging();
  return 0;
}
