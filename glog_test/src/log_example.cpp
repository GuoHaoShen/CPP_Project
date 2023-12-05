#include <glog/logging.h>

int main(int argc, char* argv[]) {
  // 初始化glog
  google::InitGoogleLogging(argv[0]);

  // 设置日志级别
  FLAGS_minloglevel = google::INFO; // 或者注释掉这一行，使用默认值0

  // 记录日志
  LOG(INFO) << "This is an informational message.";
  LOG(WARNING) << "This is a warning message.";
  LOG(ERROR) << "This is an error message.";

  // 关闭glog
  google::ShutdownGoogleLogging();
  return 0;
}
