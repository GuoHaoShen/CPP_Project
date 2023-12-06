#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImageSubscriber : public rclcpp::Node {
public:
  ImageSubscriber() : Node("image_subscriber"), count_(0) {

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_topic", rclcpp::QoS(10).best_effort(), std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    rclcpp::Time now = this->get_clock()->now();
    rclcpp::Time image_stamp(msg->header.stamp);
    // rclcpp::Duration latency = now - image_stamp;
    duration_vec.push_back(durationToDoubleSeconds(now - image_stamp));
    double average = std::accumulate(duration_vec.begin(), duration_vec.end(), 0.0) / duration_vec.size();
    RCLCPP_INFO(this->get_logger(), "Received image %ld, latency: %f ms", count_++, average*1000);
  }

  double durationToDoubleSeconds(const rclcpp::Duration& duration)
  {
    int64_t seconds = duration.seconds();
    uint64_t nanoseconds = duration.nanoseconds();
    double result = static_cast<double>(seconds) + (static_cast<double>(nanoseconds) / 1e9);
    return result;
  }

  std::vector<double> duration_vec;
  size_t count_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}