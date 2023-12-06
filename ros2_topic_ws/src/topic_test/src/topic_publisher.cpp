#include <chrono>
#include <memory>
#include <random>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node {
public:
  ImagePublisher() : Node("image_publisher"), count_(0) {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", rclcpp::QoS(10).best_effort());
    timer_ = this->create_wall_timer(100ms, std::bind(&ImagePublisher::publish_image, this));
  }

private:
  void publish_image() {
    auto message = sensor_msgs::msg::Image();
    // message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "image_frame";
    message.height = 3000;
    message.width = 3000;
    message.encoding = "rgba8";
    message.is_bigendian = false;
    message.step = message.width * 4; // Assuming 4 bytes per pixel (RGBA8)
    message.data = std::vector<uint8_t>(message.step * message.height, 0); // Random data

    // Fill the image data with random values
    std::generate(message.data.begin(), message.data.end(), std::rand);

    // RCLCPP_INFO(this->get_logger(), "Publishing image #%ld", count_++);
    rclcpp::Time start = this->get_clock()->now();
    message.header.stamp = this->get_clock()->now();
    publisher_->publish(message);
    rclcpp::Time end = this->get_clock()->now();
    // rclcpp::Duration latency = end - start;
    duration_vec.push_back(durationToDoubleSeconds(end - start));
    double average = std::accumulate(duration_vec.begin(), duration_vec.end(), 0.0) / duration_vec.size();
    RCLCPP_INFO(this->get_logger(), "Publishing image %ld, latency: %f ms", count_++, average*1000);
  }

  size_t random_size() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(100, 1000); // Random size between 100 and 1000
    return dis(gen);
  }

  double durationToDoubleSeconds(const rclcpp::Duration& duration)
  {
    int64_t seconds = duration.seconds();
    uint64_t nanoseconds = duration.nanoseconds();
    double result = static_cast<double>(seconds) + (static_cast<double>(nanoseconds) / 1e9);
    return result;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;
  std::vector<double> duration_vec;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}