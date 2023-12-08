#include "topic_test/subscriber_component.hpp"

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <thread>

using namespace std::chrono_literals;

namespace topic_test
{

// Create a TopicPublisher "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.
TopicSubscriber::TopicSubscriber(const rclcpp::NodeOptions & options)
: Node("TopicSubscriber", options), count_(0)
{
  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_topic", rclcpp::ClockQoS(), std::bind(&TopicSubscriber::image_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TopicSubscriber::timerCallback, this));
}

void TopicSubscriber::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  rclcpp::Time now = this->get_clock()->now();
  rclcpp::Time image_stamp(msg->header.stamp);
  // rclcpp::Duration latency = now - image_stamp;
  duration_vec.push_back(durationToDoubleSeconds(now - image_stamp));
  double average = std::accumulate(duration_vec.begin(), duration_vec.end(), 0.0) / duration_vec.size();
  RCLCPP_INFO(this->get_logger(), "Received image %ld, latency: %f ms", count_++, average*1000);
  // rclcpp::sleep_for(std::chrono::microseconds(1000));
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  time_count_++;
}

}  // namespace topic_test

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(topic_test::TopicSubscriber)