// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "topic_test/publisher_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace topic_test
{

// Create a TopicPublisher "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.
TopicPublisher::TopicPublisher(const rclcpp::NodeOptions & options)
: Node("TopicPublisher", options), count_(0)
{
  publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", rclcpp::ClockQoS());

  timer_ = this->create_wall_timer(10ms, std::bind(&TopicPublisher::publish_image, this));
}


void TopicPublisher::publish_image() {
  auto message = sensor_msgs::msg::Image();
  // message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "image_frame";
  message.height = 1000;
  message.width = 1000;
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

}  // namespace topic_test

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(topic_test::TopicPublisher)
