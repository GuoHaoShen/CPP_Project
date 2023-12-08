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

#ifndef COMPOSITION__TOPICPUBLISHER_COMPONENT_HPP_
#define COMPOSITION__TOPICPUBLISHER_COMPONENT_HPP_

// #include "composition/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <random>

namespace topic_test
{

class TopicPublisher : public rclcpp::Node
{
public:
  // COMPOSITION_PUBLIC
  explicit TopicPublisher(const rclcpp::NodeOptions & options);

protected:
  void publish_image();

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

private:
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  std::vector<double> duration_vec;
};

}  // namespace topic_test

#endif  // COMPOSITION__TOPICPUBLISHER_COMPONENT_HPP_
