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

#ifndef COMPOSITION__TOPICSUBSCRIBER_COMPONENT_HPP_
#define COMPOSITION__TOPICSUBSCRIBER_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <random>

namespace topic_test
{

class TopicSubscriber : public rclcpp::Node
{
public:
  // COMPOSITION_PUBLIC
  explicit TopicSubscriber(const rclcpp::NodeOptions & options);

protected:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void timerCallback() {
    double frame_rate = static_cast<double>(time_count_);
    RCLCPP_INFO(this->get_logger(), "Frame rate: %.2f fps", frame_rate);
    time_count_ = 0;
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
  size_t time_count_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time;
  std::vector<double> duration_vec;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

}  // namespace composition

#endif  // COMPOSITION__TOPICSUBSCRIBER_COMPONENT_HPP_
