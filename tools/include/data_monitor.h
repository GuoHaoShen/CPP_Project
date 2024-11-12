#ifndef CARTOGRAPHER_ROS_SENSORDATAMONITOR_H
#define CARTOGRAPHER_ROS_SENSORDATAMONITOR_H

#include <chrono>
#include <deque>
#include <iostream>
#include <mutex>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"

namespace cartographer_ros {

// Computes the rate at which pulses come in.
class SensorDataMonitor {
 public:
  using ClockType = std::chrono::steady_clock;
  // Computes the rate at which pulses come in over 'window_duration' in wall
  // time.
  explicit SensorDataMonitor(const cartographer::common::Duration window_duration)
      : window_duration_(window_duration) {
        initial_time_ = ClockType::now();
      }
  ~SensorDataMonitor() {}

  SensorDataMonitor(const SensorDataMonitor&) = delete;
  SensorDataMonitor& operator=(const SensorDataMonitor&) = delete;

  // Returns the pulse rate in Hz.
  double ComputeRate() const {
    if (events_.empty()) {
      return 0.;
    }
    return static_cast<double>(events_.size() - 1) /
           cartographer::common::ToSeconds((events_.back().time - events_.front().time));
  }

  // Returns the ratio of the pulse rate (with supplied times) to the wall time
  // rate. For example, if a sensor produces pulses at 10 Hz, but we call Pulse
  // at 20 Hz wall time, this will return 2.
  double ComputeWallTimeRateRatio() const {
    if (events_.empty()) {
      return 0.;
    }
    return cartographer::common::ToSeconds((events_.back().time - events_.front().time)) /
           cartographer::common::ToSeconds(events_.back().wall_time -
                             events_.front().wall_time);
  }

  // Records an event that will contribute to the computed rate.
  void Pulse(cartographer::common::Time time) {
    events_.push_back(Event{time, ClockType::now()});
    while (events_.size() > 2 &&
           (events_.back().wall_time - events_.front().wall_time) >
               window_duration_) {
      events_.pop_front();
    }
  }

  // Returns a debug string representation.
  std::string DebugString() const {
    if (events_.size() < 2) {
      return "events_.size() less then 2";
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << "rate " << ComputeRate() << " Hz "
        << DelaysDebugString() << " (pulsed at "
        << ComputeWallTimeRateRatio() * 100. << "% real time)" 
        << " duration last data to now " 
        << cartographer::common::ToSeconds(ClockType::now() - events_.back().wall_time);
    return out.str();
  }

  // 判断数据是否丢失（一定时间内是否有数据）
  bool IsDataLoss(const double timeout_threshold) const {
    if (events_.empty()) {
      return cartographer::common::ToSeconds(ClockType::now() - initial_time_) > 
        timeout_threshold;
    }
    return cartographer::common::ToSeconds(ClockType::now() - events_.back().wall_time) > 
      timeout_threshold;
  };

  // Returns the average and standard deviation of the delays.
  std::pair<double, double> ComputeDeviationDelays() const {
    const auto delays = ComputeDelaysInSeconds();
    const double sum = std::accumulate(delays.begin(), delays.end(), 0.);
    const double mean = sum / delays.size();

    double sum_of_squares = 0.0;
    for (const double x : delays) {
        sum_of_squares += (x - mean) * (x - mean);
    }
    const double variance = sum_of_squares / delays.size();

    return {mean, variance};
  }

 private:
  struct Event {
    cartographer::common::Time time;
    typename ClockType::time_point wall_time;
  };

  // Computes all differences in seconds between consecutive pulses.
  std::vector<double> ComputeDelaysInSeconds() const {
    CHECK_GT(events_.size(), 1);
    const size_t count = events_.size() - 1;
    std::vector<double> result;
    result.reserve(count);
    for (size_t i = 0; i != count; ++i) {
      result.push_back(
          cartographer::common::ToSeconds(events_[i + 1].time - events_[i].time));
    }
    return result;
  }

  // Returns the average and standard deviation of the delays.
  std::string DelaysDebugString() const {
    auto mean_and_variance = ComputeDeviationDelays();
    double mean = mean_and_variance.first;
    double variance = mean_and_variance.second;

    std::ostringstream out;
    out << std::scientific << std::setprecision(2) << "delay " << mean << " s +/- " 
        << variance << " s";
    return out.str();
  }

  std::deque<Event> events_;
  ClockType::time_point initial_time_;
  const cartographer::common::Duration window_duration_;
  // const MonitorParams params_;
};
}  // end namespace cartographer_ros

#endif