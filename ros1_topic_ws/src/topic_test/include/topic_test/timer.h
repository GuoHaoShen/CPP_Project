#include "ros/ros.h"
#include <vector>
#include <queue>

/**
 * @brief The timing class is used to record the time consuming of a certain code, 
 *        which can be s, ms, or ns.
*/
class Timer {
public:
    Timer(bool average_mode = false, int max_queue_size = 100) : 
                                average_mode(average_mode), 
                                max_queue_size(max_queue_size), 
                                start_time(ros::Time::now()) {}

    void start() {
        start_time = ros::Time::now();
    }

    void stop() {
        end_time = ros::Time::now();
        double elapsed_time = (end_time - start_time).toSec();
        if (average_mode) {
            elapsed_times.push_back(elapsed_time);
            if (elapsed_times.size() > max_queue_size) {
                elapsed_times.pop_front();
            }
        } else {
            single_elapsed_time = elapsed_time;
        }
    }

    double getAverageElapsedTime(const std::string& unit = "s") {
        double sum = 0.0;
        for (auto t : elapsed_times) {
            sum += t;
        }

        if (unit == "s") {
            return sum / elapsed_times.size();
        } else if (unit == "ms") {
            return sum / elapsed_times.size() * 1000;
        } else if (unit == "ns") {
            return sum / elapsed_times.size() * 1e9;
        } else {
            ROS_WARN("Invalid time unit specified. Supported units: s, ms, ns");
        }
        
        
    }

    double getSingleElapsedTime(const std::string& unit = "s") {
        if (unit == "s") {
            return single_elapsed_time;
        } else if (unit == "ms") {
            return single_elapsed_time * 1000.0;
        } else if (unit == "ns") {
            return single_elapsed_time * 1e9;
        } else {
            ROS_WARN("Invalid time unit specified. Supported units: s, ms, ns");
        }
    }

    double getElapsedTimeMSec() {
        return (end_time - start_time).toSec() * 1000.0;
    }

    int64_t getElapsedTimeNSec() {
        return (end_time - start_time).toNSec();
    }

    void printAverageElapsedTime(const std::string& message, const std::string& unit = "s") {
        if (average_mode) {
            ROS_INFO("%s: Average elapsed time: %.4f seconds", message.c_str(), getAverageElapsedTime(unit));
        } else {
            ROS_INFO("%s: Elapsed time: %.4f seconds", message.c_str(), getSingleElapsedTime(unit));
        }
    }

private:
    ros::Time start_time;
    ros::Time end_time;
    bool average_mode;
    int max_queue_size;
    std::deque<double> elapsed_times;
    double single_elapsed_time;
};