#ifndef PLANNER_CONFIG_H_
#define PLANNER_CONFIG_H_

#include "rclcpp/rclcpp.hpp"

namespace smooth_local_planner
{

/**
 * @class PannerConfig
 * @brief Config class for the smoother_local_planner and its cpmponent
*/
struct PlannerConfig
{
    std::string targetTopic;
};



}; // end namespace smooth_local_planner

#endif