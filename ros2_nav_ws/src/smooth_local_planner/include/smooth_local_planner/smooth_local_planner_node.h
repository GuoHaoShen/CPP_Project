#ifndef SMOOTHER_LOCAL_PLANNER_NODE_H_
#define SMOOTHER_LOCAL_PLANNER_NODE_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "smooth_local_planner/visualizer.hpp"
#include "smooth_local_planner/path_smoother.hpp"

namespace smooth_local_planner
{

/**
 * @struct PannerConfig
 * @brief Config class for the smoother_local_planner and its cpmponent
*/
struct PlannerConfig
{
    bool debug;

    /**
     * @brief 目标点话题名
    */
    std::string targetTopic;

    /**
     * @brief 圆形障碍物
    */
    Eigen::Matrix3Xd circleObs;

    /**
     * @brief 障碍权重
    */
    double penaltyWeight;

    /**
     * @brief 路径切片长度
    */
    double pieceLength;

    /**
     * @brief 迭代代价阈值
    */
    double relCostTol;
};

/**
 * @class SmoothLocalPlannerNode
 * @brief A class that implements smooth local path planning
*/
class SmoothLocalPlannerNode : public rclcpp::Node
{
private:
    rclcpp::Logger logger_{rclcpp::get_logger("smooth_local_planner")};

    std::shared_ptr<Visualizer> visualizer_;

    std::vector<Eigen::Vector2d> startGoal_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub_;
    rclcpp::TimerBase::SharedPtr timer_;

    PlannerConfig config_;  // config of  SmoothLocalPlanner

    /**
     * @brief 实现局部路径规划函数
    */
    void plan();

public:

    SmoothLocalPlannerNode(/* args */);
    ~SmoothLocalPlannerNode();
};

}; // end namespace smooth_local_planner

#endif // SMOOTHER_LOCAL_PLANNER_NODE_H_