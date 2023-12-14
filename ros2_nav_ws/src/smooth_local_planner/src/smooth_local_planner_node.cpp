#include "smooth_local_planner/smooth_local_planner_node.h"

namespace smooth_local_planner
{

SmoothLocalPlannerNode::SmoothLocalPlannerNode(/* args */) : Node("smooth_local_planner")
{
    RCLCPP_INFO(logger_, "Starting SmoothLocalPlannerNode!");

    config_.debug = this->declare_parameter("debug", false);
    config_.targetTopic = this->declare_parameter("targetTopic", "/goal_pose");

    std::vector<double> circle_obs_vec  = this->declare_parameter("circleObs", std::vector<double>{});
    config_.circleObs = Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>>(
            circle_obs_vec.data(), 3, circle_obs_vec.size() / 3);
    config_.penaltyWeight = this->declare_parameter("penaltyWeight", 1000.0);
    config_.pieceLength = this->declare_parameter("pieceLength", 0.5);
    config_.relCostTol = this->declare_parameter("relCostTol", 0.0);

    visualizer_ = std::make_shared<Visualizer>(this->create_sub_node("visualizer"));

    // 订阅目标点数据
    targetSub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", rclcpp::SensorDataQoS(), 
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            if (startGoal_.size() >= 2) {
                startGoal_.clear();
            }
            startGoal_.emplace_back(msg->pose.position.x, msg->pose.position.y);
            RCLCPP_INFO(logger_, "Get target Information:");
            RCLCPP_INFO(logger_, "position: x=%f, y=%f, z=%f", 
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

            visualizer_->visualizeDisks(config_.circleObs);
        });

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&SmoothLocalPlannerNode::plan, this));

}

SmoothLocalPlannerNode::~SmoothLocalPlannerNode()
{
}

void SmoothLocalPlannerNode::plan()
{
    if (startGoal_.size() == 2)
    {
        const int N = (startGoal_.back() - startGoal_.front()).norm() / config_.pieceLength;
        Eigen::Matrix2Xd innerPoints(2, N - 1);
        for (int i = 0; i < N - 1; ++i)
        {
            innerPoints.col(i) = (startGoal_.back() - startGoal_.front()) * (i + 1.0) / N + startGoal_.front();
        }

        std::cout << "path smoother is initilized" << std::endl;
        path_smoother::PathSmoother pathSmoother;
        pathSmoother.setup(startGoal_.front(), startGoal_.back(), N, config_.circleObs, config_.penaltyWeight);
        CubicCurve curve;

        if (std::isinf(pathSmoother.optimize(curve, innerPoints, config_.relCostTol)))
        {
            std::cout << "path smoother failed" << std::endl;
            return;
        }

        if (curve.getPieceNum() > 0)
        {
            std::cout << "path smoother successed" << std::endl;
            std::cout << "piece number is: " << curve.getPieceNum() << std::endl;
            visualizer_->visualize(curve);
        }
    }
}

}; // end namespace smooth_local_planner

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<smooth_local_planner::SmoothLocalPlannerNode>());
    rclcpp::shutdown();
    return 0;
}