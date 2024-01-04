#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

#include "smooth_local_planner/cubic_curve.hpp"

/**
 * @class Visualizer
 * @brief Used to visualize various graphics
*/
class Visualizer
{
private:
    rclcpp::Node::SharedPtr node_;

    // These are publishers for path, waypoints on the trajectory,
    // the entire trajectory, the mesh of free-space polytopes,
    // the edge of free-space polytopes, and spheres for safety radius
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr routePub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wayPointsPub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectoryPub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr meshPub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr edgePub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spherePub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr diskPub_;

public:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speedPub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr thrPub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr tiltPub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bdrPub_;

public:

    Visualizer(rclcpp::Node::SharedPtr node) : node_(node)
    {
        routePub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/visualizer/route", 10);
        wayPointsPub_ = node->create_publisher<visualization_msgs::msg::Marker>("/visualizer/waypoints", 10);
        trajectoryPub_ = node->create_publisher<visualization_msgs::msg::Marker>("/visualizer/trajectory", 10);
        meshPub_ = node->create_publisher<visualization_msgs::msg::Marker>("/visualizer/mesh", 1000);
        edgePub_ = node->create_publisher<visualization_msgs::msg::Marker>("/visualizer/edge", 1000);
        spherePub_ = node->create_publisher<visualization_msgs::msg::Marker>("/visualizer/spheres", 1000);
        diskPub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/visualizer/disks", 1000);
        speedPub_ = node->create_publisher<std_msgs::msg::Float32>("/visualizer/speed", 1000);
        thrPub_ = node->create_publisher<std_msgs::msg::Float32>("/visualizer/total_thrust", 1000);
        tiltPub_ = node->create_publisher<std_msgs::msg::Float32>("/visualizer/tilt_angle", 1000);
        bdrPub_ = node->create_publisher<std_msgs::msg::Float32>("/visualizer/body_rate", 1000);
    }

    ~Visualizer(){
    }

    /**
     * @brief 可视化三次样条曲线 
    */
    inline void visualize(const CubicCurve &curve)
    {
        visualization_msgs::msg::Marker routeMarker, wayPointsMarker, trajMarker;

        routeMarker.id = 0;
        routeMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
        routeMarker.header.stamp = node_->now();
        routeMarker.header.frame_id = "odom";
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::msg::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.00;
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 0.00;
        routeMarker.color.a = 1.00;
        routeMarker.scale.x = 0.1;

        wayPointsMarker = routeMarker;
        wayPointsMarker.id = -wayPointsMarker.id - 1;
        wayPointsMarker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 1.00;
        wayPointsMarker.color.g = 0.00;
        wayPointsMarker.color.b = 0.00;
        wayPointsMarker.scale.x = 0.35;
        wayPointsMarker.scale.y = 0.35;
        wayPointsMarker.scale.z = 0.35;

        trajMarker = routeMarker;
        trajMarker.header.frame_id = "odom";
        trajMarker.id = 0;
        trajMarker.ns = "trajectory";
        trajMarker.color.r = 0.00;
        trajMarker.color.g = 0.50;
        trajMarker.color.b = 1.00;
        trajMarker.scale.x = 0.20;

        if (curve.getPieceNum() > 0)
        {
            Eigen::MatrixXd wps = curve.getPositions();
            for (int i = 0; i < wps.cols(); i++)
            {
                geometry_msgs::msg::Point point;
                point.x = wps.col(i)(0);
                point.y = wps.col(i)(1);
                point.z = 0.0;
                wayPointsMarker.points.push_back(point);
            }

            wayPointsPub_->publish(wayPointsMarker);
        }

        if (curve.getPieceNum() > 0)
        {
            double T = 0.01;
            Eigen::Vector2d lastX = curve.getPos(0.0);
            for (double t = T; t < curve.getTotalDuration(); t += T)
            {
                geometry_msgs::msg::Point point;
                Eigen::Vector2d X = curve.getPos(t);
                point.x = lastX(0);
                point.y = lastX(1);
                point.z = 0.0;
                trajMarker.points.push_back(point);
                point.x = X(0);
                point.y = X(1);
                point.z = 0.0;
                trajMarker.points.push_back(point);
                lastX = X;
            }
            trajectoryPub_->publish(trajMarker);
        }
    }

    /**
     * @brief 可视化圆柱体
    */
    inline void visualizeDisks(const Eigen::Matrix3Xd &disks)
    {
        visualization_msgs::msg::Marker diskMarker;
        visualization_msgs::msg::MarkerArray diskMarkers;

        diskMarker.type = visualization_msgs::msg::Marker::CYLINDER;
        diskMarker.header.stamp = node_->now();
        diskMarker.header.frame_id = "odom";
        diskMarker.pose.orientation.w = 1.00;
        diskMarker.action = visualization_msgs::msg::Marker::ADD;
        diskMarker.ns = "disks";
        diskMarker.color.r = 1.00;
        diskMarker.color.g = 0.00;
        diskMarker.color.b = 0.00;
        diskMarker.color.a = 0.70;

        for (int i = 0; i < disks.cols(); ++i)
        {
            diskMarker.id = i;
            diskMarker.pose.position.x = disks(0, i);
            diskMarker.pose.position.y = disks(1, i);
            diskMarker.pose.position.z = 0.5;
            diskMarker.scale.x = disks(2, i) * 2.0;
            diskMarker.scale.y = disks(2, i) * 2.0;
            diskMarker.scale.z = 1.0;
            diskMarkers.markers.push_back(diskMarker);
        }

        diskPub_->publish(diskMarkers);
    }
};


#endif