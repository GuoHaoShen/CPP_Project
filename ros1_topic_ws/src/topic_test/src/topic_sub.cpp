#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <queue>
#include <sensor_msgs/PointCloud2.h>

class TopicSubscriber {
public:
    TopicSubscriber() {
        image_sub_ = nh_.subscribe("random_image", 1, &TopicSubscriber::imageCallback, this);
        point_cloud_sub_ = nh_.subscribe("random_point_cloud", 10, &TopicSubscriber::pointCloudCallback, this);
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        ros::Time receive_time = ros::Time::now();
        ros::Duration latency = receive_time - msg->header.stamp;
        ROS_INFO("Received image. Latency: %.4f ms", latency.toSec() * 1000);

        image_elapsed_times_.push_back(latency.toSec());
        double sum = 0.0;
        for (auto t : image_elapsed_times_) {
            sum += t;
        }
        ROS_INFO("Average subscriber elapsed time: %.4f ms", sum / image_elapsed_times_.size() * 1000);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        ros::Time receive_time = ros::Time::now();
        ros::Duration latency = receive_time - msg->header.stamp;
        ROS_INFO("Received image. Latency: %.4f ms", latency.toSec() * 1000);

        pcloud_elapsed_times_.push_back(latency.toSec());
        double sum = 0.0;
        for (auto t : pcloud_elapsed_times_) {
            sum += t;
        }
        ROS_INFO("Average subscriber elapsed time: %.4f ms", sum / pcloud_elapsed_times_.size() * 1000);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Subscriber point_cloud_sub_;
    std::deque<double> image_elapsed_times_;
    std::deque<double> pcloud_elapsed_times_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_subscriber_node");
    TopicSubscriber subscriber;

    ros::spin();

    return 0;
}
