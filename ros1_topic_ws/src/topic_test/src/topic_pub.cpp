#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "topic_test/timer.h"

class TopicPub
{
private:
    ros::NodeHandle nh;
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("random_image", 1);
    ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("random_point_cloud", 10);

   void ImagePubTest();
   void pointcloudPubTest();
public:
    TopicPub(/* args */);
    ~TopicPub();
};

TopicPub::TopicPub(/* args */)
{
    ImagePubTest();
    // pointcloudPubTest();
}

TopicPub::~TopicPub()
{
}

/**
 * @brief 生成并发送图像数据进行测试
*/
void TopicPub::ImagePubTest(){
    ros::Duration(0.3).sleep();
    ROS_INFO("----------------Start ImagePubTest-----------------");

    Timer image_timer(true);
    ros::Rate loop_rate(150);
    for (int i = 0; i < 500; ++i) {

        int width = 1000;
        cv::Mat random_image = cv::Mat::zeros(width, width, CV_8UC3);
        cv::randu(random_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));    // 生成的随机图像

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", random_image).toImageMsg();
        msg->header.stamp = ros::Time::now();

        image_timer.start();
        image_pub.publish(msg);     // 发布图像数据
        image_timer.stop();
        ROS_INFO("picture %d * %d publishing time: %.6f ms", width, width, image_timer.getElapsedTimeMSec());

        loop_rate.sleep();
    }
    ROS_INFO("Average publishing time: %.6f ms", image_timer.getAverageElapsedTime("ms"));
}

/**
 * @brief 生产并发布图像数据进行测试
*/
void TopicPub::pointcloudPubTest(){
    ros::Duration(0.3).sleep();
    ROS_INFO("----------------Start pointcloudPubTest----------------");

    Timer pcloud_timer(true);
    ros::Rate loop_rate(10);
    for (int i = 0; i < 500; ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->width = 5;
        cloud->height = 1;
        cloud->points.resize(cloud->width * cloud->height);

        for (size_t i = 0; i < cloud->points.size(); ++i) {     // 生成随机点云
            cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
        }

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header.frame_id = "map";
        output.header.stamp = ros::Time::now();

        pcloud_timer.start();
        point_cloud_pub.publish(output);    // 发布点云数据
        pcloud_timer.stop();
        ROS_INFO("pointcloud %d publishing time: %.6f ms", cloud->width, pcloud_timer.getElapsedTimeMSec());

        loop_rate.sleep();
    }
    ROS_INFO("Average publishing time: %.6f ms", pcloud_timer.getAverageElapsedTime("ms"));
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"hello");
    ros::NodeHandle nh;
    TopicPub TopicPub;

    ros::spin();
    return 0;
}
