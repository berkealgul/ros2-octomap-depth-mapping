#ifndef OCTOMAP_DEPTH_MAPPING_HPP
#define OCTOMAP_DEPTH_MAPPING_HPP

#include <memory>
#include <fstream>
#include <iostream> //for debug

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "octomap/OcTree.h"
#include "octomap/Pointcloud.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// temporary
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp" 
#include "geometry_msgs/msg/point32.hpp"

using namespace std::chrono_literals;
namespace ph = std::placeholders;


namespace octomap_depth_mapping
{

class OctomapDemap : public rclcpp::Node
{
protected:

    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, 
        nav_msgs::msg::Odometry>> sync_;

    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;

    octomap::Pointcloud pc;
    octomap::OcTree ocmap = octomap::OcTree(0.05);
    //std::shared_ptr<octomap::OcTree> ocmap;

    double rawDepthToMeters(ushort);
    void update_map(const cv::Mat&, const geometry_msgs::msg::Pose&);
    void publish_all();
    void demap_callback(const sensor_msgs::msg::Image::ConstSharedPtr&, 
        const nav_msgs::msg::Odometry::ConstSharedPtr&);

public:

    explicit OctomapDemap(const rclcpp::NodeOptions&, const std::string = "octomap_depth_mapping");

};

} // octomap_depth_mapping

#endif // OCTOMAP_DEPTH_MAPPING_HPP