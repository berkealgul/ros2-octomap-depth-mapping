#include "demaloc.hpp"
#include "depth_conversions.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cv_bridge/cv_bridge.h>


namespace octomap_depth_mapping
{

OctomapDemap::OctomapDemap(const rclcpp::NodeOptions &options, const std::string node_name): 
    Node(node_name, options),
    fx(524),
    fy(524),
    cx(316.8),
    cy(238.5),
    resolution(0.05),
    padding(1),
    kernel_size(1),
    encoding("mono16"),
    frame_id("map")
{
    fx = this->declare_parameter("camera_model/fx", fx);
    fy = this->declare_parameter("camera_model/fy", fy);
    cx = this->declare_parameter("camera_model/cx", cx);
    cy = this->declare_parameter("camera_model/cy", cy);
    resolution = this->declare_parameter("resolution", resolution);
    encoding = this->declare_parameter("encoding", encoding);
    frame_id = this->declare_parameter("frame_id", frame_id);

    //ocmap = std::make_shared<octomap::OcTree>(0.1);


    rclcpp::QoS qos(rclcpp::KeepLast(3));

    // pubs
    octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>("map_out", qos);

    auto rmw_qos_profile = qos.get_rmw_qos_profile();
    // subs
    depth_sub_.subscribe(this, "image_in", rmw_qos_profile);
    odom_sub_.subscribe(this, "odom_in", rmw_qos_profile);

    // bind subs with ugly way
    sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, nav_msgs::msg::Odometry>>(depth_sub_, odom_sub_, 3);
    sync_->registerCallback(std::bind(&OctomapDemap::demap_callback, this, ph::_1, ph::_2));


    print_params();
    RCLCPP_INFO(this->get_logger(), "Setup is done");
}

void OctomapDemap::demap_callback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg, const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg)
{
    RCLCPP_INFO(this->get_logger(), "callback");
    auto cv_ptr = cv_bridge::toCvCopy(depth_msg, encoding);
    update_map(cv_ptr->image, odom_msg->pose.pose);
    publish_all();
}

void OctomapDemap::publish_all()
{
    octomap_msgs::msg::Octomap msg;

    octomap_msgs::fullMapToMsg(ocmap, msg);
    msg.id = "OcTree";
    msg.header.frame_id = "map";
    
    octomap_publisher_->publish(msg);
}

void OctomapDemap::update_map(const cv::Mat& img, const geometry_msgs::msg::Pose& pose)
{
    tf2::Transform t;
    tf2::fromMsg(pose, t);

    octomap::point3d origin(pose.position.x, pose.position.y, pose.position.z);

    for(int i = 0; i < img.rows; i+=1)
    {
        for(int j = 0; j < img.cols; j+=1)
        {
            //cv::Point minLoc, maxLoc;
            //double min, raw = 0;
            //cv::minMaxLoc(img, &min, &raw, &minLoc, &maxLoc);
            ushort r = img.at<ushort>(i, j);
            double d = depth_to_meters(r);

            tf2::Vector3 p;
            p.setX((j - cx) * d / fx);
            p.setY((i - cy) * d / fy);
            p.setZ(d);
            p = t(p);

            ocmap.insertRay(origin, octomap::point3d(p.getX(), p.getY(), p.getZ()));
        }
    }
}

void OctomapDemap::print_params()
{
    RCLCPP_INFO(this->get_logger(), "--- Launch Parameters ---");
    RCLCPP_INFO_STREAM(this->get_logger(), "fx : " << fx);
    RCLCPP_INFO_STREAM(this->get_logger(), "fy : " << fy);
    RCLCPP_INFO_STREAM(this->get_logger(), "cx : " << cx);
    RCLCPP_INFO_STREAM(this->get_logger(), "cy : " << cy);
    RCLCPP_INFO_STREAM(this->get_logger(), "encoding : " << encoding);
    RCLCPP_INFO_STREAM(this->get_logger(), "resolution : " << resolution);
    RCLCPP_INFO_STREAM(this->get_logger(), "frame_id : " << frame_id);
    RCLCPP_INFO_STREAM(this->get_logger(), "input_image_topic : " << "image_in");
    RCLCPP_INFO_STREAM(this->get_logger(), "input_odom_topic : " << "odom_in");
    RCLCPP_INFO_STREAM(this->get_logger(), "output_map_topic : " << "map_out");
    RCLCPP_INFO(this->get_logger(), "-------------------------");
}   

} // octomap_depth_mapping

RCLCPP_COMPONENTS_REGISTER_NODE(octomap_depth_mapping::OctomapDemap)
