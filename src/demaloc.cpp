#include "demaloc.hpp"
#include <cv_bridge/cv_bridge.h>


namespace octomap_depth_mapping
{

OctomapDemap::OctomapDemap(const rclcpp::NodeOptions &options, const std::string node_name): 
    Node(node_name, options)
{
    octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>("/map", 10);
	pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pc", 10);
    //ocmap = std::make_shared<octomap::OcTree>(0.1);

    rclcpp::QoS qos(rclcpp::KeepLast(3));
    auto rmw_qos_profile = qos.get_rmw_qos_profile();
    depth_sub_.subscribe(this, "/depth_image", rmw_qos_profile);
    odom_sub_.subscribe(this, "/odom", rmw_qos_profile);
    sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, nav_msgs::msg::Odometry>>(depth_sub_, odom_sub_, 3);
    sync_->registerCallback(std::bind(&OctomapDemap::demap_callback, this, ph::_1, ph::_2));



    RCLCPP_INFO(this->get_logger(), "Setup is done");
}

void OctomapDemap::OctomapDemap

void OctomapDemap::demap_callback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg, const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg)
{
    auto cv_ptr = cv_bridge::toCvCopy(depth_msg, "mono16");
    update_map(cv_ptr->image, odom_msg->pose.pose);
    publish_all();
    RCLCPP_INFO(this->get_logger(), "callback");
}

void OctomapDemap::publish_all()
{
    octomap_msgs::msg::Octomap msg;

    octomap_msgs::fullMapToMsg(ocmap, msg);
    msg.id = "OcTree";
    msg.header.frame_id = "map";
    
    octomap_publisher_->publish(msg);

    sensor_msgs::msg::PointCloud pc_msg;

    for(auto& p : pc)
    {
        geometry_msgs::msg::Point32 pp;
        pp.x = p.x();
        pp.y = p.y();
        pp.z = p.z();
        pc_msg.points.push_back(pp);
    }

    pc_msg.header.frame_id = "map";
    
    sensor_msgs::msg::PointCloud2 pc_msg2;
    sensor_msgs::convertPointCloudToPointCloud2(pc_msg, pc_msg2);

    pc_publisher_->publish(pc_msg2);
}

double OctomapDemap::rawDepthToMeters(ushort raw_depth) 
{
    if(raw_depth > 6408)
    {
        return (double)(((2.5-0.9)/(15800.0-6408.0))*raw_depth);
    }        

    return 0;
}

void OctomapDemap::update_map(const cv::Mat& img, const geometry_msgs::msg::Pose& pose)
{
    pc.clear();

    tf2::Transform t, t_i;
    tf2::fromMsg(pose, t);
    t_i = t.inverse();
    
    tf2::Matrix3x3 m(524, 0, 316.8, 0, 524, 238.5, 0, 0, 1);
    auto m_i = m.inverse();
    auto r_i = t_i.getBasis();
    auto v_i = t_i.getOrigin();

    octomap::point3d origin(pose.position.x, pose.position.y, pose.position.z);

    for(int i = 0; i < img.rows; i+=1)
    {
        for(int j = 0; j < img.cols; j+=1)
        {
            cv::Point minLoc, maxLoc;
            double min, raw = 0;
            //cv::minMaxLoc(img, &min, &raw, &minLoc, &maxLoc);
            //raw = (double)img.at<uchar>(i, j);
            ushort r = img.at<ushort>(i, j);
            double d = rawDepthToMeters(r);
            //std::cout << r << " ";

            tf2::Vector3 p(i*d, j*d, d);
            p = tf2::Vector3(m_i[0].dot(p), m_i[1].dot(p), m_i[2].dot(p));
            p+=v_i;
            p = tf2::Vector3(r_i[0].dot(p), r_i[1].dot(p), r_i[2].dot(p));
            octomap::point3d target(p.getX(), p.getY(), p.getZ());

            //std::cout << target << " " << origin << std::endl;
            pc.push_back(target);
            //ocmap.updateNode(target, true);
            ocmap.insertRay(origin, target);
        }
    }
}

} // octomap_depth_mapping

RCLCPP_COMPONENTS_REGISTER_NODE(octomap_depth_mapping::OctomapDemap)