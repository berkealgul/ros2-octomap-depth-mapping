#include "demaloc.hpp"
#include "depth_conversions.hpp"
#include <math.h>
#include <cv_bridge/cv_bridge.h>


namespace octomap_depth_mapping
{

OctomapDemap::OctomapDemap(const rclcpp::NodeOptions &options, const std::string node_name): 
    Node(node_name, options),
    fx(524),
    fy(524),
    cx(316.8),
    cy(238.5)
{
    //ocmap = std::make_shared<octomap::OcTree>(0.1);

    frame_to_cam_basis.setRPY(0, M_PI_2, M_PI); // 90 degrees around X axis

    init(); 

    RCLCPP_INFO(this->get_logger(), "Setup is done");
}

void OctomapDemap::init()
{
    rclcpp::QoS qos(rclcpp::KeepLast(3));

    // pubs
    octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>("/map", qos);
	pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pc", qos);

    auto rmw_qos_profile = qos.get_rmw_qos_profile();
    // subs
    depth_sub_.subscribe(this, "/depth_image", rmw_qos_profile);
    odom_sub_.subscribe(this, "/odom", rmw_qos_profile);

    // bind subs with ugly way
    sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, nav_msgs::msg::Odometry>>(depth_sub_, odom_sub_, 3);
    sync_->registerCallback(std::bind(&OctomapDemap::demap_callback, this, ph::_1, ph::_2));
}

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

void OctomapDemap::update_map(const cv::Mat& img, const geometry_msgs::msg::Pose& pose)
{
    pc.clear();

    tf2::Transform t;
    tf2::fromMsg(pose, t);

    t.setRotation(frame_to_cam_basis * t.getRotation());

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
            //std::cout << r << " ";
            //tf2::Vector3 p(i*d, j*d, d);

            tf2::Vector3 p;
            p.setX((j*d - cx) / fx);
            p.setY((i*d - cy) / fy);
            p.setZ(d);
            p = t(p);

            // Pw = R*(Pı * M-1) + T
            // 2d to 3d Tx-Ty are transistiob
            // ray.x = (uv_rect.x - cx() - Tx()) / fx();
            // ray.y = (uv_rect.y - cy() - Ty()) / fy();
            // ray.z = 1.0;

            // https://towardsdatascience.com/what-are-intrinsic-and-extrinsic-camera-parameters-in-computer-vision-7071b72fb8ec#:~:text=The%20extrinsic%20matrix%20is%20a,to%20the%20pixel%20coordinate%20system.
            octomap::point3d target(p.getX(), p.getY(), p.getZ());

            pc.push_back(target);
            //ocmap.updateNode(target, true);
            ocmap.insertRay(origin, target);
        }
    }
}

} // octomap_depth_mapping

RCLCPP_COMPONENTS_REGISTER_NODE(octomap_depth_mapping::OctomapDemap)