#ifndef OCTOMAP_DEPTH_MAPPING_HPP
#define OCTOMAP_DEPTH_MAPPING_HPP

#include <memory>
#include <fstream>
#include <iostream> //for debug

#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "octomap/OcTree.h"
#include "octomap/Pointcloud.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"


using namespace octomap;


class Demaloc
{
public:
    
    Demaloc();

    octomap::KeyRay keyray;
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;

    octomap::Pointcloud pc;
    octomap::OcTree ocmap = octomap::OcTree(0.05);
    //std::shared_ptr<octomap::OcTree> ocmap;
    std::ifstream myfile;
    int data_counter = 0;
	std::string p = "/home/berke/Downloads/rgbd-scenes-v2_imgs/rgbd-scenes-v2/imgs/scene_01/";
    std::string p1 = "/home/berke/Downloads/rgbd-scenes-v2_pc/rgbd-scenes-v2/pc/01.pose";

	void read_dataset_once();

    double rawDepthToMeters(ushort raw_depth);

    void update_map(cv::Mat& img, geometry_msgs::msg::Pose& pose);
};

#endif // OCTOMAP_DEPTH_MAPPING_HPP