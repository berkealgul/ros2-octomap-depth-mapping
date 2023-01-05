#include <memory>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "octomap/OcTree.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"


class Demaloc
{
public:
    
    Demaloc()
    {
        myfile.open(p1);
        //ocmap = std::make_shared<octomap::OcTree>(0.1);
    }

    octomap::OcTree ocmap = octomap::OcTree(1);
    //std::shared_ptr<octomap::OcTree> ocmap;
    std::ifstream myfile;
    int data_counter = 0;
	std::string p = "/home/berke/Downloads/rgbd-scenes-v2_imgs/rgbd-scenes-v2/imgs/scene_01/";
    std::string p1 = "/home/berke/Downloads/rgbd-scenes-v2_pc/rgbd-scenes-v2/pc/01.pose";

	void read_dataset_once()
	{
        std::stringstream path_stream;
        path_stream << p << std::setw(5) << std::setfill('0') << data_counter << "-depth.png";
        std::string path = path_stream.str();
        
        cv::Mat mat = cv::imread(path, cv::IMREAD_ANYDEPTH);
        // cv::imshow("asd", mat);
        // cv::waitKey(20);
        data_counter++;

        std::string line;
        std::getline(myfile, line);
        double x, y, z, a, b, c ,d;
        std::istringstream iss(line);
        // iss >> a >> b >> c >> d >> x >> y >> z;  

        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation.x = a;
        pose.orientation.y = b;
        pose.orientation.z = c;
        pose.orientation.w = d;

        //process map
        update_map(mat, pose);
	}

    double rawDepthToMeters(int depthValue) 
    {
        if (depthValue < 2047) 
        {
            return (1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
        }
        return 0.0f;
    }

    void update_map(cv::Mat& img, geometry_msgs::msg::Pose& pose)
    {
        tf2::Transform t;
        tf2::fromMsg(pose, t);

        for(int i = 0; i < img.rows; i+=1)
        {
            for(int j = 0; j < img.cols; j+=1)
            {
                cv::Point minLoc, maxLoc;
                double min, raw;
                cv::minMaxLoc(img, &min, &raw, &minLoc, &maxLoc);
                double depth = rawDepthToMeters(raw);
            }
        }

        octomap::point3d origin(pose.position.x, pose.position.y, pose.position.z);
        octomap::point3d target(0, 0, 0);
        //ocmap.insertRay(origin, target);
    }


};
