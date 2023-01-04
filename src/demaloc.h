#include "octomap/OcTree.h"
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

class Demaloc
{
public:
    
    Demaloc()
    {

    }

private:

    int data_counter = 0;
	std::string p = "/home/berke/Downloads/rgbd-scenes-v2_imgs/rgbd-scenes-v2/imgs/scene_01/";

	void read_dataset_once()
	{
		while(data_counter < 50)
		{
			std::stringstream path_stream;
			path_stream << p << std::setw(5) << std::setfill('0') << data_counter << "-depth.png";
			std::string path = path_stream.str();
			std::cout << path << std::endl;
			cv::Mat mat = cv::imread(path, cv::IMREAD_ANYDEPTH);
			cv::imshow("asd", mat);
			cv::waitKey(20);
			data_counter++;
		}


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
        for(int i = 0; i < img.rows; i+=1)
        {
            for(int j = 0; i < img.cols; i+=1)
            {
                cv::Point minLoc, maxLoc;
                double min, raw;
                cv::minMaxLoc(img, min, raw, minLoc, maxLoc);
                double depth = rawDepthToMeters(raw);
            }
        }


        octomap::point3d origin(pose.position.x, pose.position.y, pose.position.z);
        octomap::point3d target(0, 0, 0);
        ocmap.insertRay(origin, target);
    }

    void publish_map()
    {

    }

    octomap::OcTree ocmap;
};
