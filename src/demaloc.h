#include "octomap/AbstractOccupancyOcTree.h"
#include <opencv2/opencv.hpp>
#include "geometry_msgs/Pose.h"
#include "rclcpp/rclcpp.hpp"

class Demaloc
{
public:
    

    Demaloc()
    {

    }

    void update_map(const cv::Mat& img, const geometry_msgs& pose)
    {
        
        //ocmap.castRay();
    }

    void publish_map()
    {

    }

private:
    octomap::Octree ocmap;


};
