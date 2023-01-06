#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "demaloc.h"
#include "rclcpp/rclcpp.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"

using namespace std::chrono_literals;

class DemalocNode : public rclcpp::Node
{
public:
	DemalocNode(): Node("demaloc"), count_(0)
	{
		octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>("/map", 10);
		timer_ = this->create_wall_timer(500ms, std::bind(&DemalocNode::timer_callback, this));
	}	

private:
	void timer_callback()
	{
		demaloc.read_dataset_once();

		octomap_msgs::msg::Octomap msg;
		std::vector<int8_t, std::allocator<int8_t>> map_data;

		msg.resolution = demaloc.ocmap.getResolution();
		octomap_msgs::fullMapToMsgData(demaloc.ocmap, map_data);
		msg.data = map_data;
		msg.binary = true;
		msg.id = "OcTree";
		msg.header.frame_id = "map";
		
		octomap_publisher_->publish(msg);
	}

	Demaloc demaloc;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;
	size_t count_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DemalocNode>());
	rclcpp::shutdown();
	return 0;
}