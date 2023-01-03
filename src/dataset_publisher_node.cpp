#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/core/core.hpp"
#include <cv_bridge/cv_bridge.h>


using namespace std::chrono_literals;

class DatesetNode : public rclcpp::Node
{
public:
    DatasetNode(): Node("dateset_publisher"), count_(0)
    {
		publisher_ = this->create_publisher<sensor_msgs::msg::Image>("topic", 10);
		timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }	

private:
    void timer_callback()
    {
		auto message = std_msgs::msg::String();
		message.data = "Hello, world! " + std::to_string(count_++);
		RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
		publisher_->publish(message);

	  	cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(request->depth_image, request->depth_image.encoding);
		cv::patchNaNs(cv_ptr->image, 0.0);
		// Datatype depends on your camera image encoding!
		if (cv_ptr->image.at<float>(row, col) >= 0.001)
		{
			depth = cv_ptr->image.at<float>(row, col);
			return true;
		}
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DatesetNode>());
  rclcpp::shutdown();
  return 0;
}