#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <stdio.h>
#include <stdarg.h>
#include <std_msgs/msg/float64.hpp>

class ImagePubSub : public rclcpp::Node
{
public:
	ImagePubSub() : Node("img_pubsub_node")
	{
		parseParameters();
		
		subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
			"/camera/color/image_raw", 1,
			std::bind(&ImagePubSub::processImage, this, std::placeholders::_1));
	}
private:
	void parseParameters();
	void processImage(const sensor_msgs::msg::Image::SharedPtr msg);
	void binaryThresholding(cv::Mat& img);
	cv::Mat maskImage(cv::Mat& img);
	void IPM(cv::Mat& img);
	void erosionDilation(cv::Mat& img);
	void polyFit(cv::Mat& img);

	int width_tolerance_;
	float height_scale_;
	int ipm_width_tolerance_;
	float ipm_height_scale_;
	int ipm_offset_;
	std::vector<double> erode_size_,
					 	dilate_size_;

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ImagePubSub>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}