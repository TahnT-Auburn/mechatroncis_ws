#include <cstring>
#include <string>

#include "img_pubsub_node.h"

void ImagePubSub::parseParameters()
{
	this->declare_parameter<std::string>("test");
	this->get_parameter("test", test_);
}

void ImagePubSub::processImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
	try
	{	
		cv::Mat output;

		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);


		cv::Mat raw_image = cv_ptr->image;
		
		cv::imshow("Raw Image", raw_image);
		cv::waitKey(1);
	}
	catch (cv_bridge::Exception& e)
	{
		RCLCPP_ERROR(this->get_logger(),"CV Bridge Error: %s", e.what());
	}
}