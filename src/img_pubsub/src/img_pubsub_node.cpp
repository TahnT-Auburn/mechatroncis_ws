#include <cstring>
#include <string>

#include "img_pubsub_node.h"

void ImagePubSub::parseParameters()
{	
	//Mask parameters
	this->declare_parameter<int>("width_tolerance");
	this->get_parameter("width_tolerance", width_tolerance_);

	this->declare_parameter<float>("height_scale");
	this->get_parameter("height_scale", height_scale_);

	//IPM parameters
	this->declare_parameter<int>("ipm_width_tolerance");
	this->get_parameter("ipm_width_tolerance", ipm_width_tolerance_);

	this->declare_parameter<float>("ipm_height_scale");
	this->get_parameter("ipm_height_scale", ipm_height_scale_);

	this->declare_parameter<int>("ipm_offset");
	this->get_parameter("ipm_offset", ipm_offset_);

	//Erosion and Dilation parameters
	this->declare_parameter<std::vector<double>>("erode_size");
	this->get_parameter("erode_size", erode_size_);

	this->declare_parameter<std::vector<double>>("dilate_size");
	this->get_parameter("dilate_size", dilate_size_);
}

void ImagePubSub::processImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
	try
	{
		//Covert to opencv image
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv::Mat raw_img = cv_ptr->image;
		cv::Mat raw_clone = raw_img.clone();
		cv::Mat raw_clone2 = raw_img.clone();
		//Apply binary thresholding
		binaryThresholding(raw_clone);

		//Apply mask
		cv::Mat mask = maskImage(raw_clone);
		//Dialte
		erosionDilation(raw_clone);

		cv::Mat mask_clone = raw_clone.clone();

		//Apply IPM
		IPM(raw_clone2);
		IPM(mask_clone);

		//Display
		cv::imshow("Raw", raw_img);
		cv::waitKey(1);
		cv::imshow("Binary Thresholding", raw_clone);
		cv::waitKey(1);
		// cv::imshow("Mask", mask);
		// cv::waitKey(1);
		cv::imshow("IPM", raw_clone2);
		cv::waitKey(1);
		cv::imshow("IPM2", mask_clone);
		cv::waitKey(1);
	}
	catch (cv_bridge::Exception& e)
	{
		RCLCPP_ERROR(this->get_logger(),"CV Bridge Error: %s", e.what());
	}
}

void ImagePubSub::binaryThresholding(cv::Mat& img)
{
	cv::Mat hsv_img;

	cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);
	cv::Scalar hsv_min(79,38,126);
	cv::Scalar hsv_max(132,155,256);

	cv::inRange(hsv_img, hsv_min, hsv_max, img);
}

cv::Mat ImagePubSub::maskImage(cv::Mat& img)
{	
	//Generate mask
	cv::Mat mask(img.size().height, img.size().width, CV_8UC1, cv::Scalar(0));

	//Define boundary points
	cv::Point p1 = cv::Point(0, img.size().height);															//Bottom-Left
	cv::Point p2 = cv::Point(img.size().width/2 - width_tolerance_, img.size().height*height_scale_);		//Top-Left
	cv::Point p3 = cv::Point(img.size().width/2 + width_tolerance_, img.size().height*height_scale_);		//Top-Right
	cv::Point p4 = cv::Point(img.size().width, img.size().height);											//Bottom-Right

    //Fill mask
    cv::Point vertice_points[] = {p1, p2, p3, p4};
    std::vector<cv::Point> vertices(vertice_points, vertice_points + sizeof(vertice_points) / sizeof(cv::Point));
    std::vector<std::vector<cv::Point>> vertices_to_fill;
    vertices_to_fill.push_back(vertices);
    cv::fillPoly(mask, vertices_to_fill, cv::Scalar(255));

	//Apply mask
	cv::bitwise_and(img, mask, img);

	return mask;
}

void ImagePubSub::IPM(cv::Mat& img)
{	
	cv::Point p1 = cv::Point(img.size().width/2 - ipm_width_tolerance_, img.size().height*ipm_height_scale_);		//Top-Left
	cv::Point p2 = cv::Point(0, img.size().height);															//Bottom-Left
	cv::Point p3 = cv::Point(img.size().width, img.size().height);												//Bottom-Right
	cv::Point p4 = cv::Point(img.size().width/2 + ipm_width_tolerance_, img.size().height*ipm_height_scale_);		//Top-Right
	
	cv::Point2f src_points[] = {p1,p2,p3,p4};

	//Define warping points
	cv::Point pp1 = cv::Point(ipm_offset_, 0);										//Top-Left
	cv::Point pp2 = cv::Point(ipm_offset_, img.size().height);						//Bottom-Left	
	cv::Point pp3 = cv::Point(img.size().width - ipm_offset_, img.size().height);	//Bottom-Right
	cv::Point pp4 = cv::Point(img.size().width - ipm_offset_, 0);					//Top-Right

	cv::Point2f dst_points[] = {pp1,pp2,pp3,pp4};

	//Perspective Transform
	cv::Mat pers_trans = cv::getPerspectiveTransform(src_points, dst_points);

	//Warp input image
	cv::warpPerspective(img, img, pers_trans, cv::Size(img.size().width, img.size().height), cv::INTER_LINEAR);
}

void ImagePubSub::erosionDilation(cv::Mat& img)
{	
	cv::Mat eros_elem=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(int(erode_size_[0]),int(erode_size_[1]))); //Errosion Element
	cv::Mat dial_elem=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(int(dilate_size_[0]),int(dilate_size_[1]))); //Dilation Element
	cv::erode(img,img,eros_elem);
	cv::dilate(img,img,dial_elem);
}

void ImagePubSub::polyFit(cv::Mat& img)
{	
	std::vector<double> hist;
	
	//Histogram
	hist = sum(img[int(img.size().height/2):,:])
}