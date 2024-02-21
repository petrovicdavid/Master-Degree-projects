#include "ros/ros.h"
#include "ass2_group25/detector_colored_tables.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

using namespace std;

// vector of result tables ids
vector<int> r;
bool msg_with_content = false;

// wait until a callback is executed
void spin(){
	ros::Rate rate(15); 
	while (ros::ok() && !msg_with_content) {
		ros::spinOnce();
		// controls if detection is done
		if(msg_with_content){
			ROS_INFO("msg arrivato");
			return;
		}
		
		rate.sleep();
	}
}

// callback function for managing received image
void imageCB(const sensor_msgs::ImageConstPtr& msg)
{
	// erase all the ids of past call
	r.clear();
	cv::Mat image, keypoints_mat, sharp_mat, contrast_mat, combi_mat;
	cv_bridge::CvImagePtr cvPtr;
	
	// get the image from the camera
	try
	{
		cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	msg_with_content = true;
	
	cvPtr->image.copyTo(image);
	
	// define the range of color I am interested in
	cv::Scalar lower_red = cv::Scalar(0, 0, 100);
	cv::Scalar upper_red = cv::Scalar(10, 255, 255);

	cv::Scalar lower_blue = cv::Scalar(100, 0, 0);
	cv::Scalar upper_blue = cv::Scalar(255, 50, 50);

	cv::Scalar lower_green = cv::Scalar(0, 100, 0);
	cv::Scalar upper_green = cv::Scalar(50, 255, 50);

	// apply the mask for extract the tables from the image
	cv::Mat mask_red, mask_blue, mask_green;
	cv::inRange(image, lower_red, upper_red, mask_red);
	cv::inRange(image, lower_blue, upper_blue, mask_blue);
	cv::inRange(image, lower_green, upper_green, mask_green);

	// apply the color filtering on the image
	cv::Mat table_red, table_blue, table_green;
	cv::bitwise_and(image, image, table_red, mask_red);
	cv::bitwise_and(image, image, table_blue, mask_blue);
	cv::bitwise_and(image, image, table_green, mask_green);
	
	// define a vector for containing the white pixels and the points of the centers of the tables
	std::vector<cv::Point> whitePixels;
	cv::Point2f centerRed(0, 0);
	cv::Point2f centerBlue(0, 0);
	cv::Point2f centerGreen(0, 0);
	
	// save all the non zero pixel into the vector
    cv::findNonZero(mask_red, whitePixels);
    
    // compute the mean point over the white pixels for the three tables
    for (const auto& point : whitePixels) {
        centerRed += cv::Point2f(point);
    }
    centerRed *= (1.0 / whitePixels.size());
    
    whitePixels.clear();
    
    cv::findNonZero(mask_blue, whitePixels);
    for (const auto& point : whitePixels) {
        centerBlue += cv::Point2f(point);
    }
    centerBlue *= (1.0 / whitePixels.size());
    
    whitePixels.clear();
    
    cv::findNonZero(mask_green, whitePixels);
    for (const auto& point : whitePixels) {
        centerGreen += cv::Point2f(point);
    }
    centerGreen *= (1.0 / whitePixels.size());
	
	// determine the order in which the tables appear by looking at the position of the mean point of the tables
	// the first element in the result vector is the id of the most right table with respect to the robot
	// the last element is the id of the most left table with respect to the robot
	if (centerRed.x > centerBlue.x && centerRed.x > centerGreen.x) {
		r.push_back(3);
		if(centerBlue.x > centerGreen.x){
			r.push_back(1);
			r.push_back(2);
			ROS_INFO("Il tavolo rosso a destra, il tavolo blue in centro, il tavolo verde a sinistra");
		}else{
			r.push_back(2);
			r.push_back(1);
			ROS_INFO("Il tavolo rosso a destra, il tavolo verde in centro, il tavolo blue a sinistra");
		}
	} else if (centerBlue.x > centerRed.x && centerBlue.x > centerGreen.x) {
		r.push_back(1);
		if(centerRed.x > centerGreen.x){
			r.push_back(3);
			r.push_back(2);
			ROS_INFO("Il tavolo blue a destra, il tavolo rosso in centro, il tavolo verde a sinistra");
		}else{
			r.push_back(2);
			r.push_back(3);
			ROS_INFO("Il tavolo blue a destra, il tavolo verde in centro, il tavolo rosso a sinistra");
		}
	} else {
		r.push_back(2);
		if(centerRed.x > centerBlue.x){
			r.push_back(3);
			r.push_back(1);
			ROS_INFO("Il tavolo verde a destra, Il tavolo rosso al centro, Il tavolo blu a sinistra");
		}else{
			r.push_back(1);
			r.push_back(3);
			ROS_INFO("Il tavolo verde a destra, Il tavolo blu al centro, Il tavolo rosso a sinistra");
		}
	}
    
}

// function for managing the order_colored_tables_detector service calls
bool manage(ass2_group25::detector_colored_tables::Request &req, ass2_group25::detector_colored_tables::Response &res) {
	// setup all the necessary object in order to work with images
	ros::NodeHandle n_img;
	image_transport::ImageTransport _imageTransport(n_img);
	
	// subscribe to the topic
	image_transport::Subscriber image_sub;
	image_sub = _imageTransport.subscribe("xtion/rgb/image_color", 10, imageCB);
	
	// wait until a valid message arrived
	spin();
	
	// insert the detection result into the service response
	for(int i=0;i<r.size();i++){
		res.ids.push_back(r[i]);
	}
	
	// reset the boolean variable
	msg_with_content = false;
	return true;
}
			


int main(int argc, char **argv)
{
	ros::init(argc, argv, "order_colored_tables_detector");
	
	ros::NodeHandle n;
	
	ros::ServiceServer service = n.advertiseService("colored_tables_detection", manage);
	
	ROS_INFO("Ready to detect the order of the colored tables");
	
	ros::spin();
	
	
	return 0;
}
