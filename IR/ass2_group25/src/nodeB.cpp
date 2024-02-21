#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ass2_group25/detection.h"
#include <ass2_group25/objects.h>
#include <vector>

using namespace std;

auto tfBuffer = make_shared<tf2_ros::Buffer>();

// vector of detected objects on the table
vector<ass2_group25::objects> r;
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

// callback function for reading the apriltags
void apriltagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
	// remove past apriltag detections
	r.clear();
	geometry_msgs::TransformStamped transformStamped;
	try
	{
		transformStamped = tfBuffer->lookupTransform("base_footprint", "xtion_rgb_optical_frame", ros::Time(0));
	}
	catch (tf2::TransformException& ex)
	{
		ROS_WARN("%s", ex.what());
	}
	
	
	for (const auto& detection : msg->detections)
    {
		msg_with_content = true;
		
		// apply the transformation from camera frame to base_footprint
		geometry_msgs::PoseStamped poseInCameraFrame;
		poseInCameraFrame.pose = detection.pose.pose.pose;
		poseInCameraFrame.header.stamp = ros::Time(0);
		poseInCameraFrame.header.frame_id = "xtion_rgb_optical_frame";
		geometry_msgs::PoseStamped poseInBaseLinkFrame;
		
		try
		{
		  tf2::doTransform(poseInCameraFrame, poseInBaseLinkFrame, transformStamped);
		}
		catch (tf2::TransformException& ex)
		{
		  continue;
		}				
		
		// print the result
		ROS_INFO("ID: %d", detection.id[0]);
		ROS_INFO("Pose (x, y, z): (%f, %f, %f)", detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z);
    
		// insert the detected apriltag into the vector
		ass2_group25::objects det;
		det.id = detection.id[0];
		det.position = poseInBaseLinkFrame;
		r.push_back(det);
	}
}

// function for managing the objects_detector service calls
bool manage(ass2_group25::detection::Request &req, ass2_group25::detection::Response &res) {
	ros::NodeHandle n;
	
	tf2_ros::TransformListener tfListener(*tfBuffer);
	
	// subscribe to tag_detections topic
	ros::Subscriber s = n.subscribe("tag_detections",1000, apriltagCallback);
	
	// wait until a valid message arrived 
	spin();
	
	// insert the detection result into the service response
	for(int i=0;i<r.size();i++){
		res.detection.push_back(r[i]);
	}
	
	// reset the boolean variable
	msg_with_content = false;
	
	return true;
}
			


int main(int argc, char **argv)
{
	ros::init(argc, argv, "objects_detector");
	
	ros::NodeHandle n;
	
	ros::ServiceServer service = n.advertiseService("detection", manage);
	
	ROS_INFO("Ready to detect objects");
	
	ros::spin();
	
	
	return 0;
}
