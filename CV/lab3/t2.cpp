#include <iostream>
#include <opencv2/highgui.hpp>

void onMouse(int event, int x, int y, int f, void* userdata);

int main(int argc, char **argv) {
	
	cv::Mat img = cv::imread("robocup.jpg");
	cv::imshow("Img", img);
	cv::setMouseCallback("Img", onMouse, (void*)&img);
	cv::waitKey(0);
	
	return 0;
}

void onMouse(int event, int x, int y, int f, void* userdata) {
	// left button pressed
	if(event == cv::EVENT_LBUTTONDOWN) {
		cv::Mat img = *(cv::Mat*) userdata;
		
		cv::Vec3b point = img.at<cv::Vec3b>(x,y);
		
		std::cout<<"B: "<<(int)point[0]<<std::endl;
		std::cout<<"G: "<<(int)point[1]<<std::endl;
		std::cout<<"R: "<<(int)point[2]<<std::endl;
		std::cout<<std::endl;
	}
}
