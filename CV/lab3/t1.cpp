#include <iostream>
#include <opencv2/highgui.hpp>

int main(int argc, char **argv) {
	
	cv::Mat img = cv::imread("robocup.jpg");
	cv::imshow("Img", img);
	cv::waitKey(0);
	std::cout<<img.type()<<"\n";
	std::cout<<img.channels()<<"\n";
	return 0;
}

