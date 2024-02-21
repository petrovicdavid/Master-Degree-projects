#include <iostream>
#include <opencv2/highgui.hpp>

#define NBHD_X 9 
#define NBHD_Y 9

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
		cv::Mat img_out = img.clone();
		
		if(y+NBHD_Y>img_out.rows || x+NBHD_X>img_out.cols)
			return;
		
		cv::Rect rect(x, y, NBHD_X, NBHD_Y);
		cv::Scalar mean = cv::mean(img_out(rect));
		std::cout<<"Mean: "<<mean<<std::endl;
	}
}
