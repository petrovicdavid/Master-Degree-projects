#include <iostream>
#include <opencv2/highgui.hpp>
#include <cmath>

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

        cv::Mat img_mask = cv::Mat::zeros(cv::Size(img.cols,img.rows), CV_8UC1);
        uchar T = 75;
        for(int i=0; i<img.rows; i++) {
            for(int j=0; j<img.cols; j++) {
                cv::Vec3b vec = img.at<cv::Vec3b>(i,j);
				
                if(std::abs(vec[0]-mean[0])<T && std::abs(vec[1]-mean[1])<T && std::abs(vec[2]-mean[2])<T)
                    img_mask.at<uchar>(i,j)=255;
                else
                    img_mask.at<uchar>(i,j)=0;
            }
        }

        cv::imshow("Mask", img_mask);
        cv::waitKey(0);
	}
}
