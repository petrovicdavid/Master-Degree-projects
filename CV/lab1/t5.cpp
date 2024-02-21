#include <iostream>
#include <opencv2/highgui.hpp>

int main(int argc, char** argv) {
	
	cv::Mat img1(256, 256, CV_8UC1);
	cv::Mat img2(256, 256, CV_8UC1);

	for(int i=0; i<img1.rows; i++) {
		for(int j=0; j<img1.cols; j++) {
			img1.at<uchar>(i,j) = i;
			img2.at<uchar>(i,j) = j;
		}
	}
	cv::namedWindow("Example 1");
	cv::namedWindow("Example 2");

	cv::imshow("Example 1", img1);
	cv::imshow("Example 2", img2);
	cv::waitKey(0);

	cv::Mat img3(300, 300, CV_8UC1);
	cv::Mat img4(300, 300, CV_8UC1);
	bool f3o = true, f4o = true;

	for(int i=0; i<img3.rows; i++) {
		for(int j=0; j<img3.cols; j++) {
			// switch between colors
			if (j%20==0) {
				f3o = !f3o;
			}
			if (j%50==0) {
				f4o = !f4o;
			}
			// pixel color
			if(f3o)	img3.at<uchar>(i,j) = 0;
			else	img3.at<uchar>(i,j) = 255;
			
			if(f4o)	img4.at<uchar>(i,j) = 0;
			else	img4.at<uchar>(i,j) = 255;
		}
		// odd cells in img3
		f3o = !f3o;
		// inverting chess cell color
		if((i+1)%20==0)	f3o = !f3o;
		if((i+1)%50==0)	f4o = !f4o;
	}

	cv::namedWindow("Example 3");
	cv::namedWindow("Example 4");

	cv::imshow("Example 3", img3);
	cv::imshow("Example 4", img4);
	cv::waitKey(0);
	
	return 0;
}
