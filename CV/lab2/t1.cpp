#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char **argv)
{
	cv::Mat img = cv::imread("image.jpg");
	if(img.data==NULL) {
		printf("No image found!\n");
		return 1;
	}
	cv::namedWindow("Example 1");
	cv::imshow("Example 1", img);
	cv::waitKey(0);
	
	cv::Mat g_img;
	cv::cvtColor(img, g_img, cv::COLOR_BGR2GRAY);
	
	cv::namedWindow("Example 2");
	cv::imshow("Example 2", g_img);
	cv::waitKey(0);
	
	cv::imwrite("image_greyscale.jpg", g_img);
	
	return 0;
}

