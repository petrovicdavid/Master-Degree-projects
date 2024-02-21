/**
 * Expand Task 1 implementing a max filter and a min filter in two dedicated functions
 * (implemented by you in a separate source file) that manipulate the pixels directly. Such
 * functions accept an integer representing the size of the kernel, and check that it is an odd
 * number. If it is even, the function does not process the image and prints an error message.
 * Apply the filters using several kernel sizes and find the best option that removes the electric
 * cables in the background without corrupting the image too much
*/
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "f2.hpp"

int main(int argc, char **argv)
{
	// input image
	cv::Mat img = cv::imread("image_greyscale.jpg");
	if(img.data==NULL) {
		printf("No image found!\n");
		return 1;
	}
	cv::namedWindow("Example 1");
	cv::imshow("Example 1", img);
	cv::waitKey(0);

	// kernel size
	int ksize;
	std::cout << "Insert kernel size: ";
	std::cin >> ksize;
	
	if(ksize%2==0 || ksize<1) {
		std::cout << "Wrong kernel size!";
		return -1;
	}

	cv::Mat maxImg;
	cv::Mat minImg;
	
	min_filter(img, minImg, ksize);
	cv::namedWindow("Min filter");
	cv::imshow("Min filter", minImg);
	cv::waitKey(0);

	max_filter(img, maxImg, ksize);
	cv::namedWindow("Max filter");
	cv::imshow("Max filter", maxImg);
	cv::waitKey(0);

    cv::Mat medImg;
    cv::medianBlur(img, medImg, ksize);
    cv::namedWindow("Median filter");
    cv::imshow("Median filter", medImg);
    cv::waitKey(0);

    cv::Mat gauImg;
    cv::GaussianBlur(img, gauImg, cv::Size(ksize,ksize), 3, 3);
    cv::namedWindow("Gaussian filter");
    cv::imshow("Gaussian filter", gauImg);
    cv::waitKey(0);
    
	return 0;
}
