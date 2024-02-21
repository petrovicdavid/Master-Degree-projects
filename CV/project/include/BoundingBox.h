
#ifndef __BOUNDINGBOX_H__
#define __BOUNDINGBOX_H__

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <opencv2/xfeatures2d/nonfree.hpp>



	cv::Mat saturationExtraction(cv::Mat& image);
	std::vector<cv::Rect> getBoundingBox(cv::Mat obj);
	void drawBoundingBox(cv::Mat &input,cv::Rect rect);
	

#endif //__BOUNDINGBOX_H__

