#ifndef __BREADFUNC_H__
#define __BREADFUNC_H__

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "main.h"
#include "BoundingBox.h"



bool isMostlyWhite(const cv::Mat& grayImage, float threshold);
bool isMostlyWhite(const cv::Mat& grayImage, const cv::Rect& rect, double threshold);
cv::Mat gammaCorrection(const cv::Mat& inputImage, float gamma);
void imfill(cv::Mat& image);
cv::Mat motrphologyOpeClose(cv::Mat input, int SizeOpen, int SizeClose);
cv::Mat initialbreadMask(const cv::Mat& inputImage);
cv::Mat breadMask(const cv::Mat& inputImage);
cv::Rect breadBoundingBox(const cv::Mat inputImage);
cv::Mat filterAreasLower(cv::Mat& img, int threshold);
cv::Mat filterAreasUpper(cv::Mat& img, int threshold);


#endif 