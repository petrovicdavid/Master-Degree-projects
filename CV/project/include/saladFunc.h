#ifndef __SALADFUNC_H__
#define __SALADFUNC_H__

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "main.h"
#include "BoundingBox.h"
#include "breadFunc.h"



void takeLarger(cv::Mat& input);
cv::Mat saladMask(const cv::Mat inputSaladDish);
cv::Rect saladBoundingBox(const cv::Mat inputImage);


#endif 