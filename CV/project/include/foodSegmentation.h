#ifndef __FOODSEGMENTATION_H__
#define __FOODSEGMENTATION_H__


#include <sstream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"
#include <map>
#include <iostream>
#include "UmboFunct.h"
#include "BoundingBox.h"
#include "breadFunc.h"
#include "circles.h"
#include "saladFunc.h"


bool customSort(int a, int b);
cv::Mat postProcessingmask(cv::Mat mask, int scaleClose, int threshold, int scaleDilate, int scaleCloseLast);
cv::Mat preProcess(cv::Mat input);
cv::Mat segmenentFoodIstance(int label, cv::Mat inputPreP);
void addMask(cv::Mat& image, const cv::Mat& mask);
cv::Mat segmetFoodColorLabel(std::vector<int> labels, cv::Mat imagePalte);
cv::Mat combineMasks(std::vector<cv::Mat> masks);
std::map<int, cv::Rect> extractBoxes(cv::Mat& segmentaiton);

#endif //__FOODSEGMENTATION_H__