#ifndef __P_MEASURE_H__
#define __P_MEASURE_H__

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <map>
#include <fstream>

/**
 * Mean Average Precision over all dataset
*/
double mAP();

/**
 * Intersection over Union for rectangles
 * gtr  ground truth rectangle
 * pr   predicted rectangle
*/
double IoU(cv::Rect gtr, cv::Rect pr);

/**
 * Mean Intersection over Union for blobs in all dataset.
*/
void mIoU();

/**
 * Leftover estimation for each class over all dataset.
*/
void leftover();

#endif