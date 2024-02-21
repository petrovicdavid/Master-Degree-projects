#ifndef __DETECTION_H__
#define __DETECTION_H__

#include <iostream>
#include <string.h>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/types.hpp>
#include <fstream>
#include <opencv2/core/utils/filesystem.hpp>


void writeFile(const std::vector<int> &prediction);
void sendFoodImage(cv::Mat &image, std::vector<int> &prediction, int numberPlates);
void sendLeftover(cv::Mat &image, std::vector<int> &predictionLeftover, int numberPlates);



#endif