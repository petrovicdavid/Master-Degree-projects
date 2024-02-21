#ifndef __CIRCLES_H__
#define __CIRCLES_H__

#include <iostream>
#include <string.h>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

void find_glass(const cv::Mat &src, std::vector<cv::Vec3f> &circles);

void find_plates(const cv::Mat &src, std::vector<cv::Vec3f> &plates);

void find_salad(const cv::Mat &src, std::vector<cv::Vec3f> &plate);

void circle2black(cv::Mat &image, const std::vector<cv::Vec3f> &circles);

void circlesToVector(const cv::Mat &src, const std::vector<cv::Vec3f> &circles, std::vector<cv::Mat> &images);

void circleToImage(const cv::Mat &src, std::vector<cv::Vec3f> &salad, cv::Mat &image_salad);

void computeBlackImage(const cv::Mat &src, cv::Mat &dst);

void computeImages(const cv::Mat &src, cv::Mat &image_salad, std::vector<cv::Mat> &images);

#endif
