#ifndef __FUNCTION_H__
#define __FUNCTION_H__

#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <string.h>

using namespace std;
using namespace cv;

void brute_force_matcher(const Mat first, const Mat second, const vector<KeyPoint> &keypoint_1, const vector<KeyPoint> &keypoint_2, const Mat &descriptors_1, const Mat &descriptors_2, vector<DMatch> &good_matches);

void find_min_max(const vector<DMatch> &matches, double &max_dist, double &min_dist);

int open_images(vector<Mat> &images, const vector<String> &fn);

void project_images(const vector<Mat> &images, vector<Mat> &images_projected, const int FoV);

void orb_detection_computation(const vector <Mat> &images_projected, vector<vector<KeyPoint>> &images_keypoints, vector<Mat> &images_descriptors);


#endif
