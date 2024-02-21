#ifndef __FUNCTION_H__
#define __FUNCTION_H__

#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

using namespace std;
using namespace cv;

void orb_detector(const Mat, const Mat, vector<KeyPoint> &, vector<KeyPoint> &, Mat &, Mat &);

//void sift_detector();

void brute_force_matcher(const Mat first, const Mat second, const vector<KeyPoint> &keypoint_1, const vector<KeyPoint> &keypoint_2, const Mat &descriptors_1, const Mat &descriptors_2, vector<DMatch> &good_matches, Mat &output);

void find_min_max(const vector<DMatch> &matches, double &max_dist, double &min_dist);

void count_good_matches(const vector<DMatch> &good_matches, const Mat &mask, int &numGoodMatches, int &numInliers, const int maxDistance);

#endif
