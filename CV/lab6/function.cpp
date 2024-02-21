#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include "function.h"

using namespace std;
using namespace cv;

// detect the keypoints and compute the descriptors of two images passed as arguments by using orb
void orb_detector(const Mat first, const Mat second, vector<KeyPoint> &keypoint_1, vector<KeyPoint> &keypoint_2, Mat &descriptors_1, Mat &descriptors_2)
{
  Ptr<FeatureDetector> detector = ORB::create();

  detector->detectAndCompute(first, Mat(), keypoint_1, descriptors_1);
	detector->detectAndCompute(second, Mat(), keypoint_2, descriptors_2);
}

/*
void sift_detector(const Mat first, const Mat second, vector<KeyPoint> &keypoint_1, vector<KeyPoint> &keypoint_2, Mat &descriptors_1, Mat &descriptors_2)
{
  Ptr<SiftFeatureDetector> detector = SiftFeatureDetector::create();
  detector->detectAndCompute(first, Mat(), keypoint_1, descriptors_1);
	detector->detectAndCompute(second, Mat(), keypoint_2, descriptors_2);
}
*/

// find min and max distances of the matches
void find_min_max(const vector<DMatch> &matches, double &max_dist, double &min_dist)
{
  max_dist = 0;
  min_dist = 100;

  for( int i = 0; i < matches.size(); i++ )
	{
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}
}

// apply features matching with brute force approach, filter the matches and draw them
void brute_force_matcher(const Mat first, const Mat second, const vector<KeyPoint> &keypoint_1, const vector<KeyPoint> &keypoint_2, const Mat &descriptors_1, const Mat &descriptors_2, vector<DMatch> &good_matches, Mat &output)
{
  BFMatcher matcher(NORM_HAMMING);
	vector<DMatch> matches, matches_filtered;

	matcher.match(descriptors_1, descriptors_2, matches);

  double max_dist, min_dist;

  find_min_max(matches, max_dist, min_dist);

	for( int i = 0; i < matches.size(); i++ )
	{
		if( matches[i].distance < 5*min_dist)
		{
			good_matches.push_back( matches[i]);
		}
	}

	drawMatches(first, keypoint_1, second, keypoint_2, good_matches, output);
}

// count the number of inliers and good matches for evaluating the match between the images
void count_good_matches(const vector<DMatch> &good_matches, const Mat &mask, int &numGoodMatches, int &numInliers, const int maxDistance)
{
  numGoodMatches = 0;
  numInliers = 0;
  for (size_t i = 0; i < mask.rows; i++)
	{
	    if (mask.at<uchar>(i, 0) == 1)
	    {
	        numInliers++;
	        if (good_matches[i].distance > 0.7 * maxDistance)
	        {
	            numGoodMatches++;
	        }
	    }
	}
}
