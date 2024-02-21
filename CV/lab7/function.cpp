#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <string.h>
#include "function.h"
#include "PanoramicUtils/header/panoramic_utils.h"

using namespace std;
using namespace cv;


// apply features matching between two imagees with brute force approach with Hammming distance and filter the matches
void brute_force_matcher(const Mat first, const Mat second, const vector<KeyPoint> &keypoint_1, const vector<KeyPoint> &keypoint_2, const Mat &descriptors_1, const Mat &descriptors_2, vector<DMatch> &good_matches)
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
}

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

// open all images whose paths is passed as a vector of string
int open_images(vector<Mat> &images, const vector<String> &fn)
{
  int exit_code = 0;
  for(size_t k=0;k<fn.size();k++)
	{
		cout<<"Image: "<<fn[k]<<endl;
		images.push_back(imread(fn[k]));
		if(images[k].empty())
		{
			exit_code = 1;
      break;
		}
	}
  return exit_code;
}

// projecte all images in a cylindrical projection
void project_images(const vector<Mat> &images, vector<Mat> &images_projected, const int FoV)
{
  for(size_t k=0;k<images.size();k++)
	{
		images_projected.push_back(cylindricalProj(images[k], FoV));
	}
}

// detect and compute keypoints and descriptors of all the images by using ORB
void orb_detection_computation(const vector <Mat> &images_projected, vector<vector<KeyPoint>> &images_keypoints, vector<Mat> &images_descriptors)
{
  Ptr<FeatureDetector> detector = ORB::create();

  for(size_t k=0;k<images_projected.size();k++)
	{
		vector<KeyPoint> keypoints;
		Mat descriptors;
		detector->detectAndCompute(images_projected[k], Mat(), keypoints, descriptors);
		images_keypoints.push_back(keypoints);
		images_descriptors.push_back(descriptors);
	}
}
