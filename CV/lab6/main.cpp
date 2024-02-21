#include <iostream>
#include <string.h>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include "function.h"

using namespace std;
using namespace cv;

int exit_code = 0;

int main(int argc, char** argv)
{
	// check command line arguments
	if(argc != 3)
	{
		cerr << "Provide paths of two images !!!" << endl;
		exit_code = 1;
		return exit_code;
	}
	if(strcmp(argv[1],argv[2]) == 0)
	{
		cerr << "!!! The two images are equal !!!" << endl;
		exit_code = 2;
		return exit_code;
	}

	Mat first, second;
	first = imread(argv[1]);
	second = imread(argv[2]);

	if (first.empty())
	{
		cerr << "!!! Failed imread(): image "<<argv[1]<<" not found !!!" << endl;
		exit_code = 3;
		return exit_code;
	}
	if (second.empty())
	{
		cerr << "!!! Failed imread(): image "<<argv[2]<<" not found !!!" << endl;
		exit_code = 3;
		return exit_code;
	}


	vector<KeyPoint> orb_keypoint_1, orb_keypoint_2;
	Mat orb_descriptors_1, orb_descriptors_2, img_matches;
	vector<DMatch> good_matches;

  // detect the keypoints and compute descriptors of the two images
	orb_detector(first,second,orb_keypoint_1,orb_keypoint_2,orb_descriptors_1,orb_descriptors_2);

	// match the features using brute force approach
	brute_force_matcher(first,second,orb_keypoint_1,orb_keypoint_2,orb_descriptors_1,orb_descriptors_2,good_matches,img_matches);

	// show the features matching between the two images
	imshow( "Matches", img_matches );
	waitKey(0);


	// evaluate quality of matching
	vector<Point2f> srcPoints;
	vector<Point2f> dstPoints;

	// Convert good matches to vectors of points
	for (size_t i = 0; i < good_matches.size(); i++)
	{
	    srcPoints.push_back(orb_keypoint_1[good_matches[i].queryIdx].pt);
	    dstPoints.push_back(orb_keypoint_2[good_matches[i].trainIdx].pt);
	}

	// Find homography matrix
	Mat mask;
	double maxDist, minDist;

	Mat H = findHomography(srcPoints, dstPoints, RANSAC, 3.0, mask);


	// find max and min between the matches
	find_min_max(good_matches, maxDist, minDist);

	// Set max distance threshold
	double maxDistance = 0.7 * maxDist + 0.3 * minDist;


	// Count number of good matches and inliers
	int numGoodMatches, numInliers;

	count_good_matches(good_matches, mask, numGoodMatches, numInliers, maxDistance);


	// print result of the evaluation
	if (numGoodMatches > 10 && numInliers > 0.8 * good_matches.size())
	{
	    cout << "The two images have similar content." << endl;
	}
	else if (numGoodMatches > 10 && numInliers <= 0.8 * good_matches.size())
	{
	    cout << "The two images have similar content but are processed by a strong transformation." << endl;
	}
	else
	{
	    cout << "The two images have different content." << endl;
	}


	return exit_code;
}
