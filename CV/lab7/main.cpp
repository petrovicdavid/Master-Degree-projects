#include <iostream>
#include <vector>
#include <string.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include "function.h"

using namespace std;
using namespace cv;

const int FoV = 27;
int exit_code = 0;

Mat createPanorama(const vector<Mat>& images);

int main(int argc, char** argv)
{
	vector<String> fn;
	utils::fs::glob("../Images/dolomites", "i*.png", fn);
	vector<Mat> images;
	vector<Mat> images_projected;

	// open all images
	exit_code = open_images(images, fn);
	if(exit_code == 1)
	{
		cerr << "!!! Not able to load at least one image !!!"<<endl;
		return exit_code;
	}

	//project all images
	project_images(images, images_projected, FoV);


	vector<vector<KeyPoint>> images_keypoints;
	vector<Mat> images_descriptors;

	//detect keypoints and descriptors of all images
	orb_detection_computation(images_projected, images_keypoints, images_descriptors);

	vector<vector<DMatch>> vector_good_matches;
  vector<float> translationsX;

	// for each image...
  for(size_t i=0;i<images_projected.size()-1;i++)
  {
    vector<KeyPoint> first_keypoints = images_keypoints[i];
    vector<KeyPoint> second_keypoints = images_keypoints[i+1];;
  	Mat first_descriptors = images_descriptors[i];
    Mat second_descriptors = images_descriptors[i+1];

    vector<DMatch> good_matches;

    // compute and refine the matches
		brute_force_matcher(images_projected[i], images_projected[i+1], first_keypoints, second_keypoints, images_descriptors[i], images_descriptors[i+1], good_matches);

    vector<Point2f> srcPoints, dstPoints;

		// convert in Points
    for(size_t k = 0; k < good_matches.size(); k++)
    {
        srcPoints.push_back(first_keypoints[good_matches[k].queryIdx].pt);
        dstPoints.push_back(second_keypoints[good_matches[k].trainIdx].pt);
    }

		// find the homography matrix
    Mat inliersMask;
    Mat homography = findHomography(srcPoints, dstPoints, RANSAC, 3.0, inliersMask);
    int inliers;
    float translationX;

		// find inliers and calculate the translation in x axis
    for (size_t i = 0; i < inliersMask.rows; i++)
  	{
  	    if (inliersMask.at<uchar>(i, 0) == 1)
  	    {
          inliers++;
          translationX += dstPoints[i].x - srcPoints[i].x;
        }
    }
    cout<<"translation X: "<<translationX/inliers<<endl;
    translationsX.push_back(translationX/inliers);
  }

  return exit_code;
}
