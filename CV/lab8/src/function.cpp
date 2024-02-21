#include "function.h"
#include <iostream>
#include <vector>
#include <string.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/utils/filesystem.hpp>

using namespace std;
using namespace cv;

// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{6,5};

// function for open all images in the directory that has a given pattern
int open_images(string directory, string pattern, vector<Mat> &images)
{
  vector<string> fn;
  int exit_code = 0;

  // get the path of all images that is in the directory and has the pattern
  utils::fs::glob(directory, pattern, fn);

  // open images and load them into a vector
	for(int k=0;k<fn.size();k++)
	{
		images.push_back(imread(fn[k]));
		if(images[k].empty())
		{
			exit_code = 1;
      break;
		}
	}

  return exit_code;
}


// function for getting all the 3D points
void get_point(vector<vector<Point3f>> &objpoints, vector<Point3f> &objp)
{
  for(int i{0}; i<CHECKERBOARD[1]; i++)
  {
    for(int j{0}; j<CHECKERBOARD[0]; j++)
      objp.push_back(Point3f(j,i,0));
  }
}


// function for finding the corners in the images
int find_corner(const vector<Mat> &images, Mat &gray, vector<vector<Point2f>> &detected_corners, vector<vector<Point3f>> &objpoints, vector<Point3f> &objp)
{
  int exit_code = 0;
  // for each image, convert it into a grayscale one, call findChessboardCorners function, check the result and load the corners into a vector
  for(size_t k=0; k<images.size();k++)
	{
		cvtColor(images[k],gray, COLOR_BGR2GRAY);
		vector<Point2f> corners;
		bool ris = findChessboardCorners(gray, Size(CHECKERBOARD[0], CHECKERBOARD[1]), corners);
		if(ris)
		{
			detected_corners.push_back(corners);
			objpoints.push_back(objp);
		}else{
			exit_code = 2;
			break;
		}
	}

  return exit_code;
}


// function for calculating the mean reprojection error
double calculate_mean_reprojection_error(const vector<Mat> &images, const vector<vector<Point2f>> &detected_corners, const vector<vector<Point3f>> &objpoints, Mat &cameraMatrix, Mat &distCoeffs, vector<Mat> &R, vector<Mat> &T)
{
  int totalPoints = 0;
	double totalError = 0;
  vector<Point2f> projectedPoints;

  // for each image, call projectPoints function and calculate the error, then add the latter to the total error. Also, count the number of detected corners in all images
  for(size_t i=0; i<images.size();++i)
	{
		projectPoints(objpoints[0], R[i], T[i], cameraMatrix, distCoeffs, projectedPoints);
		double error = norm(detected_corners[i], projectedPoints, NORM_L2) / detected_corners[i].size();
    totalError += error;
    totalPoints += detected_corners[i].size();
	}

  // return the total error divided into the total number of corners (the mean reprojection error)
	return totalError / totalPoints;
}


// function for creating the final images which are the comparisons of distorted and undistorted images
void compare_distorted_undistorted(const vector<Mat> &images, const Mat &cameraMatrix, const Mat &distCoeffs, vector<Mat> &comparisons)
{
  Mat rectificationMap1, rectificationMap2;

  // call initUndistortRectifyMap for getting the rectification maps
  initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, images[0].size(), CV_32FC1, rectificationMap1, rectificationMap2);

  // for each distorted image, call remap function for getting the undistorted image and create the final image which is the concatenation of the two
  for(size_t i=0;i<images.size();i++)
  {
    Mat distorted_image = images[i];
    Mat undistorted_image;

    // apply the maps to the distorted image
    remap(distorted_image, undistorted_image, rectificationMap1, rectificationMap2, INTER_LINEAR);

    // resize the images for visualize them in the screen after having concatenated. I've used the original dimensions divided by 3 for having the same proportion of the originals
    Size resizedSize1(distorted_image.cols/3, distorted_image.rows/3);
    resize(distorted_image, distorted_image, resizedSize1);
    Size resizedSize2(undistorted_image.cols/3, undistorted_image.rows/3);
    resize(undistorted_image, undistorted_image, resizedSize2);

    //concatenate the two images into one image
    Mat comparison;
    hconcat(distorted_image, undistorted_image, comparison);


    // load the final image into a vector
    comparisons.push_back(comparison);
  }
}
