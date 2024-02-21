#include <iostream>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include "function.h"


using namespace std;
using namespace cv;


int exit_code = 0;


int main(int argc, char** argv)
{
	vector<Mat> images;

	// check command line arguments and try to open images
	if(argc < 3)
	{
		cerr << "Error! Provide directory and pattern: <directory> <pattern>" << endl;
		exit_code = 1;
		return exit_code;
	}else{
		exit_code = open_images(argv[1], argv[2], images);

		if(exit_code != 0)
		{
			cerr << "!!! Not able to load images !!!" << endl;
			return exit_code;
		}
	}


	// vector to store vectors of 3D points for each checkerboard image
  vector<vector<Point3f>> objpoints;

	// defining world coordinates for 3D points
	vector<Point3f> objp;

	get_point(objpoints, objp);


	Mat gray;
	vector<vector<Point2f>> detected_corners;

	// find corners in the images
	exit_code = find_corner(images, gray, detected_corners, objpoints, objp);

	// if it is different from 0, an error occured
	if(exit_code != 0)
	{
		cerr << "Error in detect corners" << endl;
		return exit_code;
	}else{
		cout << "Corners found" <<  endl;
	}


	Mat cameraMatrix,distCoeffs;
	vector<Mat> R,T;

	calibrateCamera(objpoints, detected_corners, Size(gray.rows,gray.cols), cameraMatrix, distCoeffs, R, T);

	cout << "Camera calibrated" <<  endl;


	// calculate the mean reprojection error
	double meanError = calculate_mean_reprojection_error(images, detected_corners, objpoints, cameraMatrix, distCoeffs, R, T);

	cout << "Mean reprojection error: " << meanError << endl;

	// vector of images in which each image is a comparison between a distorted and undistorted image
	vector<Mat> comparisons;

	// compute the comparisons
	compare_distorted_undistorted(images, cameraMatrix, distCoeffs, comparisons);


	// show the final images
	for(size_t k=0;k<comparisons.size();k++)
	{
		imshow("Distorted vs Undistorted", comparisons[k]);
    waitKey(0);
	}


	return exit_code;
}
