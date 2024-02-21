#ifndef __FUNCTION_H__
#define __FUNCTION_H__

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

int open_images(string directory, string pattern, vector<Mat> &images);

void get_point(vector<vector<Point3f>> &objpoints, vector<Point3f> &objp);

int find_corner(const vector<Mat> &images, Mat &gray, vector<vector<Point2f>> &detected_corners, vector<vector<Point3f>> &objpoints, vector<Point3f> &objp);

double calculate_mean_reprojection_error(const vector<Mat> &images, const vector<vector<Point2f>> &detected_corners, const vector<vector<Point3f>> &objpoints, Mat &cameraMatrix, Mat &distCoeffs, vector<Mat> &R, vector<Mat> &T);

void compare_distorted_undistorted(const vector<Mat> &images, const Mat &cameraMatrix, const Mat &distCoeffs, vector<Mat> &comparisons);


#endif
