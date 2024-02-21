#ifndef UMBO_F
#define UMBO_F



#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>
#include <iostream>
#include <vector>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <map>



cv::Mat segmentationUs(cv::Mat image);
std::string getProjectPath();
std::string getTestPath();
void SurfDetector(cv::Mat& img1, cv::Mat& img2);
void SurfDetectorWithCheck(cv::Mat& img1, cv::Mat& img2, std::string path1, std::string path2, float ratio);
/*std::vector<std::string> getfoldersname(std::string path);
std::vector<std::string> getfilenames(std::string path);*/


#endif // !UMBO_F