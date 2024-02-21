#include "UmboFunct.h"
#ifdef _WIN32
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;
/*namespace fs = std::filesystem;


std::string getProjectPath() {
    std::filesystem::path currentPath = std::filesystem::current_path();
    std::string currentPathStr = currentPath.string();
    //make all / to \ in current path
    std::replace(currentPathStr.begin(), currentPathStr.end(), '\\', '/');
    //delete to currente path last fodler name
    currentPathStr.erase(currentPathStr.find_last_of("/\\"), std::string::npos);
    currentPathStr.erase(currentPathStr.find_last_of("/\\"), std::string::npos);
    currentPathStr.erase(currentPathStr.find_last_of("/\\"), std::string::npos);
    return currentPathStr;
}*/




std::string getProjectPath() {
    char currentPath[FILENAME_MAX];
    if (!GetCurrentDir(currentPath, sizeof(currentPath))) {
        // Error occurred while getting the current working directory
        return "";
    }

    string updatedPath = std::string(currentPath);
    for (int i = 0; i < 3; ++i) {
        size_t lastSeparatorPos = updatedPath.find_last_of("/\\");
        if (lastSeparatorPos == std::string::npos)
            break;
        updatedPath = updatedPath.substr(0, lastSeparatorPos);
    }

    return updatedPath;
}

std::string getTestPath()
{
    string projectPath = getProjectPath();
    if (projectPath != "") {
		return projectPath + "/data/Food_leftover_dataset";
	}
    return projectPath;
}





void SurfDetector(cv::Mat& img1, cv::Mat& img2)
{
    int minHessian = 400;
    cv::Ptr<SURF> detector = SURF::create(minHessian);
    std::vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;
    detector->detectAndCompute(img1, noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(img2, noArray(), keypoints2, descriptors2);

    //create matcher and feature matching
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);

    //filter matches
    const float ratio_thresh = 0.2f; // Nearest neighbor matching ratio userdefined
    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    //draw matches
    Mat img_matches;
    drawMatches(img1, keypoints1, img2, keypoints2, good_matches, img_matches, Scalar::all(-1),
        Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    //show matches
    imshow("Good Matches", img_matches);
    waitKey(0);

}
void SurfDetectorWithCheck(cv::Mat& img1, cv::Mat& img2, std::string path1, std::string path2, float ratio)
{
    int minHessian = 400;
    cv::Ptr<SURF> detector = SURF::create(minHessian);
    std::vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;
    detector->detectAndCompute(img1, noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(img2, noArray(), keypoints2, descriptors2);

    //create matcher and feature matching
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);

    //filter matches
    const float ratio_thresh = ratio; // Nearest neighbor matching ratio userdefined
    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
    if (good_matches.size() > 1) {
        //draw matches
        Mat img_matches;
        drawMatches(img1, keypoints1, img2, keypoints2, good_matches, img_matches, Scalar::all(-1),
            Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        //show matches
        imshow("Good Matches", img_matches);
        waitKey(0);
    }
    else {
        std::cout << "No matches foundbetween img1" << path1 << std::endl;
        std::cout << "And img2 " << path2 << std::endl;
    }
}


Mat segmentationUs(cv::Mat img) {


    cv::Mat mask, bgdModel, fgdModel, result = img.clone();
    //given a pint cerete a Rect object over it	

    cv::Rect r = selectROI(img);
    //select circle area interface from image	

    //cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);	
    //cv::circle(mask, cv::Point(100, 100), 50, cv::Scalar(255), cv::FILLED);
    cv::grabCut(img, mask, r, bgdModel, fgdModel, 1, cv::GC_INIT_WITH_RECT);

    //for each colomn	
    for (int i = 0; i < img.cols; i++)
    {
        for (int j = 0; j < img.rows; j++)
        {
            if ((int)mask.at<uchar>(cv::Point(i, j)) == 0)
            {
                result.at<cv::Vec3b>(cv::Point(i, j))[0] = 0;
                result.at<cv::Vec3b>(cv::Point(i, j))[1] = 0;
                result.at<cv::Vec3b>(cv::Point(i, j))[2] = 0;
            }
            else if ((int)mask.at<uchar>(cv::Point(i, j)) == 1)
            {
                result.at<cv::Vec3b>(cv::Point(i, j))[0] = 255;
                result.at<cv::Vec3b>(cv::Point(i, j))[1] = 0;
                result.at<cv::Vec3b>(cv::Point(i, j))[2] = 0;
            }
            else if ((int)mask.at<uchar>(cv::Point(i, j)) == 1)
            {
                result.at<cv::Vec3b>(cv::Point(i, j))[0] = 255;
                result.at<cv::Vec3b>(cv::Point(i, j))[1] = 0;
                result.at<cv::Vec3b>(cv::Point(i, j))[2] = 0;
            }
            else if ((int)mask.at<uchar>(cv::Point(i, j)) == 2)
            {
                result.at<cv::Vec3b>(cv::Point(i, j))[0] = 0;
                result.at<cv::Vec3b>(cv::Point(i, j))[1] = 255;
                result.at<cv::Vec3b>(cv::Point(i, j))[2] = 0;
            }
            else if ((int)mask.at<uchar>(cv::Point(i, j)) == 3)
            {
                result.at<cv::Vec3b>(cv::Point(i, j))[0] = 0;
                result.at<cv::Vec3b>(cv::Point(i, j))[1] = 0;
                result.at<cv::Vec3b>(cv::Point(i, j))[2] = 255;
            }


        }
    }

    //overalp result and img to result	
    cv::addWeighted(result, 1, img, 0.5, 0, result);
    cv::imshow("Output", result);
    cv::waitKey(0);
    //balaken pixel overlapping area bgdModel and img


    return result;
}