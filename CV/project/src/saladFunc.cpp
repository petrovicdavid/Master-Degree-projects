
#include "saladFunc.h"

using namespace std;
using namespace cv;


void takeLarger(Mat& input) {
    vector<vector<Point>> contours;
    findContours(input, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    int numBlobs = contours.size();
    //take the biggest blob
    if (numBlobs > 1) {
        double maxArea = 0;
        cout << "too many blobs, selecting the bigger as bread..." << endl;
        for (int i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);

            maxArea = max(area, maxArea);
        }
        input = filterAreasLower(input, static_cast<int>(floor(maxArea)));

    }
}
Mat saladMask(const Mat inputSaladDish) {
    Mat gamma = gammaCorrection(inputSaladDish, 0.5f);
    Mat hsv;
    cvtColor(gamma, hsv, COLOR_BGR2HSV);

    //split hsv
    vector<Mat> hsv_channels;
    split(hsv, hsv_channels);
    //equalize s
    equalizeHist(hsv_channels[1], hsv_channels[1]);
    Mat maskS;
    //threshold s
    cv::threshold(hsv_channels[1], maskS, 211, 255, cv::THRESH_BINARY);
    //fill holes
    imfill(maskS);
    //close
    cv::morphologyEx(maskS, maskS, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(73, 73)));
    //take larger blob
    takeLarger(maskS);
    cv::threshold(maskS, maskS, 0, 12, cv::THRESH_BINARY);
    return maskS;




}
Rect saladBoundingBox(const Mat inputImage) {
    Mat mask = saladMask(inputImage);
    vector<Rect> boundingBoxes;
    boundingBoxes = getBoundingBox(mask);
    Mat output = inputImage.clone();
    if (boundingBoxes.size() == 0) {
        return Rect();
    }
    if (boundingBoxes.size() == 1)
        return boundingBoxes[0];
    else {
        cout << "error, more than one box found";
    }
    return Rect();
}



