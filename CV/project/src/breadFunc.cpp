#include "breadFunc.h"

using namespace std;
using namespace cv;

typedef std::vector<cv::Point> Contour;
typedef std::vector<Contour> Contours;

bool isMostlyWhite(const cv::Mat& grayImage, float threshold = 0.9f) {
    int whiteCount = 0;
    int totalPixels = grayImage.rows * grayImage.cols;

    for (int y = 0; y < grayImage.rows; ++y) {
        for (int x = 0; x < grayImage.cols; ++x) {
            if (grayImage.at<uchar>(y, x) == 255) {
                whiteCount++;
            }
        }
    }
    float whiteRatio = static_cast<float>(whiteCount) / totalPixels;
    return whiteRatio >= threshold;
}
bool isMostlyWhite(const cv::Mat& grayImage, const cv::Rect& rect, double threshold = 0.9f)
{
    // Extract the region of interest (ROI)
    cv::Mat roi = grayImage(rect);

    // Convert ROI to grayscale

    // Calculate the average pixel value
    cv::Scalar avgPixelValue = cv::mean(roi);

    // Check if the average pixel value is above the threshold
    double average = avgPixelValue[0] / 255.00;
    return (average > threshold);
}

Mat gammaCorrection(const Mat& inputImage, float gamma) {
    cv::Mat gc_image;
    cv::Mat lookUpTable(1, 256, CV_8U);
    //float gamma = 0.5f;
    uchar* p = lookUpTable.ptr();
    for (int i = 0; i < 256; ++i)
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    cv::LUT(inputImage, lookUpTable, gc_image);
    //cv::imshow("gamma corrected", gc_image);
    //cv::waitKey(0);

    return gc_image;


}
void imfill(Mat& image)
{
    // Floodfill from point (0, 0)
    Mat im_floodfill = image.clone();
    floodFill(im_floodfill, cv::Point(0, 0), Scalar(255));

    // Invert floodfilled image
    Mat im_floodfill_inv;
    bitwise_not(im_floodfill, im_floodfill_inv);

    // Combine the two images to get the foreground.
    image = (image | im_floodfill_inv);
}


Mat motrphologyOpeClose(Mat input, int SizeOpen, int SizeClose) {

    Mat oputputMorphIstance;
    cv::morphologyEx(input, oputputMorphIstance, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(SizeOpen, SizeOpen)));
    cv::morphologyEx(oputputMorphIstance, oputputMorphIstance, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(SizeClose, SizeClose)));
    return oputputMorphIstance;
}

cv::Mat filterAreasLower(cv::Mat& img, int threshold)
{
    // Find all contours
    //ouput as img blak
    cv::Mat output = cv::Mat::zeros(img.size(), CV_8UC1);
    Contours contours;

    findContours(img.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); i++)
    {
        // Calculate contour area
        double area = contourArea(contours[i]);

        // Remove small objects by drawing the contour with black color
        //if (area > 0 && area <= treshold)
        if (area >= threshold) {
            drawContours(output, contours, i, 255, -1);
        }
    }
    return output;

}

cv::Mat filterAreasUpper(cv::Mat& img, int threshold)
{
    cv::Mat output = cv::Mat::zeros(img.size(), CV_8UC1);
    Contours contours;
    findContours(img.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); i++)
    {
        // Calculate contour area
        double area = contourArea(contours[i]);

        // Remove big objects by drawing the contour with black color
        //if (area <= treshold)
        if (area < threshold) {

            drawContours(output, contours, i, 255, -1);
        }
    }
    return output;
}


Mat initialbreadMask(const Mat& inputImage) {
    //initialization constants
    const int k = 70;
    const int b = 17;
    const int so = 1;
    const int sc = 9;
    const int sc2 = 23;
    const int AREA_CLEAR_MIN = 8000;
    const int AREA_CLEAR_MAX = 80000;

    Mat in= inputImage.clone();

    //get saturation mask
    Mat saturation = saturationExtraction(in);

    //convert input to gray
    Mat gray;
    cv::cvtColor(inputImage, gray, cv::COLOR_BGR2GRAY);
    //gama correction on gray
    gray = gammaCorrection(gray, 0.65f);


    //set niBlack to zero
    Mat niBlack = Mat::zeros(inputImage.size(), CV_8UC1);

    //apply niBlack
    cv::ximgproc::niBlackThreshold(gray, niBlack, 255, cv::THRESH_BINARY, b + b % 2 + 1, double(k) / 100.00);
    //fill holes
    imfill(niBlack);


    //check if is too texured
    if (isMostlyWhite(niBlack)) // too texured
    {
        cout << "too texured... fixing" << endl;


        cv::ximgproc::niBlackThreshold(gray, niBlack, 255, cv::THRESH_BINARY, 3, 0.4f);
        imfill(niBlack);
        //apply morphology
        niBlack = motrphologyOpeClose(niBlack, 2, 12);

        if (isMostlyWhite(niBlack)) // too texured again
        {
            cout << "Mostly white again, no bread can be detected" << endl;
            niBlack = Mat::zeros(inputImage.size(), CV_8UC1);
        }
    }
    else {
        //apply morphology
        niBlack = motrphologyOpeClose(niBlack, so, sc);
    }

    //intersection between niBlack and saturation
    Mat intersection;
    cv::bitwise_and(niBlack, saturation, intersection);



    //clean  too little  areas
    Mat cleandArea = filterAreasLower(intersection, AREA_CLEAR_MIN);

    //convert to bgr

    //reapply morphology   
    cv::dilate(cleandArea, cleandArea, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(sc2, sc2)));


    cv::morphologyEx(cleandArea, cleandArea, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20)));


    cleandArea = filterAreasUpper(cleandArea, AREA_CLEAR_MAX);
    vector<vector<Point>> contours;
    cv::findContours(cleandArea, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    int numBlobs = contours.size();
    if (numBlobs > 1) {
        cout << "too many possible bread, no real bread detected" << endl;
        cleandArea = Mat::zeros(inputImage.size(), CV_8UC1);
    }
    // TODO: add a possible check for color matching

    // Filter out unwanted contours (optional)
    // You can apply any filtering criteria based on your requirements

    // Calculate the number of blobs



    return cleandArea;


}
Mat breadMask(const Mat& inputImage) {


    Mat initialMask = initialbreadMask(inputImage);
    vector<Rect> boundingBoxes;
    boundingBoxes = getBoundingBox(initialMask);
    cv::Mat gcMask = Mat::zeros(inputImage.size(), CV_8UC1);

    //for each bounding box check if is bread

    for (int i = 0; i < boundingBoxes.size(); ++i) {

        Mat maskRectMask = Mat::zeros(inputImage.size(), CV_8UC1);
        //set maskRectMask to  GC_PR_BGD in the bounding box
        maskRectMask(boundingBoxes[0]).setTo(cv::GC_PR_BGD);


        cv::threshold(initialMask, initialMask, 0, 1, cv::THRESH_BINARY);

        cv::addWeighted(initialMask, 1, maskRectMask, 1, 0, gcMask);

        Mat bgdModel, fgdModel;

        Mat gcInput = inputImage.clone();

        cv::grabCut(gcInput, gcMask, Rect(), bgdModel, fgdModel, 5, cv::GC_INIT_WITH_MASK);

    }
    Mat1b mask_fgpf = (gcMask == GC_FGD) | (gcMask == GC_PR_FGD);

    //binarize mask
    cv::threshold(mask_fgpf, mask_fgpf, 0, 255, cv::THRESH_BINARY);

    //chek if multiple blobs
    vector<vector<Point>> contours;
    findContours(mask_fgpf, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    int numBlobs = contours.size();
    //take the biggest blob
    if (numBlobs > 1) {
        double maxArea = 0;
        cout << "too many blobs, selecting the bigger as bread..." << endl;
        for (int i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);

            maxArea = max(area, maxArea);
        }
        mask_fgpf = filterAreasLower(mask_fgpf, static_cast<int>(floor(maxArea)));
        
    }


    vector<Rect> boundingBoxBread;
    boundingBoxBread = getBoundingBox(mask_fgpf);

    //check if in the box is mostly white
    if (boundingBoxBread.size() != 0 && isMostlyWhite(mask_fgpf, boundingBoxBread[0])) {
        cout << "the bread found was FP" << endl;
        mask_fgpf = Mat::zeros(inputImage.size(), CV_8UC1);
    }

    cv::threshold(mask_fgpf, mask_fgpf, 0, 13, cv::THRESH_BINARY);

    return mask_fgpf;
}


Rect breadBoundingBox(const Mat inputImage) {
    Mat mask = breadMask(inputImage);
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

