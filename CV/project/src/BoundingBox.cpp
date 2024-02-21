#include "BoundingBox.h"



cv::Mat saturationExtraction(cv::Mat& image)
{
    
    // gamma corrected image
    cv::Mat gc_image;
    cv::Mat lookUpTable(1, 256, CV_8U);
    float gamma = 0.5f;
    uchar* p = lookUpTable.ptr();
    for (int i = 0; i < 256; ++i)
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0,gamma) * 255.0);
    cv::LUT(image, lookUpTable, gc_image);
    //cv::imshow("gamma corrected", gc_image);
    //cv::waitKey(0);
   
    // hsv image
    cv::Mat hsv_image;
    cv::cvtColor(gc_image, hsv_image, cv::COLOR_BGR2HSV);
    // saturation channel mask
    cv::Mat saturation;
    cv::extractChannel(hsv_image, saturation, 1);
    cv::threshold(saturation, saturation, 32, 255, cv::THRESH_BINARY);                   // thresholding

    cv::morphologyEx(saturation, saturation, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)));
    cv::morphologyEx(saturation, saturation, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(55, 55)));

    
    //overlay the saturation to the input image
    double alpha = 0.5; // Weight for the first image
    double beta = 0.5;  // Weight for the second image
    double g = 0.0; // Scalar added to each sum

    cv::Mat blended,saturationNotBinary;
    //convert the saturation to BGR
    cv::cvtColor(saturation, saturationNotBinary, cv::COLOR_GRAY2BGR);
    cv::addWeighted(image, alpha, saturationNotBinary, beta, g, blended);


    //cv::imshow("blended", blended);
    //cv::imshow("saturation", saturation);
    //cv::waitKey(0);
	return saturation;
}

std::vector <cv::Rect> getBoundingBox(cv::Mat segmented)
{
    
    //vector of rectangles
    std::vector <cv::Rect> boundingBoxes;
    cv::Mat labels, stats, centroids;
    //convert the image to binary
    
    
    int numComponents = cv::connectedComponentsWithStats(segmented, labels, stats, centroids);

    
    // Itera sui componenti connessi individuati
    for (int i = 1; i < numComponents; i++) {
        // Estrai le coordinate del bounding box del blob corrente
        int x = stats.at<int>(i, cv::CC_STAT_LEFT);
        int y = stats.at<int>(i, cv::CC_STAT_TOP);
        int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        
        boundingBoxes.push_back(cv::Rect(x, y, width, height));
        
       

        
    }
    return boundingBoxes;
}

void drawBoundingBox(cv::Mat& input, cv::Rect rect)
{
    cv::rectangle(input, rect, cv::Scalar(0, 255, 0), 3);
	//cv::imshow("bounding box", input);
	//cv::waitKey(0);
}



