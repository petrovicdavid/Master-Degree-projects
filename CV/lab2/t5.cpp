#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"

int main(int argc, char **argv)
{
	cv::Mat img = cv::imread("image.jpg");
	if(img.data==NULL) {
		printf("No image found!\n");
		return 1;
	}
	cv::namedWindow("Example 1");
	cv::imshow("Example 1", img);
	cv::waitKey(0);
	
    // greyscale image

	cv::Mat g_img;
	cv::cvtColor(img, g_img, cv::COLOR_BGR2GRAY);
	
	cv::namedWindow("Example 2");
	cv::imshow("Example 2", g_img);
	cv::waitKey(0);

    cv::Mat eq_img;
    cv::equalizeHist(g_img, eq_img);

    cv::namedWindow("Equalized image");
    cv::imshow("Equalized image", eq_img);
    cv::waitKey(0);

    cv::imwrite("image_eq.jpg", eq_img);

    // histogram
    int histSize = 256;
    float range[] = { 0, 256 }; //the upper boundary is exclusive
    const float* histRange = { range };
    bool uniform = true, accumulate = false;
    cv::Mat hist;
    cv::calcHist( &eq_img, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
    int hist_w = 512, hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );
    cv::Mat histImage( hist_h, hist_w, CV_8UC1, cv::Scalar(0) );
    normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
    for( int i = 1; i < histSize; i++ )
    {
        cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ),
              cv::Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
              cv::Scalar(255), 2, 8, 0  );
    }

    cv::namedWindow("Equalized hist");
    cv::imshow("Equalized hist", histImage);

    

    cv::waitKey(0);
    cv::imwrite("image_eqhist.jpg", histImage);

	return 0;
}

