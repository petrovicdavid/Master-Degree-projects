#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char **argv) {

    std::string filename = "street_scene.png";

    cv::Mat src = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    cv::Mat csrc = cv::imread(filename);
    if(src.data==NULL || csrc.data==NULL) {
		printf("No image found!\n");
		return 1;
	}

    std::vector<cv::Vec3f> circles;

    cv::HoughCircles(src, circles, cv::HOUGH_GRADIENT_ALT, 1, 
                        src.rows/16, 
                        100, 
                        0.8,
                        0, 50 );
    for(int i = 0; i < circles.size(); i++) {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // draw the circle outline, filled
        circle(csrc, center, radius, cv::Scalar(0,255,0), -1, 8, 0 );
    }
    cv::imshow("Circles", csrc);
    cv::waitKey(0);
    return 0;
}