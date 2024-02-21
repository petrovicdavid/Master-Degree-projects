#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char **argv) {

    cv::Mat dst, cdst;

    std::string filename = "street_scene.png";

    cv::Mat src = cv::imread(filename, cv::IMREAD_GRAYSCALE);
    cv::Mat csrc = cv::imread(filename);
    if(src.data==NULL || csrc.data==NULL) {
		printf("No image found!\n");
		return 1;
	}

    // edge detection
    double lowT = 288.6;
    cv::Canny(src, dst, lowT, lowT*3, 3);
    //cv::imshow("Canny", dst);

    cv::cvtColor(src, cdst, cv::COLOR_GRAY2BGR);

    // coordinates of the lines
    double m[2];
    double q[2];
    int j=0;
    
    // Standard Hough Line Transform
    std::vector<cv::Vec2f> lines; // will hold the results of the detection
    cv::HoughLines(dst, lines, 1, CV_PI/180, 135); // runs the actual detection
    // Draw the lines
    for(size_t i = 0; i < lines.size(); i++) {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        // filter lines by angle
        double angle = std::atan2((pt2.y-pt1.y), (pt2.x-pt1.x)) * 180 / CV_PI;
        if(angle > -45 && angle < 45) {
            cv::line(cdst, pt1, pt2, cv::Scalar(0,0,255), 3, cv::LINE_AA);
            std::cout<<"("<<pt1.x<<","<<pt1.y<<") - "<<"("<<pt2.x<<","<<pt2.y<<")"<<std::endl;

            m[j] = (pt1.y-pt2.y)/(double)(pt1.x-pt2.x);
            q[j] = (pt1.x*pt2.y-pt2.x*pt1.y)/(double)(pt1.x-pt2.x);
            j++;
        }
    }

    std::cout<<"Intersection: ("<<((q[1]-q[0])/(m[0]-m[1]))<<","<<(m[0]*((q[1]-q[0])/(m[0]-m[1]))+q[0])<<")"<<std::endl;

    // array with 3 points
    cv::Point points[1][3];
    int xp=cvRound((q[1]-q[0])/(m[0]-m[1])), yp=cvRound(m[0]*((q[1]-q[0])/(m[0]-m[1]))+q[0]);
    points[0][0] = cv::Point(xp,yp);
    points[0][1] = cv::Point(cvRound((csrc.rows-q[0])/m[0]), csrc.rows);
    points[0][2] = cv::Point(cvRound((csrc.rows-q[1])/m[1]), csrc.rows);
    const cv::Point* ppt = {points[0]};

    // draw polygon
    cv::fillConvexPoly(csrc, ppt, 3, cv::Scalar(0,0,255), cv::LINE_8);
 
    // Show results
    cv::imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
    cv::imshow("Final image", csrc);
    // Wait and Exit
    cv::waitKey(0);
    
    return 0;
}