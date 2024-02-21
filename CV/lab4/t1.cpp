#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

cv::Mat src, src_gr;
cv::Mat dst, detected_edges;

int lowT = 0;
const int max_lowT = 100;
const int max_ksize = 2;
int ratio = 3;
int k_size = 0;
const char* win_name = "Edge Map";

static void CannyThreshold(int, void *) {
    cv::blur(src_gr, detected_edges, cv::Size(3,3));
    /**
     * three steps of the trackbar for kernel size of Canny:
     * 1) kernel 3x3
     * 2) kernel 5x5
     * 3) kernel 7x7
     */
    cv::Canny(detected_edges, detected_edges, lowT, lowT*ratio, (3*(k_size+1)-k_size));
    dst = cv::Scalar::all(0);
    src.copyTo(dst, detected_edges);
    cv::imshow(win_name, dst);
}

int main(int argc, char **argv) {
	
	src = cv::imread("street_scene.png");
	cv::imshow("Source img", src);
	cv::waitKey(0);

    dst.create(src.size(), src.type());
    cv::cvtColor(src, src_gr, cv::COLOR_BGR2GRAY);
    cv::namedWindow(win_name, cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("Min threshold:", win_name, &lowT, max_lowT, CannyThreshold);
    cv::createTrackbar("Kernel size [steps]:", win_name, &k_size, max_ksize, CannyThreshold);
    CannyThreshold(0,0);
    cv::waitKey(0);
	return 0;
}