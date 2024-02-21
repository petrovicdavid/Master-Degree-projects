#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int exit_code = 0;

int main(int argc, char** argv)
{
	Mat image_grayscale;
	Mat I = imread("../robocup.jpg");
	if (I.empty())
	{
		cout << "!!! Failed imread(): image not found" << endl;
		exit_code = 1;
	}

	namedWindow( "Display window");
	imshow( "Display window", I );
	waitKey(0);

	return exit_code;
}
