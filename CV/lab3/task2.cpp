#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;
using namespace cv;

int exit_code = 0;

void onMouse(int event, int x, int y, int f, void* userdata);

int main(int argc, char** argv)
{
	Mat I = imread("../robocup.jpg");
	if (I.empty())
	{
		cout << "!!! Failed imread(): image not found" << endl;
		exit_code = 1;
	}

  //resize(I, I, Size(I.cols / 2.0, I.rows / 2.0));
	namedWindow( "Display window");
	imshow( "Display window", I);

  setMouseCallback("Display window", onMouse, (void*) &I);

  waitKey(0);

	return exit_code;
}


void onMouse(int event, int x, int y, int f, void* userdata){
  //if the left buttton is pressed
  if(event == EVENT_LBUTTONDOWN)
  {
    Mat image = *(Mat*) userdata;
    int blue = image.at<Vec3b>(y,x)[0];
    int green = image.at<Vec3b>(y,x)[1];
    int red = image.at<Vec3b>(y,x)[2];
    cout<<"Blue: "<<blue<<" Green: "<<green<<" Red: "<<red<<endl;
  }
}
