#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define NEIGHBORHOOD 9

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
    int k = NEIGHBORHOOD/2;
    int sum_b = 0, sum_g = 0, sum_r = 0;
    float mean_b = 0.0, mean_g = 0.0, mean_r = 0.0;
		for(int i=-k;i<=k;i++){
      for(int j=-k;j<=k;j++){
        sum_b += image.at<Vec3b>(y+i,x+j)[0];
        sum_g += image.at<Vec3b>(y+i,x+j)[1];
        sum_r += image.at<Vec3b>(y+i,x+j)[2];
      }
    }


    mean_b = (float) sum_b/(NEIGHBORHOOD * NEIGHBORHOOD);
    mean_g = (float) sum_g/(NEIGHBORHOOD * NEIGHBORHOOD);
    mean_r = (float) sum_r/(NEIGHBORHOOD * NEIGHBORHOOD);

    cout<<"Mean blue: "<<mean_b<<" Mean green: "<<mean_g<<" Mean red: "<<mean_r<<endl;
  }
}
