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
    float ref_b = 0.0, ref_g = 0.0, ref_r = 0.0;
    int T = 70; //threshold

    //to prevent segfaults for looking over the image boundaries
    if(y + NEIGHBORHOOD/2 > image.rows || x + NEIGHBORHOOD/2 > image.cols){
      return;
    }

    for(int i=-k;i<=k;i++){
      for(int j=-k;j<=k;j++){
        sum_b += image.at<Vec3b>(y+i,x+j)[0];
        sum_g += image.at<Vec3b>(y+i,x+j)[1];
        sum_r += image.at<Vec3b>(y+i,x+j)[2];
      }
    }

		int tot = NEIGHBORHOOD*NEIGHBORHOOD;
    ref_b = (float) sum_b/tot;
    ref_g = (float) sum_g/tot;
    ref_r = (float) sum_r/tot;

    cout<<"Mean blue: "<<ref_b<<" Mean green: "<<ref_g<<" Mean red: "<<ref_r<<endl;

		Mat mask(image.size(), CV_8UC1, Scalar(0));

    for(int i=0;i<image.rows;i++){
      for(int j=0;j<image.cols;j++){
				Vec3b pixel = image.at<Vec3b>(i,j);
        int b = pixel[0];
        int g = pixel[1];
        int r = pixel[2];

				float dist_b = abs(ref_b-b);
				float dist_g = abs(ref_g-g);
				float dist_r = abs(ref_r-r);

        if(dist_b <= T && dist_g <= T && dist_r <= T){
					mask.at<uchar>(i, j) = 255; // set white pixel
        }
      }
    }

    namedWindow("Display mask");
  	imshow("Display mask", mask);

  }
}
