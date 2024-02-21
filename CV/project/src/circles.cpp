#include <iostream>
#include <string.h>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "circles.h"


using namespace std;
using namespace cv;



// find the circles in the source image corresponding to the glasses
void find_glass(const Mat &src, vector<Vec3f> &circles)
{
    Mat gray;
    cvtColor(src, gray, COLOR_BGR2GRAY);
    medianBlur(gray, gray, 5);

    HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
        gray.rows/3,
        60, 60, 30, 100
    );
}


// find the circles in the source image corresponding to the plates
void find_plates(const Mat &src, vector<Vec3f> &plates)
{
    Mat gray;
    vector<Vec3f> circles;

    cvtColor(src, gray, COLOR_BGR2GRAY);
    medianBlur(gray, gray, 5);

    HoughCircles(gray, plates, HOUGH_GRADIENT, 1,
        gray.rows/3,
        60, 80, 211, 400
    );
    
}


// find the circle in the source image corresponding to the plate of the salad
void find_salad(const Mat &src, vector<Vec3f> &plate)
{
    Mat gray;

    cvtColor(src, gray, COLOR_BGR2GRAY);
    medianBlur(gray, gray, 5);

    HoughCircles(gray, plate, HOUGH_GRADIENT, 1,
        gray.rows/3,
        50, 80, 120, 260
    );
      
}


// for each element in the input vector, cover the source image with a black circle
void circle2black(Mat &image, const vector<Vec3f> &circles)
{
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Vec3i c = circles[i];
        Point center = Point(c[0], c[1]);
        int radius = c[2];

        // Access pixels within the circle
        for (int x = center.x - radius; x <= center.x + radius; x++)
        {
            for (int y = center.y - radius; y <= center.y + radius; y++)
            {
                if ((x - center.x)*(x - center.x) + (y - center.y)*(y - center.y) <= radius*radius)
                {
                    if( (y >= 0 && y < image.rows) && (x >= 0 && x < image.cols) )
                    {
                        image.at<Vec3b>(y, x) = Vec3b(0, 0, 0);
                    }
                }
            }
        }
    }
}


// for each element in the input vector, create an image with black background and the content of the circle
void circlesToVector(const Mat &src, const vector<Vec3f> &circles, vector<Mat> &images)
{
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Mat result = Mat::zeros(src.size(), src.type());
        Vec3i c = circles[i];
        Point center = Point(c[0], c[1]);
        int radius = c[2] - 50;

        for (int x = center.x - radius; x <= center.x + radius; x++)
        {
            for (int y = center.y - radius; y <= center.y + radius; y++)
            {
                if ((x - center.x)*(x - center.x) + (y - center.y)*(y - center.y) <= radius*radius)
                {
                    if( (y >= 0 && y < src.rows) && (x >= 0 && x < src.cols) )
                    {
                        result.at<Vec3b>(y, x) = src.at<Vec3b>(y, x);
                    }
                }
            }
        }

        images.push_back(result);
    }
}


// copy the content inside the circle in the input vector in a image with black background
void circleToImage(const Mat &src, vector<Vec3f> &salad, Mat &image_salad)
{
        image_salad = Mat::zeros(src.size(), src.type());
        Vec3i c = salad[0];
        Point center = Point(c[0], c[1]);
        int radius = c[2] - 5;
        
        for (int x = center.x - radius; x <= center.x + radius; x++)
        {
            for (int y = center.y - radius; y <= center.y + radius; y++)
            {
                if ((x - center.x)*(x - center.x) + (y - center.y)*(y - center.y) <= radius*radius)
                {
                    if( (y >= 0 && y < src.rows) && (x >= 0 && x < src.cols) )
                    {
                        image_salad.at<Vec3b>(y, x) = src.at<Vec3b>(y, x);
                    }
                }
            }
        }
}


// compute image with black circles instead of plates and glass (used for searching the bread in the image)
void computeBlackImage(const Mat &src, Mat &dst)
{
    vector<Vec3f> circles_plates;
    vector<Vec3f> circle_salad;
    vector<Vec3f> circles_glass;

    find_plates(src, circles_plates);
    dst = src.clone();
    circle2black(dst, circles_plates);
    find_salad(dst, circle_salad);
    circle2black(dst, circle_salad);
    find_glass(dst, circles_glass);
    circle2black(dst, circles_glass);

}


// compute the images of plates with black background and push them into a vector
// compute the image of the plate of the salad with black background and save it
void computeImages(const Mat &src, Mat &image_salad, vector<Mat> &images_plate)
{
    vector<Vec3f> circles_plates;
    vector<Vec3f> circle_salad;

    find_plates(src, circles_plates);
    Mat tmp = src.clone();
    circle2black(tmp, circles_plates);
    find_salad(tmp, circle_salad);
    
    if(circle_salad.size() > 0)
    {
        circleToImage(src, circle_salad, image_salad);
    }

    if(circles_plates.size() > 0)
    {
        circlesToVector(src, circles_plates, images_plate);
    }
    
}