#include <iostream>
#include <string.h>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "circles.h"
#include "UmboFunct.h"
#include "BoundingBox.h"
#include "func.h"
#include "breadFunc.h"

using namespace std;
using namespace cv;

int main()
{

   
        string path = getTestPath() + "/3tray2/food_image.jpg";
        // Loads an image
        Mat src = imread(path);
        // Check if image is loaded fine
        if(src.empty()){
            printf(" Error opening image\n");
            return EXIT_FAILURE;
        }

        vector<Vec3f> circles_plates;
        vector<Vec3f> salad;
        find_plates(src, salad, circles_plates);

    
        vector<Mat> images_plate;
        circlesToVector(src, circles_plates, images_plate);

        

        Mat image_salad;

        if(salad.size() > 0)
        {
            
            circleToImage(src, salad, image_salad);
            imshow("p", image_salad);
            waitKey(0);
            
        }

        /*for (size_t j = 0; j<images_plate.size(); j++)
        {   
            //int cont = i+j;
            //string name = to_string(cont);
            //string folder = "../dataset/";
            //string image = folder+name+jpg;
            //imwrite(image, circle_images[j]);
            imshow("p", images_plate[j]);
            waitKey(0);
        }*/

    

    



    return 0;
}
