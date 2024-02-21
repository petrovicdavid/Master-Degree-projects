#include <sstream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"
#include <map>
#include <opencv2/core/utils/filesystem.hpp>
#include <fstream>
#include "main.h"




using namespace std;
using namespace cv;


void SaladAndBread(Mat& image_salad,vector<Mat> &masks,const Mat src,string boxesPath,string masksPath, string fileName) {
    cout << "segmenting possible salad" << endl;
    //check if image_salad is empty
    if (!image_salad.empty()) {
        masks.push_back(saladMask(image_salad));
    }
    else {
        cout << "no salad" << endl;
    }
    cout << "segmenting salad ended" << endl;



    cout << "segmenting possible bred" << endl;
    Mat black;
    computeBlackImage(src, black);
    Mat bread = breadMask(black);

    if (!bread.empty())
        masks.push_back(bread);
    else {
        cout << "no bred" << endl;
    }
    cout << "segmenting bred ended" << endl;

    //combine masks
    Mat mask = combineMasks(masks);

    

    string filePath;
    //write the file on the disk, Proejct path
    if (fileName == "food_image") {
        filePath = masksPath + fileName + "_mask.png";
    }
    else{
        filePath = masksPath + fileName + ".png";
    }
    imwrite(filePath, mask);



    //extract boxes
    std::map<int, cv::Rect> boxesMap;
    boxesMap = extractBoxes(mask);

    cout << "number of boxes " << boxesMap.size() << endl;
    //disply all boxesMap
    for (auto it = boxesMap.begin(); it != boxesMap.end(); it++) {
        cout << "index " << it->first << " box " << it->second << endl;
        //rectangle(src, it->second, Scalar(0, 255, 0), 2);
    }

    filePath = boxesPath + fileName + "_bounding_box.txt";
    std::fstream file(filePath, std::ios::out);
    if (file.is_open()) {
        for (auto it = boxesMap.begin(); it != boxesMap.end(); it++) {
            file << "ID: " << it->first << "; [" << it->second.x<<", " << it->second.y << ", " << it->second.width << ", " << it->second.height << "]" << endl;
            //rectangle(src, it->second, Scalar(0, 255, 0), 2);
        }
        file.close();
        std::cout << "File created and written successfully." << std::endl;
    }
    else {
        std::cerr << "Error opening the file." << std::endl;
    }

}

int main()
{
    cout<< "start..." << endl;
    //get all the folders
   vector<string> folders = {"tray1", "tray2","tray3","tray4","tray5","tray6","tray7","tray8"};
    vector<string> fileNames = { "leftover1","leftover2","leftover3" };
    
    //for each folder
    for (int m = 0; m < folders.size(); m++) {
				string folderTrayPath = getTestPath() + "/" + folders[m]+"/";
                string path = folderTrayPath + "food_image.jpg";//food_image

                Mat src = imread(path);
                // Check if image is loaded fine
                if (src.empty()) {
                    printf(" Error opening image\n");
                    return EXIT_FAILURE;
                }

                Mat image_salad;
                vector<Mat> images_plate;
                vector<Mat> masks;
                computeImages(src, image_salad, images_plate);

                vector<int> prediction;//one for each tray

                utils::fs::createDirectory(getProjectPath() + "/data/img/");//create img folder for interaction with python
                string folderPath = getProjectPath() + "/data/result/";
                utils::fs::createDirectory(folderPath);//create result folder
                folderPath = getProjectPath() + "/data/result/"+ folders[m]+"/";
                utils::fs::createDirectory(folderPath);//create result folder for each tray

                std::cout << "created dir in " << folderPath << endl;



                //segmentation of the foodImage
                for (int k = 0; k < images_plate.size(); k++) {
                    vector<int> labels;
                    std::cout << "running python ..." << endl;
                    sendFoodImage(images_plate[k], labels, images_plate.size());
                    //output labels
                    std::cout << "labels by python: ";
                    for (int i = 0; i < labels.size(); i++) {
                        std::cout << labels[i] << " ";
                        prediction.push_back(labels[i]);
                    }
                    std::cout << "---" << endl;
                    //imshow("palte",images_plate[i]);
                    //waitKey(0);
                    //segment food
                    if (labels.size() > 0 && labels[0] == -1)
                        labels.clear();

                    std::cout << "segmenting food" << endl;
                    Mat mask = segmetFoodColorLabel(labels, images_plate[k]);

                    std::cout << "ended food" << endl;

                    masks.push_back(mask);
                }

                string boxPath = folderPath + "/bounding_boxes/";
                utils::fs::createDirectory(boxPath);

                std::cout << "created dir in " << folderPath << endl;
                
                string masksPath = folderPath + "/masks/";
                utils::fs::createDirectory(masksPath);

                std::cout << "created dir in " << folderPath << endl;

                //segmentation of the salad and bread and write the mask and boxes
                SaladAndBread(image_salad, masks, src, boxPath,masksPath, "food_image");


                //for each leftover
                for (int j = 0; j < fileNames.size(); j++) {
					string pathLeftOver = folderTrayPath + fileNames[j]+".jpg";//lefOver
                    Mat srcLeftOver = imread(pathLeftOver);

                    if (srcLeftOver.empty()) {
                        printf(" Error opening image\n");
                        return EXIT_FAILURE;
                    }

                    vector<Mat> images_plate_left, leftOverMasks;
                    Mat image_salad_left;
                    computeImages(srcLeftOver, image_salad_left, images_plate_left);//compute images of paltes

                    int numberPlates = images_plate_left.size();
                    writeFile(prediction);//write the file on the disk, Proejct path
                    bool flag = false;
                    vector<int> lastPred;
                    for (size_t i = 0; i < numberPlates; i++)
                    {
                        vector <int> predictionLeftover;

                        cout << "running python for leftOver Plate ..." << endl;
                        sendLeftover(images_plate_left[i], predictionLeftover, numberPlates);



                        //output labels
                        cout << "labels by python: ";
                        for (int i = 0; i < predictionLeftover.size(); i++) {
                            cout << predictionLeftover[i] << " ";
                            //prediction.push_back(predictionLeftover[i]);
                        }
                        cout << "----" << endl;
                        lastPred = predictionLeftover;
                        if (flag)
                            predictionLeftover.clear();
                        if (i == 0 && predictionLeftover.size() > 0 && predictionLeftover[0] == -1) {
                            predictionLeftover.clear();
                            flag = true;
                        }
                        if (i != 0 && predictionLeftover.size() > 0 && predictionLeftover[0] == -1) {
                            predictionLeftover.clear();
                            predictionLeftover = prediction;
                            cout<<"prediction updated "<<endl;
                            //remove the value from the vector
                            for ( auto valueToDelete : lastPred ){
                                predictionLeftover.erase(std::remove(predictionLeftover.begin(), predictionLeftover.end(), valueToDelete), predictionLeftover.end());
                            }
                        }
                        
                        //imshow("palte",images_plate[i]);
                        //waitKey(0);
                        //segment food
                        cout << "segmenting food" << endl;
                        Mat mask = segmetFoodColorLabel(predictionLeftover, images_plate_left[i]);
                        cout << "ended food" << endl;
                        leftOverMasks.push_back(mask);
                    }
                    //segment salad and bred
                    SaladAndBread(image_salad_left, leftOverMasks, srcLeftOver, boxPath, masksPath, fileNames[j]);

        }
    }
 
    
    cout << "done with data manipulation" << endl;

    std::cout << "mAP: " << mAP() << std::endl;
    mIoU();
    leftover();

    cout << "done";

    return 0;
}



