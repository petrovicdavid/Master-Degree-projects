#include "detection.h"
#include "UmboFunct.h"

using namespace std;
using namespace cv;


void writeFile(const vector<int> &prediction)
{
    ofstream file_labels;
    file_labels.open(getProjectPath()+"/data/img/labels.txt");

    if(file_labels.is_open())
    {
        for(size_t i=0;i<prediction.size();i++)
        {
            file_labels<<prediction[i]<<endl;
        }

        file_labels.close();
    }
}


void sendFoodImage(Mat &image, vector<int> &prediction, int numberPlates)
{   
    string n = to_string(numberPlates);
    string path = getProjectPath() + "/data/img/img.jpg";
    imwrite(path, image);
    string mode = to_string(0);
    #ifdef _WIN32
        string command = "python " + getProjectPath() +"/src/detection.py " + mode + " " + n + " " + getProjectPath();
        
    #elif __linux__
        string command = "python3 " + getProjectPath() + "/src/detection.py " + mode + " " + n + " " + getProjectPath();
    #elif __APPLE__
        string command = "python3 " + getProjectPath() + "/src/detection.py " + mode + " " + n + " " + getProjectPath();
    #else
        string command = "python3 " + getProjectPath() + "/src/detection.py " + mode + " " + n + " " + getProjectPath();
    #endif
    
    int exitCode = system(command.c_str());

    if (exitCode == 0)
    {
        ifstream file_labels;
        file_labels.open(getProjectPath() + "/data/img/output.txt");

        string myline;

        if ( file_labels.is_open() ) 
        {
            while ( file_labels ) 
            {
                getline(file_labels, myline);
                int label = atoi(myline.c_str());
                if(label != 0)
                {
                    prediction.push_back(label);
                }
                
            }

        }

        file_labels.close();

    } else {
        cerr << "Error in running Python script" << endl;
    }
}



void sendLeftover(Mat &image, vector<int> &predictionLeftover, int numberPlates)
{
    string n = to_string(numberPlates);
    string path = getProjectPath() + "/data/img/img.jpg";
    imwrite(path, image);

    string mode = to_string(1);
    #ifdef _WIN32
        string command = "python " + getProjectPath() + "/src/detection.py " + mode + " " + n +" "+ getProjectPath();

    #elif __linux__
        string command = "python3 " + getProjectPath() + "/src/detection.py " + mode + " " + n + " " + getProjectPath();
    #elif __APPLE__
        string command = "python3 " + getProjectPath() + "/src/detection.py " + mode + " " + n + " " + getProjectPath();
    #else
        string command = "python3 " + getProjectPath() + "/src/detection.py " + mode + " " + n + " " + getProjectPath();
    #endif
    int exitCode = system(command.c_str());

    if (exitCode == 0)
    {
        ifstream file_labels;
        file_labels.open(getProjectPath() + "/data/img/output.txt");

        string myline;

        if ( file_labels.is_open() ) {
            while ( file_labels ) 
            {
                getline(file_labels, myline);
                int label = atoi(myline.c_str());
                if(label != 0)
                {
                    predictionLeftover.push_back(label);
                }
                
            }

        }

        file_labels.close();

    } else {
        cerr << "Error in running Python script" << endl;
    }
}