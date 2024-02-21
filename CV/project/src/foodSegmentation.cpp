#include "foodSegmentation.h"


using namespace std;
using namespace cv;


bool customSort(int a, int b) {
    std::vector<int> order = { 1, 2, 3, 4, 5, 7, 8, 6, 10, 9, 11 };

    auto it_a = std::find(order.begin(), order.end(), a);
    auto it_b = std::find(order.begin(), order.end(), b);

    return it_a < it_b;
}
Mat postProcessingmask(Mat mask, int scaleClose, int threshold, int scaleDilate, int scaleCloseLast) {
    Mat output = mask.clone();

    medianBlur(mask, output, 5);
    //closing
    cv::morphologyEx(output, output, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(scaleClose, scaleClose)));

    //dilate
    output = filterAreasLower(output, threshold);
    cv::dilate(output, output, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(scaleDilate, scaleDilate)));
    cv::morphologyEx(output, output, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(scaleCloseLast, scaleCloseLast)));

    imfill(output);
    return output;

}
Mat preProcess(Mat input) {
    input = gammaCorrection(input, 0.5f);

    Mat hsv;
    cv::cvtColor(input, hsv, COLOR_BGR2HSV);
    //split into channels
    vector<Mat> hsv_channels;
    split(hsv, hsv_channels);
    equalizeHist(hsv_channels[1], hsv_channels[1]);
    //merge channels
    merge(hsv_channels, hsv);
    //convert to bgr
    cv::cvtColor(hsv, input, COLOR_HSV2BGR);
    return input;
}


Mat segmenentFoodIstance(int label, Mat inputPreP) {
    int mB = 0;
    int MB = 0;
    int mG = 0;
    int MG = 0;
    int mR = 0;
    int MR = 0;
    int sc = 0;
    int threshold = 0;
    int scaleDilate = 0;
    int sc2 = 0;
    switch (label) {
    case 1: //pasta with pesto
        mB = 2;
        MB = 133;
        mG = 177;
        MG = 220;
        mR = 210;
        MR = 255;
        sc = 82;
        threshold = 100;
        scaleDilate = 28;
        sc2 = 40;
        break;
    case 2://pasta with tomato
        mB = 27;
        MB = 63;
        mG = 142;
        MG = 200;
        mR = 188;
        MR = 225;
        sc = 64;
        threshold = 100;
        scaleDilate = 25;
        sc2 = 40;

        break;
    case 3://pasta with ragu
        mB = 0;
        MB = 62;
        mG = 123;
        MG = 165;
        mR = 174;
        MR = 209;
        sc = 45;
        threshold = 100;
        scaleDilate = 21;
        sc2 = 48;

        break;
    case 4://pasta with clams
        mB = 0;
        MB = 21;
        mG = 88;
        MG = 153;
        mR = 147;
        MR = 208;
        sc = 49;
        threshold = 50;
        scaleDilate = 25;
        sc2 = 46;

        break;
    case 5://pilaw rice
        mB = 0;
        MB = 18;
        mG = 62;
        MG = 127;
        mR = 63;
        MR = 255;
        sc = 24;
        threshold = 94;
        scaleDilate = 25;
        sc2 = 40;

        break;
    case 6://grilled pork
        mB = 62;
        MB = 127;
        mG = 145;
        MG = 192;
        mR = 176;
        MR = 232;
        sc = 19;
        threshold = 13;
        scaleDilate = 28;
        sc2 = 79;

        break;
    case 7://fish cutlet
        mB = 16;
        MB = 75;
        mG = 80;
        MG = 132;
        mR = 182;
        MR = 224;
        sc = 67;
        threshold = 100;
        scaleDilate = 15;
        sc2 = 40;

        break;
    case 8://rabbit
        mB = 0;
        MB = 38;
        mG = 68;
        MG = 108;
        mR = 151;
        MR = 235;
        sc = 60;
        threshold = 100;
        scaleDilate = 15;
        sc2 = 40;
        break;
    case 9://seafood salad
        mB = 4;
        MB = 31;
        mG = 73;
        MG = 147;
        mR = 112;
        MR = 204;
        sc = 50;
        threshold = 104;
        scaleDilate = 34;
        sc2 = 40;

        break;
    case 10://beans
        mB = 3;
        MB = 42;
        mG = 55;
        MG = 74;
        mR = 126;
        MR = 173;
        sc = 56;
        threshold = 109;
        scaleDilate = 40;
        sc2 = 42;
        break;
    case 11://potatoes
        mB = 0;
        MB = 99;
        mG = 163;
        MG = 252;
        mR = 198;
        MR = 218;
        sc = 83;
        threshold = 50;
        scaleDilate = 18;
        sc2 = 46;

        break;
    default:
        cout << "error label not accepted " << label << endl;
        break;

    }

    Mat mask;
    inRange(inputPreP, Scalar(mB, mG, mR), Scalar(MB, MG, MR), mask);



    mask = postProcessingmask(mask, sc, threshold * 100, scaleDilate, sc2);
    //cout << "label size " << label << endl;
    /*imshow("msckera dopo il ragne", mask);
    waitKey(0);*/

    cv::threshold(mask, mask, 0, label, THRESH_BINARY);

    return mask;


}
void addMask(cv::Mat& image, const cv::Mat& mask) {
    CV_Assert(image.size() == mask.size() && image.type() == CV_8UC1 && mask.type() == CV_8UC1);

    for (int y = 0; y < image.rows; y++) {
        for (int x = 0; x < image.cols; x++) {
            if (mask.at<uchar>(y, x) > 0 && image.at<uchar>(y, x) == 0) {
                image.at<uchar>(y, x) = mask.at<uchar>(y, x);  // Set mask

            }
        }
    }
}

Mat segmetFoodColorLabel(vector<int> labels, Mat imagePalte) {
    if (labels.size() == 0) {
        cout << "no lables" << endl;
        return Mat();
    }

    Mat inputPreProcessed = preProcess(imagePalte);
    Mat mask = Mat::zeros(imagePalte.size(), CV_8UC1);
    std::sort(labels.begin(), labels.end(), customSort);


    for (int i = 0; i < labels.size(); i++) {
        cout << "find label " << labels[i] << endl;
        addMask(mask, segmenentFoodIstance(labels[i], inputPreProcessed));

    }

    return mask;
}

Mat combineMasks(vector<Mat> masks) {
    Mat mask;
    int cont = 0;
    for (int i = 0; i < masks.size(); i++) {
        //check if the mask is empty
        if (masks[i].empty()) {
			//cout << "mask " << i << " is empty" << endl;
			continue;
		}
        if (cont++ == 0)
            mask = Mat::zeros(masks[i].size(), CV_8UC1);
        addMask(mask, masks[i]);
    }
    //cleaning
    cout<<"runnuing extract boutnig box to clean "<< endl;
    extractBoxes(mask);
    return mask;
}

std::map<int, cv::Rect> extractBoxes(Mat& segmentaiton) {

    map<int, cv::Rect> output;
    Mat segentationWorking = segmentaiton.clone();
    //for loop 13 elements
    for (int i = 12; i >= 0; i--) {
        Mat mask;
        cv::threshold(segentationWorking, mask, i, 255, cv::THRESH_BINARY);
        vector<Rect>box = getBoundingBox(mask);


        if (box.size() == 0) {
            //cout << "no box of index " << i + 1 << endl;
        }
        else if (box.size() > 1)
        {
            cout << "too many blobs, cleaning segmenation " << i + 1 << endl;
            Mat larger = mask.clone();
            takeLarger(larger);

            cv::Mat invertedMasklower;
            cv::bitwise_not(mask, invertedMasklower);
            cv::bitwise_or(invertedMasklower, larger, invertedMasklower);


            cv::bitwise_and(segmentaiton, invertedMasklower, segmentaiton);
            cv::bitwise_and(segentationWorking, invertedMasklower, segentationWorking);



            takeLarger(mask);
            output.insert(std::pair<int, cv::Rect>(i + 1, box[0]));
            cv::Mat invertedMask;
            cv::bitwise_not(mask, invertedMask);
            cv::bitwise_and(segentationWorking, invertedMask, segentationWorking);



        }
        else {

            output.insert(std::pair<int, cv::Rect>(i + 1, box[0]));
            cv::Mat invertedMask;

            cv::bitwise_not(mask, invertedMask);
            cv::bitwise_and(segentationWorking, invertedMask, segentationWorking);



        }

    }

    return output;


}

