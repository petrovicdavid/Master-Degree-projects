#include "p_measure.h"
#include "gt_reader.h"
#include "UmboFunct.h"
#include <iostream>

#include <stdexcept>

int const N_CLASSES = 14;

double mAP() {
    int gt_counter[N_CLASSES];    // counter for ground truth bounding boxes
    for(int i=0; i<N_CLASSES; i++)
        gt_counter[i]=0;
    std::map<int, std::vector<int>> map;    // < # class, 0 if FP / 1 if TP >

    for(int n=1; n<9; n++) {
        std::vector<std::vector<std::vector<int>>> gt = bbreader(getProjectPath()+"/data/Food_leftover_dataset/", n);
        std::vector<std::vector<std::vector<int>>> pr = bbreader(getProjectPath()+"/data/result/", n);

        for(int i=0; i<pr.size(); i++) {   // for each file in tray n
            std::vector<std::vector<int>> pr_file = pr[i];
            std::vector<std::vector<int>> gt_file = gt[i];

            for(int j=0; j<gt_file.size(); j++) {
                gt_counter[gt_file[j][0]]++;
            }

            //gt_counter += gt_file.size();   // adding ground truth bounding boxes of file gt[i]

            for(int j=0; j<pr_file.size(); j++) {   // for each bounding box detected

                std::vector<int> pr_bb = pr_file[j];
                
                bool found = false; // interrupt for after founding same class

                for(int k=0; k<gt_file.size() && !found; k++) {   // for each ground truth bounding box do IoU with same class as pr_bb
                    std::vector<int> gt_bb = gt_file[k];
                    if(gt_bb[0]==pr_bb[0]) {  // same class
                        double d = IoU(cv::Rect(gt_bb[1], gt_bb[2], gt_bb[3], gt_bb[4]),
                                        cv::Rect(pr_bb[1], pr_bb[2], pr_bb[3], pr_bb[4]));
                        std::vector<int> iou_result;
                        if(d>=0.5) {    // TP
                            if(map.find(pr_bb[0]) == map.end()) {   // key not found in map
                                iou_result.push_back(1);
                                map.insert({pr_bb[0], iou_result});
                            } else
                                (map.at(pr_bb[0])).push_back(1);
                        } else {    // FP
                            if(map.find(pr_bb[0]) == map.end()) {   // key not found in map
                                iou_result.push_back(0);
                                map.insert({pr_bb[0], iou_result});
                            } else
                                (map.at(pr_bb[0])).push_back(0);
                        }
                        found=true;
                    }
                }
            }
        }
    }

    std::vector<double> ap; // average precision vector

    // calculating AP for every class
    for(int i=1; i<N_CLASSES; i++) {    // for each class
        if( !(map.find(i) == map.end()) ) {    // class included in map

            std::vector<int> res = map.at(i);
            std::vector<double> precision, recall;
            int tp=0;
            int fp=0;

            for(int k=0; k<res.size(); k++) {   // sliding through results of TP/FP
                if(res[k]==1) // correct classification
                    tp++;
                else
                    fp++;
                // precision and recall calc for each index
                precision.push_back( (double)tp/(tp+fp) );
                recall.push_back( (double)tp/gt_counter[i] );
            }
            // removing duplicates in recall
            std::vector<int> index;
            for(int k=0; k<recall.size()-1; k++) {
                if(recall[k]==recall[k+1]) { // same recall
                    if(precision[k]>precision[k+1]) {   // precision of k is alway greater than k+1,k+2...
                        index.push_back(k+1);
                        precision[k+1]=precision[k];
                    }
                }
            }
            for(int k=0; k<index.size(); k++) {
                precision.erase( precision.begin()+index[k]-k );
                recall.erase( recall.begin()+index[k]-k );
            }
            // max precision to the right
            double max=0;
            double sum=0;   // needed for AP
            for(int k=precision.size()-1; k>=0; k--) {
                if(precision[k]>max)
                    max = precision[k];
                else
                    precision[k] = max;
            }
            bool flag = false;
            if (i == 8)
                flag = true;
            for(int k=0, r=0; k<11; k++) {
                for(; recall[r]<(double)k/10; r++);
                if(precision.size()-1> r)
					sum+=precision[r];  
            }
            // average precision (AP)
            ap.push_back( ((double)1/11)*sum );
        }
    }
    //std::cout <<"MAP " << map.size()<< std::endl;
    double sum = 0;
    for(int i=0; i<map.size(); i++) {
        //std::cout<<"AP for class " <<i+1;
        //std::cout<<": " << ap[i] <<std::endl;
        sum += ap[i];
    }
    return (sum/map.size());
}

double IoU(cv::Rect gtr, cv::Rect pr) {
    // intersection
    cv::Rect ir = gtr & pr;
    // union
    double ur = gtr.area() + pr.area() - ir.area();
    return (ir.area()/ur);
}

void mIoU() {
    std::string gt_path = getProjectPath() + "/data/Food_leftover_dataset/tray";
    std::string pr_path = getProjectPath() + "/data/result/tray";
    std::string subdir = "/masks/";
    std::vector<std::string> filenames = {"food_image_mask.png", "leftover1.png", "leftover2.png"};

    /**
     * Classes:
     * 0. Background
     * 1. pasta with pesto
     * 2. pasta with tomato sauce
     * 3. pasta with meat sauce
     * 4. pasta with clams and mussels
     * 5. pilaw rice with peppers and peas
     * 6. grilled pork cutlet
     * 7. fish cutlet
     * 8. rabbit
     * 9. seafood salad
     * 10. beans
     * 11. basil potatoes
     * 12. salad
     * 13. bread
    */
    size_t intersec[N_CLASSES];    // intersection pixels
    size_t n_intersec[N_CLASSES];  // NOT intersection pixels
    for(int i=0; i<N_CLASSES; i++) {
        intersec[i]=0;
        n_intersec[i]=0;
    }

    for(int n=1; n<9; n++) {    // for each tray n  
        for(int fn=0; fn<filenames.size(); fn++) {  // for each image in tray n
            std::string tmp_path = gt_path+std::to_string(n)+subdir+filenames[fn];
            cv::Mat gtimg = cv::imread( tmp_path, cv::IMREAD_GRAYSCALE);
            tmp_path = (pr_path+std::to_string(n)+subdir+filenames[fn]);
            cv::Mat primg = cv::imread( tmp_path, cv::IMREAD_GRAYSCALE);
            
            if(gtimg.data==NULL || primg.data==NULL) {
                throw std::invalid_argument("Image "+filenames[fn]+" not found!");
            }
            
            // checking pixel segmentation
            for(size_t i=0; i<gtimg.rows; i++) {
                for(size_t j=0; j<gtimg.cols; j++) {
                    uchar gtval = gtimg.at<uchar>(i,j);
                    uchar prval = primg.at<uchar>(i,j);
                    
                    if( gtval == prval ) {  // correct segmentation between ground truth and prediction
                        intersec[gtval]++;
                    } else {
                        n_intersec[gtval]++;
                        n_intersec[prval]++;
                    }
                }
            }
        }
    }
    // IoU array
    double iou[N_CLASSES];
    int zero_count=0;   // count classes not needed in this image
    for(int i=0; i<N_CLASSES; i++) {
        if(intersec[i]!=0 && n_intersec[i]!=0)
            iou[i] = (double)intersec[i]/( intersec[i] + n_intersec[i] );
        else {
            iou[i]=0;
            zero_count++;
        }
    }
    for(int i=0; i<N_CLASSES; i++) {
        std::cout<<"mIoU for class " <<i<<": " << iou[i] <<std::endl;
    }
}

void leftover() {
    std::string pr_path = getProjectPath() + "/data/result/tray";
    std::string subdir = "/masks/";
    std::vector<std::string> filenames = {"food_image_mask.png", "leftover1.png", "leftover2.png", "leftover3.png"};

    std::ofstream file;
    file.open(getProjectPath() + "/data/result/results.txt");
    
    /**
     * Classes:
     * 0. Background
     * 1. pasta with pesto
     * 2. pasta with tomato sauce
     * 3. pasta with meat sauce
     * 4. pasta with clams and mussels
     * 5. pilaw rice with peppers and peas
     * 6. grilled pork cutlet
     * 7. fish cutlet
     * 8. rabbit
     * 9. seafood salad
     * 10. beans
     * 11. basil potatoes
     * 12. salad
     * 13. bread
    */
    size_t bfarr[N_CLASSES];
    size_t afarr[N_CLASSES];

    for(int n=1; n<9; n++) {    // for each tray n
        
        std::string tmp_path = pr_path+std::to_string(n)+subdir+filenames[0];
        cv::Mat bfimg = cv::imread( tmp_path, cv::IMREAD_GRAYSCALE);

        for(int af_fn=1; af_fn<filenames.size(); af_fn++) {  // for each after image in tray n
            file<<"Tray "<<n<<":"<<std::endl;
            file<<"Before Image: "<<filenames[0]<<", After Image: "<<filenames[af_fn]<<std::endl;
            for(int i=0; i<N_CLASSES; i++) {
                bfarr[i]=0;
                afarr[i]=0;
            }

            tmp_path = (pr_path+std::to_string(n)+subdir+filenames[af_fn]);
            cv::Mat afimg = cv::imread( tmp_path, cv::IMREAD_GRAYSCALE);

            if(bfimg.data==NULL || afimg.data==NULL) {
                throw std::invalid_argument("Image not found!");
            }

            // counting pixel in segmentations
            for(size_t i=0; i<bfimg.rows; i++) {
                for(size_t j=0; j<bfimg.cols; j++) {
                    if(bfimg.at<uchar>(i,j)!=0)
                        bfarr[ bfimg.at<uchar>(i,j) ]++;
                }
            }
            for(size_t i=0; i<afimg.rows; i++) {
                for(size_t j=0; j<afimg.cols; j++) {
                    if(afimg.at<uchar>(i,j)!=0)
                        afarr[ afimg.at<uchar>(i,j) ]++;
                }
            }

            for(int i=0; i<N_CLASSES; i++) {
                if(bfarr[i]!=0) {
                    double foo = (double)afarr[i] / bfarr[i];
                    if(foo>1)
                        foo=1;
                    file<<"Class "<<i<<" leftover: "<< foo <<std::endl;
                }
            }
        }
    }
    file.close();
}