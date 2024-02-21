#include "f2.hpp"
#include <iostream>

void max_filter(cv::Mat& src, cv::Mat& dst, int ksize) {
	int anchor = ksize/2; // ksize odd, so /2 is integer
	//dst = cv::Mat::zeros(src.size(), src.type());
	dst = src.clone();
	
	for(int i=0; i<src.rows-ksize; i++) {
		for(int j=0; j<src.cols-ksize; j++) {
			// filter --> find max
			uchar max = src.at<uchar>(i,j);
			for(int k=0; k<=ksize; k++) {
				for(int l=0; l<=ksize; l++) {
					// update if new max and x,y not in anchor
					//if((src.at<uchar>(i+k,j+l) > max) && !(k==anchor && l==anchor)) {
					if(src.at<uchar>(i+k,j+l) > max) {
						//std::cout << "max pixel found in " << (i+k) << "x" << (j+l) << " with value " << (int)src.at<uchar>(i+k,j+l) << "\n";
						max = src.at<uchar>(i+k,j+l);
					}
				}
			}
			dst.at<uchar>(i+anchor, j+anchor) = max;
			std::cout << "Sobstitution in " << j+anchor << "x" << i+anchor << " with " << (int)max << "\n";
		}
	}
	/*
	for (int i = anchor; i < src.rows - anchor; i++) {
        for (int j = anchor; j < src.cols - anchor; j++) {
            uchar maxVal = 0;
            for (int k = -anchor; k <= anchor; k++) {
                for (int l = -anchor; l <= anchor; l++) {
                    uchar val = src.at<uchar>(i + k, j + l);
                    if (val > maxVal) {
                        maxVal = val;
                    }
                }
            }
            dst.at<uchar>(i - anchor, j - anchor) = maxVal;
        }
    }
	*/
	std::cout << "anchor: " << anchor << "\n";
	std::cout << "rows: " << src.rows << ", cols: " << src.cols << "\n";
}

void min_filter(cv::Mat& src, cv::Mat& dst, int ksize) {
	int anchor = ksize/2; // ksize odd, so /2 is integer
	//dst = cv::Mat::zeros(src.size(), src.type());
	dst = src.clone();
	for(int i=0; i<src.rows-ksize; i++) {
		for(int j=0; j<src.cols-ksize; j++) {
			// filter --> find max
			uchar min = src.at<uchar>(i,j);
			for(int k=0; k<=ksize; k++) {
				for(int l=0; l<=ksize; l++) {
					// update if new max and x,y not in anchor
					//if((src.at<uchar>(i+k,j+l) < min) && !(k==anchor && l==anchor)) {
					if(src.at<uchar>(i+k,j+l) < min) {
						//std::cout << "max pixel found in " << (i+k) << "x" << (j+l) << " with value " << (int)src.at<uchar>(i+k,j+l) << "\n";
						min = src.at<uchar>(i+k,j+l);
					}
				}
			}
			dst.at<uchar>(i+anchor, j+anchor) = min;
			std::cout << "Sobstitution in " << j+anchor << "x" << i+anchor << " with " << (int)min << "\n";
		}
	}
	std::cout << "anchor: " << anchor << "\n";
	std::cout << "rows: " << src.rows << ", cols: " << src.cols << "\n";
}
