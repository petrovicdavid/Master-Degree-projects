#include <iostream>
#include <string>
#include <opencv2/highgui.hpp>

int main(int argc, char** argv) {
	
	if(argc>=2) {
		cv::Mat img = cv::imread(argv[1]);
		if(img.data==NULL) {
			printf("Wrong filename!\n");
			return 1;
		}
		int n_cha = img.channels();
		printf("Img channels: %d\n", n_cha);
		if(n_cha==3) {
			if(std::stoi(argv[2])<0 || std::stoi(argv[2])>2) {
				printf("Wrong channel!\n");
				return 1;
			}
			for(int i=0; i<img.rows; i++)
				for(int j=0; j<img.cols; j++) {
					img.at<cv::Vec3b>(i,j)[std::stoi(argv[2])] = 0;
				}
		}
		cv::namedWindow("Example 1");
		cv::imshow("Example 1", img);
		char c = cv::waitKey(0);
		printf("Key pressed: %u\n", c);
	} else {
		printf("Warning! You shall provide an image filename!\n");
		return 1;
	}

	return 0;
}

