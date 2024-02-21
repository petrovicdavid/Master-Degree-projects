#include <iostream>
#include <opencv2/highgui.hpp>

int main(int argc, char** argv) {
	
	if(argc>=2) {
		cv::Mat img = cv::imread(argv[1]);
		if(img.data==NULL) {
			printf("Wrong filename!\n");
			return 1;
		}
		printf("Img channels: %d\n", img.channels());
		cv::namedWindow("Example 1");
		cv::imshow("Example 1", img);
		char c = cv::waitKey(0);
		printf("Key pressed: %u\n", c);
	} else {
		printf("Warning! You shall provide an image filename!\n");
		return 2;
	}

	return 0;
}
