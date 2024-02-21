#include <iostream>
#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

int main(int argc, char** argv) {
	
	if(argc>=2) {
		cv::Mat img = cv::imread(argv[1]);
		if(img.data==NULL) {
			printf("Wrong filename!\n");
			return 1;
		}
		printf("Img channels: %d\n", img.channels());
		int n_cha = img.channels();
		printf("Img channels: %d\n", n_cha);
		if(n_cha==3) {
			cv::Mat channels[3];
			cv::split(img, channels);

			for(int i=0; i<3; i++) {
				std::string s = "Example ";
				s.append(std::to_string(i));
				cv::namedWindow(s, i);
				cv::imshow(s, channels[0]);
				char c = cv::waitKey(0);
				printf("Key pressed: %u\n", c);
			}
		}
	} else {
		printf("Warning! You shall provide an image filename!\n");
		return 2;
	}

	return 0;
}