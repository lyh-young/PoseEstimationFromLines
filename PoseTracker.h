#include "opencv2/opencv.hpp"
#include "opencv/cv.h"
#include "opencv/cxcore.h"
using namespace cv;
using namespace std;
bool init_tracker(const char* model_file_name, double* camera_para, int nsample_step = 100, int nsamples= 20);
bool run_tracker(IplImage* image, cv::Mat& rmat, cv::Mat& tmat, int nt, double nmethod = 2);
bool display_result(IplImage* image, cv::Mat rmat, cv::Mat tmat);