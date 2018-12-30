#include <opencv2/opencv.hpp>
#include "opencv/cv.h"
#include "opencv/cxcore.h"
#include "opencv/highgui.h"
#include <io.h>
#include <string>
#include <fstream>
#include <sstream>
#include <time.h>
#include "PoseTracker.h"
#include "lineParDef.h"
#if defined(__GNUC__)
#include <dirent.h>
#endif
using namespace std;

#pragma warning(disable:4996)
vector<int> modelLines, imageLines;



int main(int argc, char* argv[])
{
#if defined(_MSC_VER)
		string imgpre = "E:/Test-for-paper/ModelBasedTracking_New_sequence/ModelTracking/ModelTracking/Rotation_new/Sequences/";
		string Campre = "E:/Test-for-paper/ModelBasedTracking_New_sequence/ModelTracking/ModelTracking/Rotation_new/Cam/";
		string Plapre = "E:/Test-for-paper/ModelBasedTracking_New_sequence/ModelTracking/ModelTracking/Rotation_new/Pla/";
		string filename, cam, pla;
		vector<string> img_names, Cam_names, Pla_names;
		//读取目录中所有图像名字
		struct _finddata_t bmp_file, cam_file, pla_file;
		intptr_t hFile, cFile, pFile;
		filename = imgpre + "/*.jpg"; cam = Campre + "/*.cameraPose"; pla = Plapre + "/*.planePose";
		if ((hFile = _findfirst(filename.data(), &bmp_file)) == -1L){
			printf("no *.bmp files in directory \n");return 0;}
		else{do{img_names.push_back(imgpre + "/" + bmp_file.name);} while (_findnext(hFile, &bmp_file) == 0);}
		if ((cFile = _findfirst(cam.data(), &cam_file)) == -1L){printf("no *.camerapose files in directory \n");return 0;}
		else{do{Cam_names.push_back(Campre + "/" + cam_file.name);} while (_findnext(cFile, &cam_file) == 0);}
		if ((pFile = _findfirst(pla.data(), &pla_file)) == -1L){printf("no *.plane files in directory \n");return 0;}
		else{do{Pla_names.push_back(Plapre + "/" + pla_file.name);} while (_findnext(pFile, &pla_file) == 0);}

#elif defined(__GNUC__)
	vector<string> img_names;
    string filename;
	DIR* dir = opendir("/home/sky/桌面/ModelTracking/ModelTracking/Rotation_new/Sequences");
    string imgpre = "/home/sky/桌面/ModelTracking/ModelTracking/Rotation_new/Sequences/";
	dirent* p = NULL;
	while ((p = readdir(dir)) != NULL)
	{
		if (p->d_name[0] != '.')//d_name是一个char数组，存放当前遍历到的文件名
		{
			string name = imgpre + string(p->d_name);
			img_names.push_back(name);
		}
	}
	closedir(dir);//关闭指定目录
#endif
		double camera_oi_para[4] = { 3010.32192 / 2, 3010.88935 / 2, 845.0 / 2, 471.0 / 2 };
		int nstep = 10; //单位毫米
		int nsamples = 10; //每条模型直线采样个数
		filename = imgpre + "/Model_bak.model";
		//模型跟踪初始化
		if (!init_tracker(filename.data(), camera_oi_para, nstep, nsamples)){
			return 0;
		}
		//位姿初值
		cv::Mat r_mat = cv::Mat::zeros(3, 3, CV_32FC1);
		cv::Mat t_mat = cv::Mat::zeros(3, 1, CV_32FC1);
		r_mat.at<float>(0, 0) = 0.16034; r_mat.at<float>(0, 1) = -0.9872765; r_mat.at<float>(0, 2) = 0.001;
		r_mat.at<float>(1, 0) = 0.256882; r_mat.at<float>(1, 1) = 0.05267472; r_mat.at<float>(1, 2) = 0.96538;
		r_mat.at<float>(2, 0) = -0.953091; r_mat.at<float>(2, 1) = -0.1518919; r_mat.at<float>(2, 2) = 0.26159;
		t_mat.at<float>(0, 0) = -0.92224; t_mat.at<float>(1, 0) = -1.49767; t_mat.at<float>(2, 0) = -92.69;

		IplImage* pframe = NULL;
		double nst = cv::getTickCount();
		for (int i = 0; i < 50; i++){
			pframe = cvLoadImage(img_names[i].data(), 1);
			run_tracker(pframe, r_mat, t_mat, i, 1);
			display_result(pframe, r_mat, t_mat);
			cvShowImage("Tracking result", pframe);
			cvWaitKey(1);
			cvReleaseImage(&pframe);
			modelLines.clear(); imageLines.clear();
		}
		double net = cv::getTickCount();
		cout << "Time:" << (net - nst) * 1000 / cv::getTickFrequency() << endl;
		cvDestroyAllWindows();
		return 0;
	}

