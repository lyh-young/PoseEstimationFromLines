
#include "PoseTracker.h"
#include "LinePoseEstimation.h"
#include "lineProjectDef.h"
lineProjectDef myLineProjector;
lineAttitudeDef myLineTracker;
bool init_tracker(const char* model_file_name, double* camera_para, int nsample_step, int nsamples)
{
	if (!myLineProjector.read_object_model(model_file_name, nsample_step, nsamples))
	{
		return false;
	}
	if (!camera_para)
	{
		return false;
	}
	myLineProjector.set_proj_para(camera_para);
	myLineTracker.set_camera_para(&myLineProjector);
}


bool run_tracker(IplImage* image, cv::Mat& rmat, cv::Mat& tmat,int nt,  double nmethod)
{
	cv::Mat rmat0 = rmat.clone();
	cv::Mat tmat0 = tmat.clone();
	if (!image)
	{
		return false;
	}

	myLineTracker.m_pLine_project->detect_image_lines(image, false);  
	if (!myLineTracker.get_optimal_pose(image, rmat, tmat, nmethod))
	{
		return false;
	}
	return true;
}
bool display_result(IplImage* image, cv::Mat rmat, cv::Mat tmat)
{
	myLineProjector.display_result_lis(image, rmat, tmat);
	
	return true;
}