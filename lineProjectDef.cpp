#include "lineProjectDef.h"
#include "RobustEstimation.hpp"
#include "opencv2/opencv.hpp"
#include "opencv/cv.h"
#include "opencv/cxcore.h"
#include "opencv/highgui.h"
#include "PoseTracker.h"
#pragma warning(disable:4996)

lineProjectDef::lineProjectDef(void)
{
	m_sLines_image = 0;
	m_rmat = cv::Mat::eye(3, 3, CV_32F);
	m_tmat = cv::Mat::zeros(3, 1, CV_32F);
	m_kmat = cv::Mat::zeros(3, 3, CV_32F);
}
//设置相机内参数
void lineProjectDef::set_proj_para(double* para)
{
	m_kmat.at<float>(0, 0) = para[0];
	m_kmat.at<float>(1, 1) = para[1];
	m_kmat.at<float>(0, 2) = para[2];
	m_kmat.at<float>(1, 2) = para[3];
	m_kmat.at<float>(2, 2) = 1;
	m_fx = para[0];
	m_fy = para[1];
	m_cx = para[2];
	m_cy = para[3];
	m_kmat_inv = m_kmat.inv();
}

lineProjectDef::~lineProjectDef(void)
{
	delete[] m_mplane;
	m_mplane = NULL;
	delete[] m_mQuad; 
	m_mplane = NULL;
}
bool lineProjectDef::read_object_model(const char* model_file_name, int nsample_step, int nsamples)
{
	FILE *fp = NULL;
	fp = fopen(model_file_name, "r");
	if (fp == NULL)
	{
		return false;
	}
	char    fchar[50];
	int fint[5];
	double fdouble[5];
	double fscale = 1.0;
	fscanf(fp, "%s", &fchar);
	fscanf(fp, "%lf", &fscale);
	fscanf(fp, "%s", &fchar);
	fscanf(fp, "%d", &fint[0]);
	//点数
	m_nPoint = fint[0];
	m_mModel_Pts = cv::Mat::zeros(4, m_nPoint, CV_32F);
	int i;
	for (i = 0; i< m_nPoint; i++)
	{
		fscanf(fp, "%d %lf %lf %lf", &fint[0], &fdouble[0], &fdouble[1], &fdouble[2]);
		m_mModel_Pts.at<float>(0, fint[0]) = fdouble[0];                              //x,y,z
		m_mModel_Pts.at<float>(1, fint[0]) = fdouble[1];
		m_mModel_Pts.at<float>(2, fint[0]) = fdouble[2];
		m_mModel_Pts.at<float>(3, fint[0]) = 1.0 / fscale;                              //
	}
	m_mModel_Pts = fscale*m_mModel_Pts;        //对应缩放

	//模型直线条数
	fscanf(fp, "%s", &fchar);
	fscanf(fp, "%d", &fint[0]);
	m_nLine = fint[0];
	m_mModel_Lis = cv::Mat::zeros(5, m_nLine, CV_32F);
	m_mLine_Pts = cv::Mat::zeros(6, m_nLine, CV_32F);
	for (i = 0; i< m_nLine; i++)
	{
		fscanf(fp, "%d %d %d %d %d", &fint[0], &fint[1], &fint[2], &fint[3], &fint[4]);
		m_mModel_Lis.at<float>(0, fint[0]) = fint[1];                              //前两位存储为端点序号
		m_mModel_Lis.at<float>(1, fint[0]) = fint[2];

		m_mModel_Lis.at<float>(2, fint[0]) = fint[3];                              //后两位存储为直线所在平面
		m_mModel_Lis.at<float>(3, fint[0]) = fint[4];

		m_mModel_Lis.at<float>(4, fint[0]) = 1;                                    //最后一位存储直线的可见性标志

		m_mLine_Pts.at<float>(3, fint[0]) = fint[3];
		m_mLine_Pts.at<float>(4, fint[0]) = fint[4];
		m_mLine_Pts.at<float>(5, fint[0]) = 1;

	}
	//读取模型平面
	fscanf(fp, "%s", &fchar);
	fscanf(fp, "%d", &fint[0]);
	m_nFace = fint[0];
	m_mplane = new Plane[m_nFace];
	for (i = 0; i< m_nFace; i++)
	{
		fscanf(fp, "%d %d %d %d %d", &fint[0], &fint[1], &fint[2], &fint[3], &fint[4]);
		m_mplane[fint[0]].pointID[0] = fint[1];
		m_mplane[fint[0]].pointID[1] = fint[2];
		m_mplane[fint[0]].pointID[2] = fint[3];
		m_mplane[fint[0]].pointID[3] = fint[4];
		m_mplane[fint[0]].GetPoints(
			cvPoint3D32f(m_mModel_Pts.at<float>(0, fint[1]), m_mModel_Pts.at<float>(1, fint[1]), m_mModel_Pts.at<float>(2, fint[1])),
			cvPoint3D32f(m_mModel_Pts.at<float>(0, fint[2]), m_mModel_Pts.at<float>(1, fint[2]), m_mModel_Pts.at<float>(2, fint[2])),
			cvPoint3D32f(m_mModel_Pts.at<float>(0, fint[3]), m_mModel_Pts.at<float>(1, fint[3]), m_mModel_Pts.at<float>(2, fint[3])),
			cvPoint3D32f(m_mModel_Pts.at<float>(0, fint[4]), m_mModel_Pts.at<float>(1, fint[4]), m_mModel_Pts.at<float>(2, fint[4])));
	}
	fclose(fp);
	m_mQuad = new Quadrangle[m_nFace];
	m_line_hidden_remover.init_model_para(m_mplane, m_mQuad, m_nFace, m_mModel_Lis, m_mModel_Pts, nsample_step, nsamples);
	m_nSample_Pts = m_line_hidden_remover.m_nPt;
	m_mModel_Pts_motion = m_mModel_Pts.clone();
	return true;
}

//点投影
void lineProjectDef::project_3d_model_pts()
{
	//顶点投影
	m_mModel_Pts_im = m_kmat*m_mModel_Pts_motion.rowRange(cv::Range(0, 3));  //此时m_mModel_Pts_motion点坐标已在像机系下
	m_mModel_Pts_im.row(0) = m_mModel_Pts_im.row(0) / m_mModel_Pts_im.row(2);
	m_mModel_Pts_im.row(1) = m_mModel_Pts_im.row(1) / m_mModel_Pts_im.row(2);
	m_mModel_Pts_im.row(2) = m_mModel_Pts_im.row(2) / m_mModel_Pts_im.row(2);
	//直线中点投影
	m_mLine_Pts_im = m_kmat*m_mLine_Pts.rowRange(cv::Range(0, 3));
	m_mLine_Pts_im.row(0) = m_mLine_Pts_im.row(0) / m_mLine_Pts_im.row(2);
	m_mLine_Pts_im.row(1) = m_mLine_Pts_im.row(1) / m_mLine_Pts_im.row(2);
	m_mLine_Pts_im.row(2) = m_mLine_Pts_im.row(2) / m_mLine_Pts_im.row(2);
	//平面端点投影
	int* fint;
	for (int i = 0; i<m_nFace; i++)
	{
		fint = m_mplane[i].pointID;
		m_mQuad[i].GetPoints(
			cvPoint2D32f(m_mModel_Pts_im.at<float>(0, fint[0]), m_mModel_Pts_im.at<float>(1, fint[0])),
			cvPoint2D32f(m_mModel_Pts_im.at<float>(0, fint[1]), m_mModel_Pts_im.at<float>(1, fint[1])),
			cvPoint2D32f(m_mModel_Pts_im.at<float>(0, fint[2]), m_mModel_Pts_im.at<float>(1, fint[2])),
			cvPoint2D32f(m_mModel_Pts_im.at<float>(0, fint[3]), m_mModel_Pts_im.at<float>(1, fint[3])));
	}
}

void lineProjectDef::motion_model_pts(cv::Mat r, cv::Mat t)
{
	cv::Mat pts = r*m_mModel_Pts.rowRange(cv::Range(0, 3)) + cv::repeat(t, 1, m_nPoint);          //变换到跟踪系
	m_mModel_Pts_motion.rowRange(cv::Range(0, 3)) = m_rmat*pts + cv::repeat(m_tmat, 1, m_nPoint); //变换到相机系
	int* fint;
	for (int i = 0; i<m_nFace; i++)
	{
		fint = m_mplane[i].pointID;
		m_mplane[i].MotionPoints(
			cvPoint3D32f(m_mModel_Pts_motion.at<float>(0, fint[0]), m_mModel_Pts_motion.at<float>(1, fint[0]), m_mModel_Pts_motion.at<float>(2, fint[0])),
			cvPoint3D32f(m_mModel_Pts_motion.at<float>(0, fint[1]), m_mModel_Pts_motion.at<float>(1, fint[1]), m_mModel_Pts_motion.at<float>(2, fint[1])),
			cvPoint3D32f(m_mModel_Pts_motion.at<float>(0, fint[2]), m_mModel_Pts_motion.at<float>(1, fint[2]), m_mModel_Pts_motion.at<float>(2, fint[2])),
			cvPoint3D32f(m_mModel_Pts_motion.at<float>(0, fint[3]), m_mModel_Pts_motion.at<float>(1, fint[3]), m_mModel_Pts_motion.at<float>(2, fint[3])));
		m_mplane[i].FitPlane();  //依据三个或四个点计算出平面参数
	}
	//直线中点运动
	for (int i = 0; i<m_nLine; i++)
	{
		int p1 = m_mModel_Lis.at<float>(0, i);
		int p2 = m_mModel_Lis.at<float>(1, i);
		CvPoint3D32f pts[2];
		pts[0] = cvPoint3D32f(m_mModel_Pts_motion.at<float>(0, p1), m_mModel_Pts_motion.at<float>(1, p1), m_mModel_Pts_motion.at<float>(2, p1));
		pts[1] = cvPoint3D32f(m_mModel_Pts_motion.at<float>(0, p2), m_mModel_Pts_motion.at<float>(1, p2), m_mModel_Pts_motion.at<float>(2, p2));
		m_mLine_Pts.at<float>(0, i) = (pts[0].x + pts[1].x) / 2.0;
		m_mLine_Pts.at<float>(1, i) = (pts[0].y + pts[1].y) / 2.0;
		m_mLine_Pts.at<float>(2, i) = (pts[0].z + pts[1].z) / 2.0;
	}
}

vector<Line>  lineProjectDef::project_3d_model_lis()
{
	vector<Line> lines;
	//获得模型直线投影
	for (int i = 0; i<m_nLine; i++)
	{
		int l1, l2;
		l1 = m_mModel_Lis.at<float>(0, i);
		l2 = m_mModel_Lis.at<float>(1, i);
		CvPoint2D32f p1 = cvPoint2D32f(m_mModel_Pts_im.at<float>(0, l1), m_mModel_Pts_im.at<float>(1, l1));
		CvPoint2D32f p2 = cvPoint2D32f(m_mModel_Pts_im.at<float>(0, l2), m_mModel_Pts_im.at<float>(1, l2));
		lines.push_back(Line(p1, p2));
	}
	return lines;
}

void  lineProjectDef::hidden_removel_lis()                  //模型直线消隐
{
	m_line_hidden_remover.motionPoint(m_mModel_Pts_motion, m_mModel_Pts_im);
	m_line_hidden_remover.hiddenLinesRemoval(m_mLine_Pts, m_mLine_Pts_im);

}
void lineProjectDef::display_result_lis(IplImage* pframe, cv::Mat R, cv::Mat T)
{
	xmax = 0, ymax = 0, xmin = pframe->width, ymin = pframe->height;
	//显示优化结果
	motion_model_pts(R, T);
	project_3d_model_pts();
	hidden_removel_lis();
	//显示跟踪结果
	for (int i = 0; i<m_nLine; i++)
	{
		if (!m_mModel_Lis.at<float>(4, i))
		{
			continue;
		}
		int p1, p2;
		CvPoint line[2];
		p1 = m_mModel_Lis.at<float>(0, i);
		p2 = m_mModel_Lis.at<float>(1, i);
		line[0] = cvPoint(m_mModel_Pts_im.at<float>(0, p1), m_mModel_Pts_im.at<float>(1, p1));
		line[1] = cvPoint(m_mModel_Pts_im.at<float>(0, p2), m_mModel_Pts_im.at<float>(1, p2));
		cvLine(pframe, line[0], line[1], CV_RGB(0, 255, 0), 2, 16, 0);
		xmin = (xmin > line[0].x ? line[0].x : xmin);
		xmin = (xmin > line[1].x ? line[1].x : xmin);
		ymin = (ymin > line[0].y ? line[0].y : ymin);
		ymin = (ymin > line[1].y ? line[1].y : ymin);
		xmax = (xmax < line[0].x ? line[0].x : xmax);
		xmax = (xmax < line[1].x ? line[1].x : xmax);
		ymax = (ymax < line[0].y ? line[0].y : ymax);
		ymax = (ymax < line[1].y ? line[1].y : ymax);
		xmin -= 20;
		ymin -= 20;
		xmax += 20;
		ymax += 20;
		xmin = max(xmin, 0);
		ymin = max(ymin, 0);
		xmax = min(xmax, pframe->width);
		ymax = min(ymax, pframe->height);
	}

}
bool first_frame = true;
void lineProjectDef::detect_image_lines(IplImage* image, bool bShowLine)
{
	if (first_frame){
		m_rImage_roi = cvRect(0, 0, image->width, image->height);
		first_frame = false;
	}
	else{
		m_rImage_roi = cvRect(xmin, ymin, xmax - xmin, ymax - ymin);
	}
	m_line_detector.detect_LineSR(image, &m_rImage_roi);
	m_line_detector.mergeLines(1.0, 10, 3);
	m_sLines_image = m_line_detector.getLineSet();
	m_mProjPlane = cv::Mat::zeros(3, m_sLines_image->size(), CV_32F);
	for (int i = 0; i<m_sLines_image->size(); i++)
	{
		cv::Mat n1 = (cv::Mat_<float>(3, 1) << (*m_sLines_image)[i].point1.x, (*m_sLines_image)[i].point1.y, 1);
		cv::Mat n2 = (cv::Mat_<float>(3, 1) << (*m_sLines_image)[i].point2.x, (*m_sLines_image)[i].point2.y, 1);

		CvPoint2D32f p1 = (*m_sLines_image)[i].point1;
		CvPoint2D32f p2 = (*m_sLines_image)[i].point2;
		if (bShowLine)
		{
			cvLine(image, cvPointFrom32f(p1), cvPointFrom32f(p2), CV_RGB(255, 0, 0), 1, 16);
		}
		//图像点变换到相机系
		n1 = m_kmat_inv*n1;
		n2 = m_kmat_inv*n2;
		cv::Mat plane = n1.cross(n2);
		cv::normalize(plane, plane);
		//图像直线同光心确定的平面参数
		m_mProjPlane.at<float>(0, i) = plane.at<float>(0, 0);
		m_mProjPlane.at<float>(1, i) = plane.at<float>(1, 0);
		m_mProjPlane.at<float>(2, i) = plane.at<float>(2, 0);
	}
}