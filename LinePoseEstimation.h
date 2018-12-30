#pragma once
#ifndef _LINE_ATT
#define _LINE_ATT
#include "lineExtraction.h"
#include "lineParDef.h"
#include "lineProjectDef.h"
using namespace std;

class lineAttitudeDef
{
public:
	lineAttitudeDef(void);
	~lineAttitudeDef(void);
	lineAttitudeDef(lineProjectDef* pLine_project);
	lineAttitudeDef(double* para_oi);
	void   set_camera_para(lineProjectDef* pLine_project = NULL);
	bool   get_optimal_pose(IplImage* pimg, cv::Mat& R, cv::Mat& T, double nmethod = 2);                  //获取优化的目标姿态
	lineProjectDef*   m_pLine_project;        //直线投影
private:
	
	cv::Mat m_kmat;                           //相机内参数
	cv::Mat m_kmat_inv;
	cv::Mat m_mModel_lis;                     //模型直线
	cv::Mat m_mModel_pts;                     //模型点
	cv::Mat m_mModel_pts_im;                  //模型点投影
	int     m_nImg_width;
	int     m_nImg_height;
	double  m_dfx;
	double  m_dfy;
	double  m_dcx;
	double  m_dcy;
	cv::Mat m_Lie_G[6];	//刚体变换指数表示

	//旋转矩阵的指数表示
	cv::Mat m_Lie_g[3];
	//向量表示
	cv::Mat m_Vec_g[3];
	vector<Line>*  Lines_image;
//	vector<int> modelLines, imageLines;
public:
	vector<Line3D> m_sModel_lis;
private:
	double get_point_line_dis(Line li, CvPoint2D32f pt);
	void   get_image_model_pairs(cv::Mat R, cv::Mat T, cv::Mat& pairs);
	void   get_image_model_pairs(cv::Mat pt2ds, vector<Line>* li2ds, cv::Mat& pairs, double disTh, double angTh);
	bool   R_and_T_Methods(cv::Mat& R, cv::Mat& T, int nmethod = 2);
public:
	void   init_camera_paras(double* para_oi);
	bool   R_and_T_Mod_Novel(vector<Line>* li2ds, cv::Mat lipairs, cv::Mat& R, cv::Mat& T);
	bool   R_and_T_Mod_Novel_length(vector<Line>* li2ds, cv::Mat lipairs, cv::Mat& R, cv::Mat& T);   
	bool   get_pose_lines_R_and_T_Lie(vector<Line>* li2ds, cv::Mat lipairs, cv::Mat& R, cv::Mat& T);
};
#endif