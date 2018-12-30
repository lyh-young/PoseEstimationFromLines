#pragma once
#ifndef _LINE_TYPE
#define _LINE_TYPE
#include "lineHiddenRemovelDef.h"
#include "lineParDef.h"
#include "lineExtraction.h"

	class lineProjectDef
	{
	public:
		lineProjectDef(void);
		~lineProjectDef(void);
		/*设置相机内参数*/
		void   set_proj_para(double* para);
		/*读取模型数据（线框模型）*/
		void   display_result_lis(IplImage* pframe, cv::Mat R, cv::Mat T);
		bool   read_object_model(const char* model_file_name, int nsample_step = 100, int nsamples = 20);
		void   detect_image_lines(IplImage* image, bool bShowLine);

		void     motion_model_pts(cv::Mat r, cv::Mat t);                                 //目标顶点刚体运动   
		void     project_3d_model_pts();       //模型点投影
		void     hidden_removel_lis();         //模型直线消隐
		vector<Line>   project_3d_model_lis(); //获得模型直线投影
		//成员变量
	public:
		lineExtraction               m_line_detector;
		lineHiddenRemovelDef         m_line_hidden_remover; //模型消隐
		cv::Mat m_mModel_Lis;                  //3D模型直线 存为端点形式
		cv::Mat m_mModel_Pts;                   //顶点							 4×N  x, y, z, bVisable  模型坐标系下的3D点
		cv::Mat m_mModel_Pts_motion;            //顶点 运动后					 4×N  x, y, z, bVisable
		cv::Mat m_mModel_Pts_im;                //顶点 图像投影
		cv::Mat m_mLine_Pts;                    //直线中点，直线快速消隐判断使用
		cv::Mat m_mLine_Pts_im;                 //直线中点图像，直线快速消隐判断使用
		cv::Mat m_mProjPlane;                   //投影平面      
		Plane*         m_mplane;                //模型平面
		Quadrangle*    m_mQuad;                 //投影平面      
		vector<Line>*  m_sLines_image;          //图像直线 
		int	           m_nSample_Pts;           //采样点个数
		int            m_nLine;                 //总直线数
		CvRect         m_rImage_roi;           //预测目标图像区域
		double    m_fx;
		double    m_fy;
		double    m_cx;
		double    m_cy;
		cv::Mat   m_kmat;                        //相机外参数  
		cv::Mat   m_kmat_inv;                    //归一化像点坐标使用
		int xmax, ymax, xmin, ymin;
	private:
		cv::Mat   m_rmat;                        //相机外参数     
		cv::Mat   m_tmat;                        //相机外参数
		int m_nPoint;                            //顶点个数
		int m_nFace;                             //面个数
	};
#endif