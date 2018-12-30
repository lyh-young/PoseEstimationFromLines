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
		/*��������ڲ���*/
		void   set_proj_para(double* para);
		/*��ȡģ�����ݣ��߿�ģ�ͣ�*/
		void   display_result_lis(IplImage* pframe, cv::Mat R, cv::Mat T);
		bool   read_object_model(const char* model_file_name, int nsample_step = 100, int nsamples = 20);
		void   detect_image_lines(IplImage* image, bool bShowLine);

		void     motion_model_pts(cv::Mat r, cv::Mat t);                                 //Ŀ�궥������˶�   
		void     project_3d_model_pts();       //ģ�͵�ͶӰ
		void     hidden_removel_lis();         //ģ��ֱ������
		vector<Line>   project_3d_model_lis(); //���ģ��ֱ��ͶӰ
		//��Ա����
	public:
		lineExtraction               m_line_detector;
		lineHiddenRemovelDef         m_line_hidden_remover; //ģ������
		cv::Mat m_mModel_Lis;                  //3Dģ��ֱ�� ��Ϊ�˵���ʽ
		cv::Mat m_mModel_Pts;                   //����							 4��N  x, y, z, bVisable  ģ������ϵ�µ�3D��
		cv::Mat m_mModel_Pts_motion;            //���� �˶���					 4��N  x, y, z, bVisable
		cv::Mat m_mModel_Pts_im;                //���� ͼ��ͶӰ
		cv::Mat m_mLine_Pts;                    //ֱ���е㣬ֱ�߿��������ж�ʹ��
		cv::Mat m_mLine_Pts_im;                 //ֱ���е�ͼ��ֱ�߿��������ж�ʹ��
		cv::Mat m_mProjPlane;                   //ͶӰƽ��      
		Plane*         m_mplane;                //ģ��ƽ��
		Quadrangle*    m_mQuad;                 //ͶӰƽ��      
		vector<Line>*  m_sLines_image;          //ͼ��ֱ�� 
		int	           m_nSample_Pts;           //���������
		int            m_nLine;                 //��ֱ����
		CvRect         m_rImage_roi;           //Ԥ��Ŀ��ͼ������
		double    m_fx;
		double    m_fy;
		double    m_cx;
		double    m_cy;
		cv::Mat   m_kmat;                        //��������  
		cv::Mat   m_kmat_inv;                    //��һ���������ʹ��
		int xmax, ymax, xmin, ymin;
	private:
		cv::Mat   m_rmat;                        //��������     
		cv::Mat   m_tmat;                        //��������
		int m_nPoint;                            //�������
		int m_nFace;                             //�����
	};
#endif