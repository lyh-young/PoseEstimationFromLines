#pragma once
#include <opencv2/opencv.hpp>
#include "opencv/cv.h"
#include "opencv/cxcore.h"
#include <math.h>
#include <vector>
using namespace std;
using namespace cv;
	struct Plane//�ռ�ƽ��
	{
		int pointID[4];
		CvPoint3D32f pa_orig, pb_orig, pc_orig, pd_orig;//���㣺���������ĸ� �˶�ǰ
		CvPoint3D32f pa, pb, pc, pd;//���㣺���������ĸ�
		double a, b, c, d;//ax+by+cz+d=0
		void GetPoints(CvPoint3D32f p1, CvPoint3D32f p2, CvPoint3D32f p3, CvPoint3D32f p4)
		{
			pa_orig = p1; pb_orig = p2; pc_orig = p3; pd_orig = p4;
		}
		void GetPoints(CvPoint3D32f p1, CvPoint3D32f p2, CvPoint3D32f p3)
		{
			pa_orig = p1; pb_orig = p2; pc_orig = p3;
		}
		void MotionPoints(CvPoint3D32f p1, CvPoint3D32f p2, CvPoint3D32f p3, CvPoint3D32f p4)
		{
			pa = p1; pb = p2; pc = p3; pd = p4;
		}
		void fitplane_analytic(CvPoint3D32f p1, CvPoint3D32f p2, CvPoint3D32f p3, double &A, double &B, double &C, double &D)
		{
			double x1 = p1.x; double y1 = p1.y; double z1 = p1.z;
			double x21 = p2.x - p1.x;	double x31 = p3.x - p1.x;
			double y21 = p2.y - p1.y;	double y31 = p3.y - p1.y;
			double z21 = p2.z - p1.z;	double z31 = p3.z - p1.z;
			A = (y21 * z31 - z21 * y31);
			B = (z21 * x31 - x21 * z31);
			C = (x21 * y31 - y21 * x31);
			D = -x1 * A - y1 * B - z1 * C;
		}
		void FitPlane()
		{
			fitplane_analytic(pa, pb, pc, a, b, c, d);
		}
	};
	struct Quadrangle //�ı��Σ�����������ʾ�ռ���ε�ͶӰͼ��
	{
		CvPoint2D32f pa, pb, pc, pd;
		CvPoint2D32f po;
		bool bVisable;
		void GetPoints(CvPoint2D32f p1, CvPoint2D32f p2, CvPoint2D32f p3, CvPoint2D32f p4)
		{
			pa = p1; pb = p2; pc = p3; pd = p4;
			po = cvPoint2D32f((pa.x + pb.x + pc.x + pd.x) / 4, (pa.y + pb.y + pc.y + pd.y) / 4);
			bVisable = true;
		}
	};

	class lineHiddenRemovelDef
	{
	public:
		lineHiddenRemovelDef(void);
		~lineHiddenRemovelDef(void);
		void hiddenLinesRemoval(cv::Mat& pt3ds, cv::Mat& pt2ds);
		void init_model_para(Plane* planes, Quadrangle* quas, int nface, cv::Mat line3Ds, cv::Mat vpts, int nstep = 0.1, int nsamples = 20);
		void motionPoint(cv::Mat vexPts, cv::Mat imPts);
	private:
		double checkin(double P1x, double P1y, double P2x, double P2y, double P3x, double P3y, double P4x, double P4y, double Px, double Py);
		int    PLP_analytic(cv::Mat plane, cv::Mat pp1, cv::Mat pp2, CvPoint3D32f &plp);
	private:
		cv::Mat m_mModel_Lis;                  //3Dģ��ֱ�� ��Ϊ�˵��������ʽ
		cv::Mat m_mModel_Pts;                    //����
		cv::Mat m_mModel_Pts_im;                 //�����Ӧ��ͼ�������
		Plane* m_planes;                                         //Ŀ��ģ��ƽ��
		Quadrangle* m_quads;                                     //Ŀ��ģ��ƽ��ͶӰ
		int    m_nFace;                                          //�����
		int    m_nLine;                                          //ֱ����Ŀ
		double m_nStep;
		int    m_nVps;                                           //�������
		int    m_nSample;                                        //ÿ��ֱ���ϲ��������
	public:
		int    m_nPt;                           //��������Ŀ

	};

