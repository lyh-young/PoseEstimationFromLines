#include "lineHiddenRemovelDef.h"
lineHiddenRemovelDef::lineHiddenRemovelDef(void)
{
}

lineHiddenRemovelDef::~lineHiddenRemovelDef(void)
{
}
void lineHiddenRemovelDef::init_model_para(Plane* planes, Quadrangle* quas, int nface, cv::Mat line3Ds, cv::Mat vpts, int nstep, int nsamples)
{
	m_planes = planes;
	m_quads = quas;
	m_nSample = nsamples;
	m_mModel_Lis = line3Ds;
	m_mModel_Pts = vpts;
	m_nFace = nface;
	m_nStep = nstep;
	m_nLine = line3Ds.cols;
	m_nVps = vpts.cols;
	m_nStep = m_nStep / 10000.0;

}
double lineHiddenRemovelDef::checkin(double P1x, double P1y, double P2x, double P2y, double P3x, double P3y, double P4x, double P4y, double Px, double Py)
{
	CvSeq* contour;
	CvMemStorage* storage = cvCreateMemStorage(0);
	contour = cvCreateSeq(CV_SEQ_POLYGON, /* sequence of integer elements */
		sizeof(CvContour), /* header size - no extra fields */
		sizeof(CvPoint), /* element size */
		storage /* the container storage */);
	CvPoint pt;
	pt.x = int(P1x + 0.5);	pt.y = int(P1y + 0.5);
	cvSeqPush(contour, &pt);
	pt.x = int(P2x + 0.5);	pt.y = int(P2y + 0.5);
	cvSeqPush(contour, &pt);
	pt.x = int(P3x + 0.5);	pt.y = int(P3y + 0.5);
	cvSeqPush(contour, &pt);
	pt.x = int(P4x + 0.5);	pt.y = int(P4y + 0.5);
	cvSeqPush(contour, &pt);
	CvPoint2D32f p;
	p.x = float(Px + 0.5); p.y = float(Py + 0.5);
	double ret = cvPointPolygonTest(contour, p, 1);//���ֶ�ָ������ʱ������1 ��cvFindContoursʱ0,1�Կ�
	cvReleaseMemStorage(&storage);
	return ret;
}
//��������ƽ��Ľ���
int lineHiddenRemovelDef::PLP_analytic(cv::Mat plane, cv::Mat pp1, cv::Mat pp2, CvPoint3D32f &plp)
{
	double a = plane.at<float>(0, 0);
	double b = plane.at<float>(1, 0);
	double c = plane.at<float>(2, 0);
	double d = plane.at<float>(3, 0);
	CvPoint3D32f p1 = cvPoint3D32f(pp1.at<float>(0, 0), pp1.at<float>(1, 0), pp1.at<float>(2, 0));
	CvPoint3D32f p2 = cvPoint3D32f(pp2.at<float>(0, 0), pp2.at<float>(1, 0), pp2.at<float>(2, 0));

	double l = p2.x - p1.x;//ֱ�ߵķ�������(m,n,l)
	double m = p2.y - p1.y;
	double n = p2.z - p1.z;

	double eps = 0;			//������ֵ�ļ��㾫��
	if (fabs(l*a + m*b + n*c) <= eps)//ֱ������ƽ��
	{
		plp = cvPoint3D32f(0, 0, 0);
		return -1;
	}

	double dis = sqrt(l*l + m*m + n*n);
	double cos_alpha = l / dis;
	double cos_beta = m / dis;
	double cos_gamma = n / dis;

	double x1 = p1.x; double y1 = p1.y; double z1 = p1.z;

	double t = (a*x1 + b*y1 + c*z1 + d) / (a*cos_alpha + b*cos_beta + c*cos_gamma);

	plp.x = x1 - t * cos_alpha;
	plp.y = y1 - t * cos_beta;
	plp.z = z1 - t * cos_gamma;
	return 1;
}
//ֱ�����������жϷ�
void lineHiddenRemovelDef::hiddenLinesRemoval(cv::Mat& pt3ds, cv::Mat& pt2ds)
{
	//�ж϶���ɼ���׼��Ϊ������ͶӰ��һ������η�Χ�ڣ�������ȴ���ͬ������ȣ���ö��㲻�ɼ�
	//ƽ��ɼ����ж�׼��Ϊ��ƽ�泬���������㲻�ɼ���ƽ�治�ɼ�������ƽ�棬ֻҪ��һ���㲻�ɼ���ƽ��Ͳ��ɼ���
	for (int i = 0; i< m_nLine; i++)
	{
		pt3ds.at<float>(5, i) = 1;
		CvPoint3D32f p3d;
		CvPoint2D32f p2d;

		p3d = cvPoint3D32f(pt3ds.at<float>(0, i), pt3ds.at<float>(1, i), pt3ds.at<float>(2, i));
		p2d = cvPoint2D32f(pt2ds.at<float>(0, i), pt2ds.at<float>(1, i));

		int myFaceID1 = 0, myFaceID2 = 0;

		myFaceID1 = pt3ds.at<float>(3, i);
		myFaceID2 = pt3ds.at<float>(4, i);


		if (p2d.x <1 || p2d.y <1)
		{
			pt3ds.at<float>(5, i) = 0;
			continue;
		}

		double ret = 0;
		for (int j = 0; j< m_nFace; j++)
		{
			if (myFaceID1 == j || myFaceID2 == j)
				continue;
			ret = checkin(m_quads[j].pa.x, m_quads[j].pa.y,
				m_quads[j].pb.x, m_quads[j].pb.y,
				m_quads[j].pc.x, m_quads[j].pc.y,
				m_quads[j].pd.x, m_quads[j].pd.y,
				p2d.x, p2d.y);
			if (ret > 0)//���ڲ�������������������������������������������������������Ҫ�úÿ����£�����λ�ڶ��ƽ��ǵ㴦����θ����жϿɼ���
			{
				//�ٽ���ǰ���ж�
				cv::Mat pEye = cv::Mat::zeros(3, 1, CV_32F);//�ӵ�E
				cv::Mat pt = (cv::Mat_<float>(3, 1) << p3d.x, p3d.y, p3d.z);
				CvPoint3D32f pp;
				int nret = 0;
				cv::Mat plane = (cv::Mat_<float>(4, 1) << m_planes[j].a, m_planes[j].b, m_planes[j].c, m_planes[j].d);
				nret = PLP_analytic(plane, pEye, pt, pp);   //������ռ��ͶӰ��ͬһͼ����ģ��ƽ���ϵ�λ��

				if ((fabs(p3d.x - pp.x) + fabs(p3d.y - pp.y) + fabs(p3d.z - pp.z))<1e-2)  //1e-1
				{
					continue;
				}
				double ME = 0, PE = 0;
				ME = p3d.x*p3d.x + p3d.y*p3d.y + p3d.z*p3d.z; //���жϵ����
				PE = pp.x*pp.x + pp.y*pp.y + pp.z*pp.z;       //ƽ���ϵ����
				if (PE < ME - 1e-2)
				{
					pt3ds.at<float>(5, i) = 0;
					break;
				}
			}
		}
	}
	//ģ��ֱ�߿ɼ����ж�׼��Ϊ��ģ��ֱ�߶�Ӧ���������㶼���ɼ�����ģ��ֱ�߲��ɼ�
	//ֱ���е㲻�ɼ�
	for (int i = 0; i< m_nLine; i++)
	{
		m_mModel_Lis.at<float>(4, i) = 1;
		//int p1 = m_mModel_Lis.at<float>(0, i);
		//int p2 = m_mModel_Lis.at<float>(1, i);
		//int pl1 = m_mModel_Lis.at<float>(2, i);
		//int pl2 = m_mModel_Lis.at<float>(3, i);
		//ֱ�����ڵ�ƽ�涼���ɼ�����ֱ�߲��ɼ�����ǰ�ĳ�����ֻ���������е�ɼ���Ϊ����
		if (!pt3ds.at<float>(5, i))
		{
			m_mModel_Lis.at<float>(4, i) = 0;
		}

	}
}


void lineHiddenRemovelDef::motionPoint(cv::Mat vexPts, cv::Mat imPts)
{
	m_mModel_Pts = vexPts;
	m_mModel_Pts_im = imPts;
}