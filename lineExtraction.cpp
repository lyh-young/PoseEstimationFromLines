
#include "lineExtraction.h"
#include "RobustEstimation.hpp"
#include "lsd.h"
double lineExtraction::distance(CvPoint2D32f p1, CvPoint2D32f p2) {
	float x1 = p1.x;
	float x2 = p2.x;
	float y1 = p1.y;
	float y2 = p2.y;
	return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}
double lineExtraction::distance(CvPoint3D32f p1, CvPoint3D32f p2) {
	float x1 = p1.x;
	float x2 = p2.x;
	float y1 = p1.y;
	float y2 = p2.y;
	float z1 = p1.z;
	float z2 = p2.z;
	return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));
}

lineExtraction::lineExtraction(void)
{
	line_detected = new vector<Line>;
	line_detected->clear();
}
lineExtraction::~lineExtraction(void)
{}



std::vector<Line>* lineExtraction::getLineSet()
{
	return line_detected;
}

std::vector< Line>* lineExtraction::mergeLines(float termTh, float distTh, float angTh)
{
	vector<Line>* mergeLine = new vector<Line>;
	vector<bool> flag;            //直线合并标志
	for (int i = 0; i<line_detected->size(); i++)
		flag.push_back(false);
	float angle = 0.0;
	float endpointdis11, endpointdis12, endpointdis21, endpointdis22, endpointdis, midpointdis;
	int pointpair;
	CvPoint2D32f point1, point2;
	//计算直线间的夹角,以及直线间的距离
	for (int i = 0; i<line_detected->size(); i++)	  //遍历的成本过高
	{
		int j;
		if (flag[i])
			continue;
		for (j = i + 1; j<line_detected->size(); j++)
		{
			//判断直线夹角
			if (flag[j])
				continue;
			angle = fabs((*line_detected)[i].ang - (*line_detected)[j].ang);
			if (angle>90)
			{
				angle = 180 - angle;
			}
			if ((angle<angTh)/*&&(angle2<angTh)&&(angle3<angTh)*/)
			{
				//判断端点间距离（p2-p3 or p1-p4）(p1,p2),(p3,p4)

				endpointdis11 = distance((*line_detected)[i].point1, (*line_detected)[j].point1);
				endpointdis12 = distance((*line_detected)[i].point1, (*line_detected)[j].point2);
				endpointdis21 = distance((*line_detected)[i].point2, (*line_detected)[j].point1);
				endpointdis22 = distance((*line_detected)[i].point2, (*line_detected)[j].point2);
				endpointdis = minimum(endpointdis11, endpointdis12, endpointdis21, endpointdis22);
				//返回距离最近的端点队序号
				pointpair = minimal(endpointdis11, endpointdis12, endpointdis21, endpointdis22);
				midpointdis = distance((*line_detected)[i].midpoint, (*line_detected)[j].midpoint);
				if ((endpointdis<distTh/**MIN((*line_detected)[i].len,(*line_detected)[j].len)*/) && (midpointdis>(MAX((*line_detected)[i].len / 2, (*line_detected)[j].len / 2))))
				{
					//判断拟合误差
					float a, b, c;
					CvPoint2D32f* edge = new CvPoint2D32f[4];
					float* line = new float[4];
					edge[0] = (*line_detected)[i].point1;
					edge[1] = (*line_detected)[i].point2;
					edge[2] = (*line_detected)[j].point1;
					edge[3] = (*line_detected)[j].point2;
					//拟合四个点
					FitLine2D(edge, 4, CV_DIST_L2, 0, 0.01, 0.01, line);
					a = -line[0];
					b = line[1];
					c = line[0] * line[3] - line[1] * line[2];
					if (line != NULL)
					{
						delete[] line;
						line = NULL;
					}
					if (edge != NULL)
					{
						delete[] edge;
						edge = NULL;
					}
					//计算拟合误差
					float fiterror1 = fabs((*line_detected)[i].point1.x*b + (*line_detected)[i].point1.y*a + c);
					float fiterror2 = fabs((*line_detected)[i].point2.x*b + (*line_detected)[i].point2.y*a + c);
					float fiterror3 = fabs((*line_detected)[j].point1.x*b + (*line_detected)[j].point1.y*a + c);
					float fiterror4 = fabs((*line_detected)[j].point2.x*b + (*line_detected)[j].point2.y*a + c);
					float fiterror = (fiterror1 + fiterror2 + fiterror3 + fiterror4) / 4;

					if (fiterror<termTh)
					{
						//将端点坐标投影到拟合直线
						double X1, Y1, X2, Y2;
						if (pointpair == 1)
						{
							point1 = (*line_detected)[i].point2;
							point2 = (*line_detected)[j].point2;
						}
						if (pointpair == 2)
						{
							point1 = (*line_detected)[i].point2;
							point2 = (*line_detected)[j].point1;
						}
						if (pointpair == 3)
						{
							point1 = (*line_detected)[i].point1;
							point2 = (*line_detected)[j].point2;
						}
						if (pointpair == 4)
						{
							point1 = (*line_detected)[i].point1;
							point2 = (*line_detected)[j].point1;
						}
						if (point1.x<point2.x)
						{
							X1 = point1.y;
							Y1 = point1.x;
							X2 = point2.y;
							Y2 = point2.x;

						}
						else
						{
							X1 = point2.y;
							Y1 = point2.x;
							X2 = point1.y;
							Y2 = point1.x;

						}
						a = a / sqrt(a*a + b*b);
						b = b / sqrt(a*a + b*b);
						double p1_x = a*a*Y1 - a*b*X1 - b*c;
						double p1_y = b*b*X1 - a*b*Y1 - a*c;
						double p2_x = a*a*Y2 - a*b*X2 - b*c;
						double p2_y = b*b*X2 - a*b*Y2 - a*c;

						double x1 = p1_x;
						double y1 = p1_y;
						double x2 = p2_x;
						double y2 = p2_y;

						line_detected->push_back(Line(cvPoint2D32f(x1, y1), cvPoint2D32f(x2, y2)));
						//lineang.push_back(ang);
						flag.push_back(false);

						flag[i] = true;
						flag[j] = true;
						break;
					}

				}
			}
		}
	}

	for (int i = 0; i<line_detected->size(); i++)
	{
		if ((*line_detected)[i].len<10)
		{
			//continue;
		}
		if (!flag[i])
			mergeLine->push_back((*line_detected)[i]);

	}
	flag.clear();
	line_detected->clear();
	line_detected = mergeLine;
	return mergeLine;
}

//LSD method
void lineExtraction::detect_LineSR(IplImage* img, CvRect* roi)
{
	line_detected->clear();
	imWith = img->width;
	imHeight = img->height;

	if (roi)
		m_rImage_roi = *roi;
	else m_rImage_roi = cvRect(0, 0, m_rImage_roi.width, m_rImage_roi.height);
	cvSetImageROI(img, m_rImage_roi);
	IplImage* img_cv = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	if (img->nChannels == 3)
	{
		cvCvtColor(img, img_cv, CV_RGB2GRAY);
	}
	else
		cvCopy(img, img_cv);
	cvResetImageROI(img);
	image_double image = new_image_double(img_cv->width, img_cv->height);
	unsigned char* im_src = (unsigned char*)img_cv->imageData;
	int xsize = image->xsize;
	int ysize = image->ysize;
	int y, x;
	for (y = 0; y < ysize; y++)
	{
		for (x = 0; x < xsize; x++)
		{
			image->data[y * xsize + x] = im_src[y * img_cv->widthStep + x];
		}
	}
	ntuple_list detected_lines = lsd(image);
	free_image_double(image);
	int i;
	int dim = detected_lines->dim;
	for (i = 0; i < detected_lines->size; i++)
	{
		double x1 = detected_lines->values[i*dim + 0];
		double y1 = detected_lines->values[i*dim + 1] + 1;
		double x2 = detected_lines->values[i*dim + 2];
		double y2 = detected_lines->values[i*dim + 3] + 1;

		x1 += m_rImage_roi.x;
		y1 += m_rImage_roi.y;

		x2 += m_rImage_roi.x;
		y2 += m_rImage_roi.y;

		if (((x1 + x2) / 2.0 > (img->width - 10)) || ((y1 + y2) / 2.0 > (img->height - 10)))
			continue;
		if (((x1 + x2) / 2.0 < 10) || ((y1 + y2) / 2.0 < 10))
			continue;
		line_detected->push_back(Line(cvPoint2D32f(x1, y1), cvPoint2D32f(x2, y2)));
	}
	free_ntuple_list(detected_lines);
	cvReleaseImage(&img_cv);
}

int lineExtraction::minimal(double d1, double d2, double d3, double d4) {
	if (d1 <= d2 && d1 <= d3 && d1 <= d4){ return 1; }
	if (d2 <= d1 && d2 <= d3 && d2 <= d4){ return 2; }
	if (d3 <= d1 && d3 <= d2 && d3 <= d4){ return 3; }
	if (d4 <= d1 && d4 <= d2 && d4 <= d3){ return 4; }
	return 0;
}

double lineExtraction::minimum(double d1, double d2, double d3, double d4) {
	if (d1 <= d2 && d1 <= d3 && d1 <= d4){ return d1; }
	if (d2 <= d1 && d2 <= d3 && d2 <= d4){ return d2; }
	if (d3 <= d1 && d3 <= d2 && d3 <= d4){ return d3; }
	if (d4 <= d1 && d4 <= d2 && d4 <= d3){ return d4; }
	return 0;
}
