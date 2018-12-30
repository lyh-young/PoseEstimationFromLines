#pragma once
#ifndef _LINE_EXTRACT_TYPE
#define _LINE_EXTRACT_TYPE
#include <math.h>
#include <vector>
#include "lineParDef.h"
using namespace std;
class lineExtraction
{
public:
	lineExtraction(void);
	~lineExtraction(void);
	std::vector<Line>* getLineSet();
	std::vector< Line>* mergeLines(float termTh = 3.0, float distTh = 10, float angTh = 5.0);
	void detect_LineSR(IplImage* img, CvRect* roi = NULL);
private:
	double distance(CvPoint2D32f p1, CvPoint2D32f p2);
	double distance(CvPoint3D32f p1, CvPoint3D32f p2);
	int    minimal(double d1, double d2, double d3, double d4);
	double minimum(double d1, double d2, double d3, double d4);
private:
	std::vector<Line>* line_detected;
	int       imWith;
	int       imHeight;
	CvRect    m_rImage_roi;

};
#endif