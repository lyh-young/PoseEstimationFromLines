#pragma once
#ifndef _LINE_PAR_TYPE
#define _LINE_PAR_TYPE
#include <opencv2/opencv.hpp>
#include "opencv/cv.h"
#include "opencv/cxcore.h"
namespace LineDef
{
	// 空间直线方向向量
	struct Linevector
	{
		double a;
		double b;
		double c;
	};

	// 空间坐标定义
	struct Point3D
	{
		Point3D() { x = y = z = 0; };
		Point3D(double Ix, double Iy, double Iz) { x = Ix; y = Iy; z = Iz; };
		double	x;		// x坐标
		double	y;		// y坐标
		double	z;		// z坐标
		void Getpara(double Ix, double Iy, double Iz) { x = Ix; y = Iy; z = Iz; };

	};

	// 二维点的坐标类型定义
	struct Point2d32f
	{
		double	x;		// x坐标
		double	y;		// y坐标
		double  val[3];  //val[0] 梯度赋值；val[1] 角度；val[2] 距离
		Point2d32f() { x = y = 0; val[0] = val[1] = val[2] = 0; };
		Point2d32f(double Ix, double Iy) { x = Ix; y = Iy; };

		Point2d32f operator - ()
		{
			return Point2d32f(-x, -y);
		};
		friend Point2d32f operator + (const Point2d32f &p1, const Point2d32f &p2)
		{
			return Point2d32f(p1.x + p2.x, p1.y + p2.y);
		};
		friend Point2d32f operator - (const Point2d32f &p1, const Point2d32f &p2)
		{
			return Point2d32f(p1.x - p2.x, p1.y - p2.y);
		};
		double Distance2P(Point2d32f p = Point2d32f(0, 0))
		{
			return sqrt((x - p.x)* (x - p.x) + (y - p.y)* (y - p.y));
		}
	};
	//空间直线定义
	struct Line3D
	{
		Linevector v;
		Point3D  p;
		cv::Mat p1, p2;
		double  len;
		void getLine3D(double a[3], double b[3])  //（方向向量， 点）
		{
			v.a = a[0]; v.b = a[1]; v.c = a[2]; p.x = b[0]; p.y = b[1]; p.z = b[2];
			p1 = (cv::Mat_<float>(3, 1) << a[0], a[1], a[2]);
			p2 = (cv::Mat_<float>(3, 1) << b[0], b[1], b[2]);
			len = sqrt((a[0] - b[0])*(a[0] - b[0]) + (a[1] - b[1])*(a[1] - b[1]) + (a[2] - b[2])*(a[2] - b[2]));

		};
		Line3D getLine3D(Point3D pt1, Point3D pt2)
		{
			Line3D li;
			double norm = sqrt((pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y) + (pt1.z - pt2.z)*(pt1.z - pt2.z));
			li.p = pt1;
			li.v.a = (pt1.x - pt2.x) / norm;
			li.v.b = (pt1.y - pt2.y) / norm;
			li.v.c = (pt1.z - pt2.z) / norm;
			li.p1 = (cv::Mat_<float>(3, 1) << pt1.x, pt1.y, pt1.z);
			li.p2 = (cv::Mat_<float>(3, 1) << pt2.x, pt2.y, pt2.z);
			li.len = norm;
			return li;
		}
	};


	//2D line
	typedef struct Line{
		CvPoint2D32f point1;
		CvPoint2D32f point2;
		CvPoint2D32f midpoint;
		float*       descriptorL;
		float*       descriptorR;
		float*       descriptor;
		float        ang;
		float        len;
		double       par[3]; //直线参数

		Line(CvPoint2D32f p1 = cvPoint2D32f(0, 0), CvPoint2D32f p2 = cvPoint2D32f(0, 0))
		{
			point1 = p1;
			point2 = p2;
			ang = 180.0 * atan2(float(point1.y - point2.y), float(point1.x - point2.x)) / CV_PI;
			len = sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));
			midpoint = cvPoint2D32f((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
			if (ang<0)
				ang += 180;
			//直线参数，a, b， c
			double x1 = p1.y, x2 = p2.y, y1 = p1.x, y2 = p2.x;
			par[0] = (y1 - y2) / len;
			par[1] = (x2 - x1) / len;
			par[2] = (x1*y2 - x2*y1) / len;
		}
		Line(Point2d32f p1, Point2d32f p2)
		{
			point1 = cvPoint2D32f(p1.x, p1.y);
			point2 = cvPoint2D32f(p2.x, p2.y);
			ang = 180.0 * atan2(float(point1.y - point2.y), float(point1.x - point2.x)) / CV_PI;
			len = sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));
			midpoint = cvPoint2D32f((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
			if (ang<0)
				ang += 180;
			//直线参数，a, b， c
			double x1 = p1.y, x2 = p2.y, y1 = p1.x, y2 = p2.x;
			par[0] = (y1 - y2) / len;
			par[1] = (x2 - x1) / len;
			par[2] = (x1*y2 - x2*y1) / len;
		}
	}Line;

}
using namespace LineDef;
#endif