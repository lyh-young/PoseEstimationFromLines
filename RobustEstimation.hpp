#ifndef RobustE_H
#define RobustE_H
#include <opencv/cv.h>
using namespace cv;
#define TUKEY_C 4.6851
double get_median(double* samples, int nsize);
void get_weights(double* samples, int nsize);
void  FitLine2D( CvPoint2D32f* points, int count, int dist,void *param, float reps, float aeps, float* line );
#endif