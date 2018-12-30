
#include "RobustEstimation.hpp"
void  FitLine2D(CvPoint2D32f* points, int count, int dist, void *param, float reps, float aeps, float* line)
{
	CvMat mat = cvMat(1, count, CV_32FC2, points);
	float _param = param != NULL ? *(float*)param : 0.f;
	assert(dist != CV_DIST_USER);
	cvFitLine(&mat, dist, _param, reps, aeps, line);
}


double tukey_weight(double x)
{
	if (fabs(x) >= TUKEY_C)
	{
		return 0;
	}
	x = x / TUKEY_C;
	return (1 - x*x)*(1 - x*x);
}

static int weight_cmp(const void* p1, const void* p2)
{
	double* _p1 = (double*)p1;
	double* _p2 = (double*)p2;

	if ((*_p1) > (*_p2))
		return -1;
	if ((*_p1) < (*_p2))
		return 1;
	return 0;
}
double get_median(double* samples, int nsize)
{
	double med_weight = 10;
	double* weights = new double[nsize];
	memset(weights, 0, sizeof(double)*nsize);
	memcpy(weights, samples, sizeof(double)*nsize);
	qsort(weights, nsize, sizeof(double), &weight_cmp);
	//获得中值
	med_weight = (weights[nsize / 2] + weights[nsize / 2 - 1]) / 2.0;
	delete[] weights;
	return med_weight;
}

void get_weights(double* samples, int nsize)
{
	double med_dis = 10;
	double med_detar = 10;
	double detar = 10;
	double* detars = new double[nsize];
	memset(detars, 0, sizeof(double)*nsize);
	med_dis = get_median(samples, nsize);

	for (int k2 = 0; k2<nsize; k2++)
	{
		detars[k2] = samples[k2] - med_dis;
	}

	med_detar = get_median(detars, nsize);

	for (int k2 = 0; k2<nsize; k2++)
	{
		detars[k2] = fabs(detars[k2] - med_detar);
	}
	med_detar = get_median(detars, nsize);
	detar = med_detar / 1.48;
	for (int k2 = 0; k2<nsize; k2++)
	{
		samples[k2] = tukey_weight((samples[k2] - med_dis) / (detar + 1e-5));
	}
	delete[] detars;
}