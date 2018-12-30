
#include "RobustEstimation.hpp"
#include "LinePoseEstimation.h"

bool bfirst = false;
int    maxIterNum = 6;                                 //最大迭代次数
double TerminateTh = 1e-5;                               //迭代终止条件

double distance_p2p(CvPoint2D32f p1, CvPoint2D32f p2) {
	float x1=p1.x;
	float x2=p2.x;
	float y1=p1.y;
	float y2=p2.y;
	return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

lineAttitudeDef::lineAttitudeDef()
{

}

lineAttitudeDef::lineAttitudeDef(double* para_oi)
{
	m_dfx           = para_oi[0];
	m_dfy           = para_oi[1];
	m_dcx           = para_oi[2];
	m_dcy           = para_oi[3];

	m_kmat          = (cv::Mat_<float>(3, 3)<< m_dfx,     0, m_dcx,
		                                           0, m_dfy, m_dcy,
												   0,     0,     1);
	m_kmat_inv      = m_kmat.inv();
	m_Lie_g[0] = cv::Mat::zeros(3, 3, CV_32F); m_Lie_g[0].at<float>(1, 2) = -1;  m_Lie_g[0].at<float>(2, 1) =  1;
	m_Lie_g[1] = cv::Mat::zeros(3, 3, CV_32F); m_Lie_g[1].at<float>(0, 2) =  1;  m_Lie_g[1].at<float>(2, 0) = -1;
	m_Lie_g[2] = cv::Mat::zeros(3, 3, CV_32F); m_Lie_g[2].at<float>(0, 1) = -1;  m_Lie_g[2].at<float>(1, 0) =  1;
	m_Vec_g[0] = cv::Mat::zeros(3, 1, CV_32F); m_Vec_g[0].at<float>(0, 0) =  1;
	m_Vec_g[1] = cv::Mat::zeros(3, 1, CV_32F); m_Vec_g[1].at<float>(1, 0) =  1; 
	m_Vec_g[2] = cv::Mat::zeros(3, 1, CV_32F); m_Vec_g[2].at<float>(2, 0) =  1;

	m_Lie_G[0] = cv::Mat::zeros(4, 4, CV_32F); m_Lie_G[0].at<float>(0, 3) = 1;
	m_Lie_G[1] = cv::Mat::zeros(4, 4, CV_32F); m_Lie_G[1].at<float>(1, 3) = 1;
	m_Lie_G[2] = cv::Mat::zeros(4, 4, CV_32F); m_Lie_G[2].at<float>(2, 3) = 1;
	m_Lie_G[3] = cv::Mat::zeros(4, 4, CV_32F); m_Lie_G[3].at<float>(1, 2) = -1;  m_Lie_G[3].at<float>(2, 1) = 1;
	m_Lie_G[4] = cv::Mat::zeros(4, 4, CV_32F); m_Lie_G[4].at<float>(0, 2) = 1;  m_Lie_G[4].at<float>(2, 0) = -1;
	m_Lie_G[5] = cv::Mat::zeros(4, 4, CV_32F); m_Lie_G[5].at<float>(0, 1) = -1;  m_Lie_G[5].at<float>(1, 0) = 1;
}
void lineAttitudeDef::init_camera_paras(double* para_oi)
{
	m_dfx           = para_oi[0];
	m_dfy           = para_oi[1];
	m_dcx           = para_oi[2];
	m_dcy           = para_oi[3];

	m_kmat          = (cv::Mat_<float>(3, 3)<< m_dfx,     0, m_dcx,
		0, m_dfy, m_dcy,
		0,     0,     1);
	m_kmat_inv      = m_kmat.inv();

	m_Lie_G[0] = cv::Mat::zeros(4, 4, CV_32F); m_Lie_G[0].at<float>(0, 3) = 1;
	m_Lie_G[1] = cv::Mat::zeros(4, 4, CV_32F); m_Lie_G[1].at<float>(1, 3) = 1;
	m_Lie_G[2] = cv::Mat::zeros(4, 4, CV_32F); m_Lie_G[2].at<float>(2, 3) = 1;
	m_Lie_G[3] = cv::Mat::zeros(4, 4, CV_32F); m_Lie_G[3].at<float>(1, 2) = -1;  m_Lie_G[3].at<float>(2, 1) = 1;
	m_Lie_G[4] = cv::Mat::zeros(4, 4, CV_32F); m_Lie_G[4].at<float>(0, 2) = 1;  m_Lie_G[4].at<float>(2, 0) = -1;
	m_Lie_G[5] = cv::Mat::zeros(4, 4, CV_32F); m_Lie_G[5].at<float>(0, 1) = -1;  m_Lie_G[5].at<float>(1, 0) = 1;
}
lineAttitudeDef::lineAttitudeDef(lineProjectDef* pLine_project)
{
	
	 set_camera_para(pLine_project);
}

void lineAttitudeDef::set_camera_para(lineProjectDef* pLine_project)
{
	m_pLine_project = pLine_project;
	m_kmat          = m_pLine_project->m_kmat;
	m_dfx           = m_pLine_project->m_fx;
	m_dfy           = m_pLine_project->m_fy;
	m_dcx           = m_pLine_project->m_cx;
	m_dcy           = m_pLine_project->m_cy;
	m_kmat_inv      = m_kmat.inv();
	m_mModel_lis    = m_pLine_project->m_mModel_Lis;
	m_mModel_pts    = m_pLine_project->m_mModel_Pts;
	m_mModel_pts_im = m_pLine_project->m_mModel_Pts_im;
	Line3D li;
	for(int i=0; i<m_mModel_lis.cols; i++)
	{
		int p1 = m_mModel_lis.at<float>(0, i);
		int p2 = m_mModel_lis.at<float>(1, i);
		Point3D pts[2];

		pts[0] = Point3D(m_mModel_pts.at<float>(0, p1), m_mModel_pts.at<float>(1, p1), m_mModel_pts.at<float>(2, p1));
		pts[1] = Point3D(m_mModel_pts.at<float>(0, p2), m_mModel_pts.at<float>(1, p2), m_mModel_pts.at<float>(2, p2));

		li = li.getLine3D(pts[0], pts[1]);
		m_sModel_lis.push_back(li);
	}
	m_Lie_g[0] = cv::Mat::zeros(3, 3, CV_32F); m_Lie_g[0].at<float>(1, 2) = -1;  m_Lie_g[0].at<float>(2, 1) =  1;
	m_Lie_g[1] = cv::Mat::zeros(3, 3, CV_32F); m_Lie_g[1].at<float>(0, 2) =  1;  m_Lie_g[1].at<float>(2, 0) = -1;
	m_Lie_g[2] = cv::Mat::zeros(3, 3, CV_32F); m_Lie_g[2].at<float>(0, 1) = -1;  m_Lie_g[2].at<float>(1, 0) =  1;

	m_Vec_g[0] = cv::Mat::zeros(3, 1, CV_32F); m_Vec_g[0].at<float>(0, 0) =  1;
	m_Vec_g[1] = cv::Mat::zeros(3, 1, CV_32F); m_Vec_g[1].at<float>(1, 0) =  1; 
	m_Vec_g[2] = cv::Mat::zeros(3, 1, CV_32F); m_Vec_g[2].at<float>(2, 0) =  1;

	m_Lie_G[0] = cv::Mat::zeros(4, 4, CV_32F); m_Lie_G[0].at<float>(0, 3) = 1;
	m_Lie_G[1] = cv::Mat::zeros(4, 4, CV_32F); m_Lie_G[1].at<float>(1, 3) = 1;
	m_Lie_G[2] = cv::Mat::zeros(4, 4, CV_32F); m_Lie_G[2].at<float>(2, 3) = 1;
	m_Lie_G[3] = cv::Mat::zeros(4, 4, CV_32F); m_Lie_G[3].at<float>(1, 2) = -1;  m_Lie_G[3].at<float>(2, 1) = 1;
	m_Lie_G[4] = cv::Mat::zeros(4, 4, CV_32F); m_Lie_G[4].at<float>(0, 2) = 1;  m_Lie_G[4].at<float>(2, 0) = -1;
	m_Lie_G[5] = cv::Mat::zeros(4, 4, CV_32F); m_Lie_G[5].at<float>(0, 1) = -1;  m_Lie_G[5].at<float>(1, 0) = 1;
}

lineAttitudeDef::~lineAttitudeDef(void)
{
	Lines_image->clear();
	Lines_image = NULL;
//	modelLines.clear();
//	imageLines.clear();


}

void   lineAttitudeDef::get_image_model_pairs(cv::Mat R, cv::Mat T, cv::Mat& pairs)
{	
	//模型直线同图像直线对应
	if (bfirst)
	{
		double disth = 8, angth = 8;
		m_pLine_project->motion_model_pts(R, T);  //计算三维平面参数及直线中点
		m_pLine_project->project_3d_model_pts();
		m_pLine_project->hidden_removel_lis();
		get_image_model_pairs(m_pLine_project->m_mModel_Pts_im, m_pLine_project->m_sLines_image, pairs, disth, angth);
		bfirst = false;
	}
}


//注意直线方向判断
void lineAttitudeDef::get_image_model_pairs(cv::Mat pt2ds, vector<Line>* li2ds, cv::Mat& pairs, double disTh, double angTh)
{
	extern vector<int> modelLines, imageLines;
	Line imgLine;
	double angbetweenlines = 0.0;
	vector<bool> flag((*li2ds).size(),false);
	IplImage* img = cvCreateImage(cvSize(m_nImg_width, m_nImg_height), 8, 3);
	cvZero(img);
	for (int i=0; i<m_mModel_lis.cols -12 ; i++)
	{
		//判断直线可见性
		if(!m_mModel_lis.at<float>(4, i))
			continue;
		Line li;         
		int p1 = m_mModel_lis.at<float>(0, i);
		int p2 = m_mModel_lis.at<float>(1, i);
		CvPoint2D32f pt1 = cvPoint2D32f(pt2ds.at<float>(0, p1), pt2ds.at<float>(1, p1));
		CvPoint2D32f pt2 = cvPoint2D32f(pt2ds.at<float>(0, p2), pt2ds.at<float>(1, p2));
		
		cvLine(img, cvPointFrom32f(pt1), cvPointFrom32f(pt2), CV_RGB(245, 220, 255), 1, 16);
		li = Line(pt1, pt2);   //这是模型直线投影后的直线(消隐后）
	
		//如果模型直线在图像上的投影不在图像范围内，continue
		if (((pt1.x<0)&&(pt2.x<0))||((pt1.x>m_nImg_width-1)&&(pt2.x>m_nImg_width-1))||
			((pt1.y<0)&&(pt2.y<0))||((pt1.y>m_nImg_height-1)&&(pt2.y>m_nImg_height-1)))
			continue;
		for (int c=0; c<3; c++)   
		{
			int ncount = 0;
			float maxlinelen = li.len/10;
			float mindis     = 10.0;
			int   num        = 0;
			vector<int> ind_lis;
			for (int j = 0; j < (*li2ds).size(); j++)
			{
				//直线间夹角
				if(flag[j])
					continue;
				imgLine         = (*li2ds)[j];
				angbetweenlines = fabs(imgLine.ang-li.ang);
				if (angbetweenlines > 90)
					angbetweenlines = 180-angbetweenlines;
				if (imgLine.len >  3*li.len)  //或者根据分类（垂翼的比例系数稍大，其他的需要比较小）
					continue;				
				if(angbetweenlines > angTh)
					continue;
				//直线端点距离
				//直线间中点距离
				float mid_dis = distance_p2p(li.midpoint, imgLine.midpoint);

				if(mid_dis > 1.4*(li.len+imgLine.len)/3.0)    //确定的依据是什么
					continue;
				//直线间距离
				float dislines = MIN(MIN(get_point_line_dis(li,imgLine.point1), get_point_line_dis(li,imgLine.point2)),
									 MIN(get_point_line_dis(imgLine,li.point1), get_point_line_dis(imgLine,li.point2)));
				//直线间距离小于一定阈值
				if ((dislines < (c+1)*disTh)&&(imgLine.len>maxlinelen))  //保留满足条件的最长直线
				{
					maxlinelen=imgLine.len;
					mindis = dislines;
					ncount++;
					num=j;
					ind_lis.push_back(j);
				}
			}
			if (ncount>0)
			{
				imageLines.push_back(num);
				modelLines.push_back(i);
				flag[num]=true;
				break;
			}
		}
	}
	pairs = cv::Mat::zeros(2, imageLines.size(), CV_32F);
	cvZero(img);

	for (int i=0; i<pairs.cols; i++)
	{
		pairs.at<float>(0, i) = modelLines[i];
		pairs.at<float>(1, i) = imageLines[i];		
	}
	
	cvReleaseImage(&img);
}
double lineAttitudeDef::get_point_line_dis(Line li,CvPoint2D32f pt)
{
	float lix=   li.par[0];
	float liy=   li.par[1];
	float liw=   li.par[2];
	double dis=   fabs(pt.y*lix + pt.x*liy + liw);
	return dis;
}


bool   lineAttitudeDef::R_and_T_Methods(cv::Mat& R, cv::Mat& T, int nmethod)
{
	bool bSucceed = false;
	bfirst = true;
	cv::Mat pairs;
	switch (nmethod)
	{
	case 1:      //
		Lines_image = m_pLine_project->m_sLines_image;
		bSucceed = get_pose_lines_R_and_T_Lie(m_pLine_project->m_sLines_image, pairs, R, T);
		break;
	case 2:      //
		Lines_image = m_pLine_project->m_sLines_image;
		bSucceed = R_and_T_Mod_Novel(m_pLine_project->m_sLines_image, pairs, R, T);
		break;
	case 3:      //
		bSucceed = R_and_T_Mod_Novel_length(m_pLine_project->m_sLines_image, pairs, R, T);
		break;
	default:
		break;
	}
	return bSucceed;
}

bool   lineAttitudeDef::get_optimal_pose(IplImage* pimg, cv::Mat& R, cv::Mat& T, double nmethod )
{
	bool bSuccess = true;
	m_nImg_width  = pimg->width;
	m_nImg_height = pimg->height;
	bSuccess = R_and_T_Methods(R, T, nmethod);
	return bSuccess;
}

//R T为世界系到像机系变换的参数
bool  lineAttitudeDef::R_and_T_Mod_Novel(vector<Line>* li2ds, cv::Mat lipairs, cv::Mat& R, cv::Mat& T)
{
	//直线对应建立
	bool bsuccess = false;     
	//中间变量
	cv::Mat JmatT, JTWJmat, JTWEmat;      
	cv::Mat p1, p0, p2, P1, P2, P1c, P2c, Ni, Nk, Wk;
	cv::Mat U, W, VT; 
	cv::Mat RT = R.t();
	cv::Mat Tw = -RT*T;                //相机系原点在世界系坐标
	double fx2 = m_dfx*m_dfx;
	double fy2 = m_dfy*m_dfy;
	for (int iter=0; iter<maxIterNum; iter++)
	{
		get_image_model_pairs(R, T, lipairs);
		int  npairs   = lipairs.cols;
		if (npairs<3)
		{
			return false;
		}
		Line3D* lines0 = new Line3D[lipairs.cols]; 
		Line3D* lines1 = new Line3D[lipairs.cols]; 
		for(int i=0; i<lipairs.cols; i++)
		{
			int l0 = lipairs.at<float>(0, i);
			int l1 = lipairs.at<float>(1, i);

			cv::Mat p11 = (cv::Mat_<float>(3, 1)<<(*li2ds)[l1].point1.x, (*li2ds)[l1].point1.y, 1);
			cv::Mat p22 = (cv::Mat_<float>(3, 1)<<(*li2ds)[l1].point2.x, (*li2ds)[l1].point2.y, 1);
			//归一化像点坐标
			p11 = m_kmat_inv*p11;
			p22 = m_kmat_inv*p22;

			Point3D p1 = Point3D(p11.at<float>(0, 0), p11.at<float>(1, 0), p11.at<float>(2, 0));
			Point3D p2 = Point3D(p22.at<float>(0, 0), p22.at<float>(1, 0), p22.at<float>(2, 0));
			//模型直线序列
			lines0[i] = m_sModel_lis[l0];
			//图像直线序列
			Line3D li =li.getLine3D(p1, p2);
			lines1[i] = li;
		}
		//测量残差
		double* weights   = new double[npairs*3];
		memset(weights,   0, sizeof(double)*npairs*3);
		int    count       = 0;
		//int    maxIterNum  = 10;                                  //最大迭代次数
		//double TerminateTh = 1e-5;                               //迭代终止条件
		cv::Mat Jmat = cv::Mat::zeros(npairs*3, 6, CV_32F);      //雅可比矩阵
		cv::Mat Emat = cv::Mat::zeros(npairs*3, 1, CV_32F);      //测量矩阵
		cv::Mat para = cv::Mat::zeros(6, 1, CV_32F);             //参数矩阵
		cv::Mat Wmat = cv::Mat::eye(npairs*3, npairs*3, CV_32F); //权值矩阵，m-estimator             
		RT = R.t();
		//求解雅克比矩阵
		for (int k=0; k<npairs; k++)
		{
			//计算雅可比矩阵
			//for first point on line
			p1 = (cv::Mat_<float>(3, 1)<<lines1[k].p1.at<float>(0, 0), lines1[k].p1.at<float>(1, 0), 
				lines1[k].p1.at<float>(2, 0));
			p2 = (cv::Mat_<float>(3, 1)<<lines1[k].p2.at<float>(0, 0), lines1[k].p2.at<float>(1, 0), 
				lines1[k].p2.at<float>(2, 0));
			p0 = (p1+p2)/2.0;
			P1 = (cv::Mat_<float>(3, 1)<<lines0[k].p1.at<float>(0, 0), lines0[k].p1.at<float>(1, 0), 
				lines0[k].p1.at<float>(2, 0));
			P2 = (cv::Mat_<float>(3, 1)<<lines0[k].p2.at<float>(0, 0), lines0[k].p2.at<float>(1, 0), 
				lines0[k].p2.at<float>(2, 0));
			//变换到相机系
			P1  = P1 - Tw;
			P2  = P2 - Tw;
			P1c = R*P1;
			P2c = R*P2;
			//投影平面
			Ni = P1c.cross(P2c);
			double Nx = Ni.at<float>(0, 0), Ny = Ni.at<float>(1, 0), Nz = Ni.at<float>(2, 0); 
			//分母Mt
			double Mt = sqrt(Nx*Nx/fx2+Ny*Ny/fy2);
			double Mt_inv = 1/(Mt+1e-7);
			//计算残差
			//why加个负号
			Emat.at<float>(3*k, 0)   = -Mt_inv*p1.dot(Ni);
			Emat.at<float>(3*k+1, 0) = -Mt_inv*p2.dot(Ni);
			Emat.at<float>(3*k+2, 0) = -Mt_inv*p0.dot(Ni);
			//赋给残差矩阵
			weights[3*k]   = Emat.at<float>(3*k, 0);
			weights[3*k+1] = Emat.at<float>(3*k+1, 0);
			weights[3*k+2] = Emat.at<float>(3*k+2, 0);
			//分别对w,t求雅克比矩阵
			//平移向量求取雅克比矩阵
			for (int g=0; g<3; g++)
			{
				Nk = m_Lie_g[g]*(P1-P2);
				//第一个端点
				Wk = RT*p1;
				Jmat.at<float>(3*k, g)   = Mt_inv*Wk.dot(Nk);
				//第二个端点
				Wk = RT*p2;
				Jmat.at<float>(3*k+1, g) = Mt_inv*Wk.dot(Nk);
				//中点
				Wk = RT*p0;
				Jmat.at<float>(3*k+2, g) = Mt_inv*Wk.dot(Nk);
			}
			//旋转矩阵指数参数求解雅克比矩阵
			//R(t+1)=exp(w)*R
			for (int g=0; g<3; g++)
			{
				Nk = P1.cross(P2);
				//第一个端点
				Wk = RT*m_Lie_g[g]*p1;
				Jmat.at<float>(3*k, g+3)   = -Mt_inv*Wk.dot(Nk);
				//第二个端点
				Wk = RT*m_Lie_g[g]*p2;
				Jmat.at<float>(3*k+1, g+3) = -Mt_inv*Wk.dot(Nk);
				//中点
				Wk = RT*m_Lie_g[g]*p0;
				Jmat.at<float>(3*k+2, g+3) = -Mt_inv*Wk.dot(Nk);
			}
		}
		//构建权值矩阵
		get_weights(weights, npairs*3);
		//加权
		for (int k2=0; k2<npairs*3; k2++)
		{
			Wmat.at<float>(k2, k2) = weights[k2]/double(2.45);
			if(k2%3==2)
			{
				Wmat.at<float>(k2, k2) = 4*Wmat.at<float>(k2, k2);
			}
		}
		//计算位姿更新量
		JmatT = Jmat.t();
		JmatT = JmatT*Wmat;
		JTWJmat = JmatT*Jmat;        //JTWJ= JT*W*J
		JTWEmat = JmatT*Emat;        //JTWE= JT*W*E
		cv::solve(JTWJmat, JTWEmat, para, DECOMP_LU);
		cv::Mat dT     = para.rowRange(0, 3);  
		cv::Mat dOmiga = para.rowRange(3, 6);   
		cv::Mat R_datar = (cv::Mat_<float>(3, 3)<<1,                      -dOmiga.at<float>(2, 0),   dOmiga.at<float>(1, 0),
			dOmiga.at<float>(2, 0),                        1,  -dOmiga.at<float>(0, 0),
			-dOmiga.at<float>(1, 0),   dOmiga.at<float>(0, 0),                       1);
		R = R_datar*R; 
		Tw = Tw + dT;
		//旋转矩阵正交化
		cv::SVDecomp(R, W, U, VT, CV_SVD_V_T);
		R = U * VT;
		count++;
		//判断收敛条件
		double dt_norm = cv::norm(dT);
		double dr_norm = cv::norm(dOmiga);
		T = -R*Tw;
		delete[] weights;
		delete[] lines0;
		delete[] lines1;
		if((dt_norm<TerminateTh) && (dr_norm<0.1*TerminateTh))
		{
			cout<<iter<<endl;
			bsuccess =  true;
			//break;
		}		
	}
	return bsuccess;
}

bool  lineAttitudeDef::R_and_T_Mod_Novel_length(vector<Line>* li2ds, cv::Mat lipairs, cv::Mat& R, cv::Mat& T)
{
	//直线对应建立
	bool bsuccess = false;      
	//中间变量
	cv::Mat JmatT, JTWJmat, JTWEmat;      
	cv::Mat p1, p0, p2, P1, P2, P1c, P2c, Ni, Nk, Wk;
	cv::Mat U, W, VT; 
	cv::Mat RT = R.t();
	cv::Mat Tw = -RT*T;                //相机系原点在世界系坐标
	double fx2 = m_dfx*m_dfx;
	double fy2 = m_dfy*m_dfy;
	for (int iter=0; iter<maxIterNum; iter++)
	{
		get_image_model_pairs(R, T, lipairs);
		if (lipairs.cols<3)
		{
			return false;
		}
		int  npairs   = lipairs.cols;
		Line3D* lines0 = new Line3D[lipairs.cols]; 
		Line3D* lines1 = new Line3D[lipairs.cols]; 
		for(int i=0; i<lipairs.cols; i++)
		{
			int l0 = lipairs.at<float>(0, i);
			int l1 = lipairs.at<float>(1, i);

			cv::Mat p11 = (cv::Mat_<float>(3, 1)<<(*li2ds)[l1].point1.x, (*li2ds)[l1].point1.y, 1);
			cv::Mat p22 = (cv::Mat_<float>(3, 1)<<(*li2ds)[l1].point2.x, (*li2ds)[l1].point2.y, 1);
			//归一化像点坐标
			p11 = m_kmat_inv*p11;
			p22 = m_kmat_inv*p22;

			Point3D p1 = Point3D(p11.at<float>(0, 0), p11.at<float>(1, 0), p11.at<float>(2, 0));
			Point3D p2 = Point3D(p22.at<float>(0, 0), p22.at<float>(1, 0), p22.at<float>(2, 0));
			//模型直线序列
			lines0[i] = m_sModel_lis[l0];
			//图像直线序列
			Line3D li =li.getLine3D(p1, p2);
			lines1[i] = li;
		}
		//测量残差
		double* weights   = new double[npairs*3];
		memset(weights,   0, sizeof(double)*npairs*3);
		int    count       = 0;
		cv::Mat Jmat = cv::Mat::zeros(npairs*3, 6, CV_32F);      //雅可比矩阵
		cv::Mat Emat = cv::Mat::zeros(npairs*3, 1, CV_32F);      //测量矩阵
		cv::Mat para = cv::Mat::zeros(6, 1, CV_32F);             //参数矩阵
		cv::Mat Wmat = cv::Mat::eye(npairs*3, npairs*3, CV_32F); //权值矩阵，m-estimator            
		RT = R.t();
		//求解雅克比矩阵
		for (int k=0; k<npairs; k++)
		{
			//计算雅可比矩阵
			p1 = (cv::Mat_<float>(3, 1)<<lines1[k].p1.at<float>(0, 0), lines1[k].p1.at<float>(1, 0), 
				lines1[k].p1.at<float>(2, 0));
			p2 = (cv::Mat_<float>(3, 1)<<lines1[k].p2.at<float>(0, 0), lines1[k].p2.at<float>(1, 0), 
				lines1[k].p2.at<float>(2, 0));
			p0 = (p1+p2)/2.0;
			P1 = (cv::Mat_<float>(3, 1)<<lines0[k].p1.at<float>(0, 0), lines0[k].p1.at<float>(1, 0), 
				lines0[k].p1.at<float>(2, 0));
			P2 = (cv::Mat_<float>(3, 1)<<lines0[k].p2.at<float>(0, 0), lines0[k].p2.at<float>(1, 0), 
				lines0[k].p2.at<float>(2, 0));
			//变换到相机系
			P1  = P1 - Tw;
			P2  = P2 - Tw;
			P1c = R*P1;
			P2c = R*P2;
			//投影平面
			Ni = P1c.cross(P2c);
			double Nx = Ni.at<float>(0, 0), Ny = Ni.at<float>(1, 0), Nz = Ni.at<float>(2, 0); 
			//分母Mt
			double Mt = sqrt(Nx*Nx/fx2+Ny*Ny/fy2);
			double Mt_inv = 1/(Mt+1e-7);
			//计算残差
			Emat.at<float>(3*k, 0)   = -Mt_inv*p1.dot(Ni);
			Emat.at<float>(3*k+1, 0) = -Mt_inv*p2.dot(Ni);
			Emat.at<float>(3*k+2, 0) = -Mt_inv*p0.dot(Ni);
			//赋给残差矩阵
			weights[3*k]   = Emat.at<float>(3*k, 0);
			weights[3*k+1] = Emat.at<float>(3*k+1, 0);
			weights[3*k+2] = Emat.at<float>(3*k+2, 0);
			//分别对w,t求雅克比矩阵
			//平移向量求取雅克比矩阵
			for (int g=0; g<3; g++)
			{
				Nk = m_Lie_g[g]*(P1-P2);
				//第一个端点
				Wk = RT*p1;
				Jmat.at<float>(3*k, g)   = Mt_inv*Wk.dot(Nk);
				//第二个端点
				Wk = RT*p2;
				Jmat.at<float>(3*k+1, g) = Mt_inv*Wk.dot(Nk);
				//中点
				Wk = RT*p0;
				Jmat.at<float>(3*k+2, g) = Mt_inv*Wk.dot(Nk);
			}
			//旋转矩阵指数参数求解雅克比矩阵
			//R(t+1)=exp(w)*R
			for (int g=0; g<3; g++)
			{
				Nk = P1.cross(P2);
				//第一个端点
				Wk = RT*m_Lie_g[g]*p1;
				Jmat.at<float>(3*k, g+3)   = -Mt_inv*Wk.dot(Nk);
				//第二个端点
				Wk = RT*m_Lie_g[g]*p2;
				Jmat.at<float>(3*k+1, g+3) = -Mt_inv*Wk.dot(Nk);
				//中点
				Wk = RT*m_Lie_g[g]*p0;
				Jmat.at<float>(3*k+2, g+3) = -Mt_inv*Wk.dot(Nk);
			}
		}
		//构建权值矩阵
		get_weights(weights, npairs*3);
		//加权
		for (int k2=0; k2<npairs*3; k2++)
		{
			double len = lines1[k2/3].len;
			len = sqrt(len);
			Wmat.at<float>(k2, k2) = weights[k2]*len/double(2.45);
			if(k2%3==2)
			{
				Wmat.at<float>(k2, k2) = 4*Wmat.at<float>(k2, k2);
			}
		}
		//计算位姿更新量
		JmatT = Jmat.t();
		JmatT = JmatT*Wmat;
		JTWJmat = JmatT*Jmat;        //JTWJ= JT*W*J
		JTWEmat = JmatT*Emat;        //JTWE= JT*W*E
		cv::solve(JTWJmat, JTWEmat, para, DECOMP_LU);
		cv::Mat dT     = para.rowRange(0, 3);  
		cv::Mat dOmiga = para.rowRange(3, 6);   
		cv::Mat R_datar = (cv::Mat_<float>(3, 3)<<1,                      -dOmiga.at<float>(2, 0),   dOmiga.at<float>(1, 0),
			dOmiga.at<float>(2, 0),                        1,  -dOmiga.at<float>(0, 0),
			-dOmiga.at<float>(1, 0),   dOmiga.at<float>(0, 0),                       1);
		R = R_datar*R; 
		Tw = Tw + dT;
		//旋转矩阵正交化
		cv::SVDecomp(R, W, U, VT, CV_SVD_V_T);
		R = U * VT;
		T = -R*Tw;
		count++;
		//判断收敛条件
		double dt_norm = cv::norm(dT);
		double dr_norm = cv::norm(dOmiga);

		if((dt_norm<TerminateTh) && (dr_norm<0.1*TerminateTh))
		{
			if (iter == 0) 
				cout<<"weight:"<<Wmat.diag()<<endl;
			bsuccess =  true;
			break;
		}
		delete[] weights;
		delete[] lines0;
		delete[] lines1;

	}
	return bsuccess;
}


bool  lineAttitudeDef::get_pose_lines_R_and_T_Lie(vector<Line>* li2ds, cv::Mat lipairs, cv::Mat& R, cv::Mat& T)
{
	//直线对应建立
	get_image_model_pairs(R, T, lipairs);
	int npairs = lipairs.cols;
	for (int iter = 0; iter < maxIterNum; iter++)
	{
		cv::Mat motion_mat = cv::Mat::zeros(4, 4, CV_32F);
		cv::Mat motion_mat_detar = cv::Mat::eye(4, 4, CV_32F);
		//投影矩阵
		motion_mat.at<float>(0, 0) = R.at<float>(0, 0); motion_mat.at<float>(0, 1) = R.at<float>(0, 1); motion_mat.at<float>(0, 2) = R.at<float>(0, 2); motion_mat.at<float>(0, 3) = T.at<float>(0, 0);
		motion_mat.at<float>(1, 0) = R.at<float>(1, 0); motion_mat.at<float>(1, 1) = R.at<float>(1, 1); motion_mat.at<float>(1, 2) = R.at<float>(1, 2); motion_mat.at<float>(1, 3) = T.at<float>(1, 0);
		motion_mat.at<float>(2, 0) = R.at<float>(2, 0); motion_mat.at<float>(2, 1) = R.at<float>(2, 1); motion_mat.at<float>(2, 2) = R.at<float>(2, 2); motion_mat.at<float>(2, 3) = T.at<float>(2, 0);
		motion_mat.at<float>(3, 0) = 0; motion_mat.at<float>(3, 1) = 0; motion_mat.at<float>(3, 2) = 0; motion_mat.at<float>(3, 3) = 1;
		//空间直线
		Line3D* lines0 = new Line3D[npairs];
		cv::Mat Nc = cv::Mat::zeros(4, npairs, CV_32F);
		double a, b, c;
		//计算投影平面法向量
		for (int i = 0; i<npairs; i++)
		{
			int l0 = lipairs.at<float>(0, i);
			int l1 = lipairs.at<float>(1, i);
			//图像直线端点
			CvPoint2D32f img_pt1 = (*li2ds)[l1].point1;
			CvPoint2D32f img_pt2 = (*li2ds)[l1].point2;
			//图像坐标系原点定义在主点
			img_pt1.x = img_pt1.x - m_dcx; img_pt1.y = img_pt1.y - m_dcy;
			img_pt2.x = img_pt2.x - m_dcx; img_pt2.y = img_pt2.y - m_dcy;
			//直线参数，cos, sin， p
			double x1 = img_pt1.x, x2 = img_pt2.x, y1 = img_pt1.y, y2 = img_pt2.y;
			a = (y1 - y2);
			b = (x2 - x1);
			c = x1*y2 - x2*y1;
			double norm = sqrt(a*a + b*b);
			a = m_dfx*a / (norm + 1e-5);   //fxcos
			b = m_dfy*b / (norm + 1e-5);   //fysin
			c = c / (norm + 1e-5);         //-p
			//归一化为单位向量,R_and_T
			norm = sqrt(a*a + b*b + c*c);
			a = a / (norm + 1e-5);
			b = b / (norm + 1e-5);
			c = c / (norm + 1e-5);

			Nc.at<float>(0, i) = a; Nc.at<float>(1, i) = b; Nc.at<float>(2, i) = c; Nc.at<float>(3, i) = 0;
			//模型直线序列
			lines0[i] = m_sModel_lis[l0];
		}
		//迭代优化R,T R and T
		//测量残差
		double* weights = new double[npairs * 2];
		memset(weights, 0, sizeof(double)*npairs * 2);
		cv::Mat Jmat = cv::Mat::zeros(npairs * 2, 6, CV_32F);      //雅可比矩阵
		cv::Mat Emat = cv::Mat::zeros(npairs * 2, 1, CV_32F);      //测量矩阵
		cv::Mat para = cv::Mat::zeros(6, 1, CV_32F);             //参数矩阵
		cv::Mat Wmat = cv::Mat::eye(npairs * 2, npairs * 2, CV_32F); //权值矩阵，m-estimator                  
		cv::Mat JmatT, JTWJmat, JTWEmat;
		cv::Mat P1, P2, Pci, Ni;
		motion_mat_detar = cv::Mat::eye(4, 4, CV_32F);
		for (int k = 0; k<npairs; k++)
		{
		//计算雅可比矩阵
			P1 = (cv::Mat_<float>(4, 1) << lines0[k].p1.at<float>(0, 0), lines0[k].p1.at<float>(1, 0),
					lines0[k].p1.at<float>(2, 0), 1);
			P2 = (cv::Mat_<float>(4, 1) << lines0[k].p2.at<float>(0, 0), lines0[k].p2.at<float>(1, 0),
					lines0[k].p2.at<float>(2, 0), 1);

			Ni = Nc.col(k);
			P1 = motion_mat*P1;
			P2 = motion_mat*P2;
			Emat.at<float>(2 * k, 0) = Ni.dot(P1);
			Emat.at<float>(2 * k + 1, 0) = Ni.dot(P2);
			weights[2 * k] = Emat.at<float>(2 * k, 0);
			weights[2 * k + 1] = Emat.at<float>(2 * k + 1, 0);
			for (int g = 0; g<6; g++)
			{
				Pci = m_Lie_G[g] * P1;
				Jmat.at<float>(2 * k, g) = Ni.dot(Pci);
				Pci = m_Lie_G[g] * P2;
				Jmat.at<float>(2 * k + 1, g) = Ni.dot(Pci);
			}
		}
		//构建权值矩阵
		get_weights(weights, npairs * 2);
		for (int k2 = 0; k2<npairs * 2; k2++)
		{
			Wmat.at<float>(k2, k2) = weights[k2];
		}
		JmatT = Jmat.t();
		JmatT = JmatT*Wmat;
		JTWJmat = JmatT*Jmat;        //JTWJ= JT*W*J
		JTWEmat = JmatT*Emat;        //JTWE= JT*W*E
		cv::solve(JTWJmat, JTWEmat, para, DECOMP_LU);
		for (int g = 0; g<6; g++)
		{
			motion_mat_detar = motion_mat_detar - para.at<float>(g, 0)*m_Lie_G[g];
		}
		motion_mat = motion_mat_detar*motion_mat;
		cv::Mat dT = (cv::Mat_<float>(3, 1) << para.at<float>(0, 0), para.at<float>(1, 0), para.at<float>(2, 0));
		cv::Mat dOmiga = (cv::Mat_<float>(3, 1) << para.at<float>(3, 0), para.at<float>(4, 0), para.at<float>(5, 0));

		double dt_norm = cv::norm(dT);
		double dr_norm = cv::norm(dOmiga);
		//cout << "dt_norm" << dt_norm << endl << "dr_norm" << dr_norm << endl;
		double m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33;

		m00 = motion_mat.at<float>(0, 0); m01 = motion_mat.at<float>(0, 1); m02 = motion_mat.at<float>(0, 2); m03 = motion_mat.at<float>(0, 3);
		m10 = motion_mat.at<float>(1, 0); m11 = motion_mat.at<float>(1, 1); m12 = motion_mat.at<float>(1, 2); m13 = motion_mat.at<float>(1, 3);
		m20 = motion_mat.at<float>(2, 0); m21 = motion_mat.at<float>(2, 1); m22 = motion_mat.at<float>(2, 2); m23 = motion_mat.at<float>(2, 3);
		m30 = motion_mat.at<float>(3, 0); m31 = motion_mat.at<float>(3, 1); m32 = motion_mat.at<float>(3, 2); m33 = motion_mat.at<float>(3, 3);

		//cout << "R" << R << "T" << T << endl;
		R = (cv::Mat_<float>(3, 3) << m00, m01, m02,
			m10, m11, m12,
			m20, m21, m22);
		T = (cv::Mat_<float>(3, 1) << m03, m13, m23);
		R = R / m33;
		T = T / m33;
		
		cv::Mat U, D, W, VT;
		cv::SVDecomp(R, W, U, VT, CV_SVD_V_T);
		R = U * VT;
		//cout << "After Iter" << "R" << R << "T" << T << endl;
		if ((dt_norm<10 * TerminateTh) && (dr_norm<0.1*TerminateTh))
		{
			return true;
		}
		delete[] weights;
		delete[] lines0;	
	}
	return true;
}
