#include "RPnP.h"

void LookMat(Mat& mat, char* str)
{
	printf("%s is:\n",str);
	for (int i=0;i<mat.rows;i++)
	{
		for (int j=0;j<mat.cols;j++)
			printf("%.20f\t",mat.at<double>(i,j));
		printf("\n");
	}
	printf("\n\n");
}

void Lookvalue(double& val,char* str)
{
	printf("%s is:\n",str);
	printf("%.20f\n",val);
}

void calcampose(Mat& XXc,Mat&XXw,Mat& foundR, Mat& foundT)
{
	int n=XXc.cols;
	Mat_<double> X=XXw;
	Mat_<double> Y=XXc;
	Mat_<double> K=Mat::eye(cvSize(n,n),CV_64F)-Mat::ones(cvSize(n,n),CV_64F)/n;

	Mat_<double> ux(3,1,CV_64F);
	Scalar mssim = mean(X.row(0));
	ux.at<double>(0,0)=mssim(0);
	mssim = mean(X.row(1));
	ux.at<double>(1,0)=mssim(0);
	mssim = mean(X.row(2));
	ux.at<double>(2,0)=mssim(0);

	Mat_<double> uy(3,1,CV_64F);
	mssim = mean(Y.row(0));
	uy.at<double>(0,0)=mssim(0);
	mssim = mean(Y.row(1));
	uy.at<double>(1,0)=mssim(0);
	mssim = mean(Y.row(2));
	uy.at<double>(2,0)=mssim(0);

	Mat_<double> tmpxk=X*K;
	reduce(tmpxk.mul(tmpxk),tmpxk,0,CV_REDUCE_SUM);
	double sigmax2;
	mssim = mean(tmpxk);
	sigmax2=mssim(0);
	Mat_<double> SXY=Y*K*X.t()/n;

	Mat U,D,V;
	SVD::compute(SXY,D, U, V,SVD::FULL_UV);
	U.col(0)=U.col(0)*-1;
	U.col(2)=U.col(2)*-1;
	V=V.t();
	V.col(0)=V.col(0)*-1;
	V.col(2)=V.col(2)*-1;

	Mat_<double> S=Mat::eye(cvSize(3,3),CV_64F);
	if (determinant(SXY)<0)
	{
		S.at<double>(2,2)=-1;
	}
	foundR=U*S*V.t();
	Mat_<double> convertD=Mat::zeros(cvSize(3,3),CV_64F);
	convertD.at<double>(0,0)=D.at<double>(0,0);
	convertD.at<double>(1,1)=D.at<double>(1,0);
	convertD.at<double>(2,2)=D.at<double>(2,0);
	mssim=trace(convertD*S);
	double c2=mssim(0)/sigmax2;
	foundT=uy-c2*foundR*ux;
	if (norm(foundR.col(0).cross(foundR.col(1))-foundR.col(2))>2e-2)
	{
		foundR.col(2)=foundR.col(2)*-1;
	}
}

bool RPnP(std::vector<Point3d>& WorldPts, std::vector<Point2d>& Imgpts, Mat& rvec, Mat& tvec, Point2d& PrincipalPoint, double focallength)
{
	int n=Imgpts.size();
	Mat_<double> xxv=Mat::ones(3,n,CV_64F);
	Mat_<double> xx=Mat(Imgpts).reshape(1).t();

  xx.row(0)=(xx.row(0)-PrincipalPoint.x)/510;
  xx.row(1)=(xx.row(1)-PrincipalPoint.y)/546;


	xx.copyTo(xxv(Range(0,2),Range::all()));
	Mat_<double> world3pts=Mat(WorldPts).reshape(1).t();
	Mat_<double> XXw=world3pts.clone();
	for (int i=0;i<xxv.cols;i++)
		xxv.col(i)=xxv.col(i)/norm(xxv.col(i));

	int i1=0;
	int i2=1;
	double lmin=xxv.col(i1).dot(xxv.col(i2));
	RNG rng;
	Mat_<double> rij=Mat(n,2,CV_64F);
	for (int j=0;j<rij.rows;j++)
		for (int k=0;k<rij.cols;k++)
			rij.at<double>(j,k)=ceil(rng.uniform(0.0,1.0)*n);
	int a,b;
	double l;
	for (int j=0;j<rij.rows;j++)
	{
		a=ceil(rij.at<double>(j,0));
		b=ceil(rij.at<double>(j,1));
		if (fabsf(a-b)<1e-5)
			continue;
		l=xxv.col(a-1).dot(xxv.col(b-1));
		if (l<lmin)
		{
			i1=a-1;
			i2=b-1;
			lmin=l;
		}
	}

	Mat_<double> p1=world3pts.col(i1);
	Mat_<double> p2=world3pts.col(i2);
	Mat_<double> p0=(p1+p2)*0.5;
	Mat_<double> x=p2-p0;
	x=x/norm(x);
	Mat_<double> a_y = (Mat_<double>(3,1) << 0.0, 1.0, 0.0);
	Mat_<double> a_z = (Mat_<double>(3,1) << 0.0, 0.0, 1.0);
	Mat_<double> y,z;

	if (fabsf(a_y.dot(x))<fabsf(a_z.dot(x)))
	{
		z=x.cross(a_y);		z=z/norm(z);
		y=z.cross(x);		y=y/norm(y);
	}
	else
	{
		y=a_z.cross(x);		y=y/norm(y);
		z=x.cross(y);		z=z/norm(z);
	}
	
	Mat_<double> R0(3,3,CV_64F);
	x.copyTo(R0.col(0));
	y.copyTo(R0.col(1));
	z.copyTo(R0.col(2));
	world3pts.row(0)=world3pts.row(0)-p0.at<double>(0,0);
	world3pts.row(1)=world3pts.row(1)-p0.at<double>(1,0);
	world3pts.row(2)=world3pts.row(2)-p0.at<double>(2,0);
	world3pts=R0.t()*world3pts;

	Mat_<double> v1=xxv.col(i1);
	Mat_<double> v2=xxv.col(i2);
	double cg1=v1.dot(v2);
	double sg1=sqrt(1-cg1*cg1);
	double D1=norm(world3pts.col(i1)-world3pts.col(i2));
	Mat_<double> D4=Mat::zeros(n-2,5,CV_64F);

	Mat idex=Mat::ones(1,n,CV_32F);
	idex.at<float>(0,i1)=0;	idex.at<float>(0,i2)=0;
	Mat_<double> vi(3,n-2,CV_64F);
	Mat_<double> didx(1,n-2,CV_64F);

	int myindex=0;
	for (int i=0;i<n;i++)
		if (idex.at<float>(0,i)==1)
			{
				xxv.col(i).copyTo(vi.col(myindex));
				didx.at<double>(0,myindex++)=i;
			}
	Mat_<double> cg2=vi.t()*v1;
	Mat_<double> cg3=vi.t()*v2;
	Mat_<double> sg2=cg2.clone();
	Mat_<double> D2=cg2.clone();
	Mat_<double> D3=cg2.clone();

	for (int i=0;i<sg2.rows;i++)
		for (int j=0;j<sg2.cols;j++)
			sg2.at<double>(i,j)=sqrt(1-cg2.at<double>(i,j)*cg2.at<double>(i,j));
	for (int i=0;i<n-2;i++)
	{
		D2.at<double>(i,0)=norm(world3pts.col(i1)-world3pts.col(didx.at<double>(0,i)));
		D3.at<double>(i,0)=norm(world3pts.col(didx.at<double>(0,i))-world3pts.col(i2));
	}
	Mat_<double> A1=D2.mul(D2/(D1*D1));
	Mat_<double> A2=A1*sg1*sg1-sg2.mul(sg2);
	Mat_<double> A3=cg2.mul(cg3)-cg1;
	Mat_<double> A4=cg1*cg3-cg2;
	Mat_<double> A6=(D3.mul(D3)-D1*D1-D2.mul(D2))/(2*D1*D1);
	Mat_<double> A7=1-cg1*cg1-cg2.mul(cg2)+cg1*cg2.mul(cg3)+A6*sg1*sg1;
	D4.col(0)=A6.mul(A6)-A1.mul(cg3.mul(cg3));
	D4.col(1)=2*(A3.mul(A6)-A1.mul(A4.mul(cg3)));
	D4.col(2)=A3.mul(A3)+2*A6.mul(A7)-A1.mul(A4.mul(A4))-A2.mul(cg3.mul(cg3));
	D4.col(3)=2*(A3.mul(A7)-A2.mul(A4.mul(cg3)));
	D4.col(4)=A7.mul(A7)-A2.mul(A4.mul(A4));

	Mat_<double> F7=Mat::zeros(n-2,8,CV_64F);
	F7.col(0)=4*D4.col(0).mul(D4.col(0));
	F7.col(1)=7*D4.col(1).mul(D4.col(0));
	F7.col(2)=6*D4.col(2).mul(D4.col(0))+3*D4.col(1).mul(D4.col(1));
	F7.col(3)=5*D4.col(3).mul(D4.col(0))+5*D4.col(2).mul(D4.col(1));
	F7.col(4)=4*D4.col(4).mul(D4.col(0))+4*D4.col(3).mul(D4.col(1))+2*D4.col(2).mul(D4.col(2));
	F7.col(5)=3*D4.col(4).mul(D4.col(1))+3*D4.col(3).mul(D4.col(2));
	F7.col(6)=2*D4.col(4).mul(D4.col(2))+D4.col(3).mul(D4.col(3));
	F7.col(7)=D4.col(4).mul(D4.col(3));
	Mat_<double> D7,tmpD7;
	reduce(F7,D7,0,CV_REDUCE_SUM);
	tmpD7=D7.clone();
	double invtmp;
	double tmpfirst=tmpD7.at<double>(0,0);
	if (tmpfirst<1e-20)
  {  cout<<"tmpfitst"<<tmpfirst<<endl;
		return false;
  }
	for (int i=0;i<tmpD7.cols/2;i++)
	{
		invtmp=tmpD7.at<double>(0,i);
		tmpD7.at<double>(0,i)=tmpD7.at<double>(0,tmpD7.cols-1-i)/tmpfirst;
		tmpD7.at<double>(0,tmpD7.cols-1-i)=invtmp/tmpfirst;
	}
	Mat tmpt2s;
	solvePoly(tmpD7,tmpt2s);
	double maxreal=0;
	for (int i=0;i<tmpt2s.rows;i++)
	{
		cv::Complexd tmpcomplex=tmpt2s.at<cv::Complexd>(i,0);
		//fprintf(fp,"re is %.20f\t,img is %.20f\n",tmpcomplex.re,tmpcomplex.im); //²é¿ŽÇóœâœá¹û
		double tmpre=fabsf(tmpcomplex.re);
		if (tmpre>maxreal)
			maxreal=tmpre;
	}

	std::vector<double> realt2s;
	for (int i=0;i<tmpt2s.rows;i++)
	{
		cv::Complexd tmp=tmpt2s.at<cv::Complexd>(i,0);
		if (fabsf(tmp.im)/maxreal<=0.001)
			realt2s.push_back(tmp.re);
	}
	Mat_<double> t2s(realt2s);
	std::vector<double> calrealt2s;
	Mat_<double> ForD6=(Mat_<double>(1,7) << 7.0,6.0,5.0,4.0,3.0,2.0,1.0);
	Mat_<double> D6=ForD6.mul(D7.colRange(Range(0,7)));
	for (int i=0;i<realt2s.size();i++)
		if (D6.at<double>(0,0)*pow(realt2s[i],6)+
			D6.at<double>(0,1)*pow(realt2s[i],5)+
			D6.at<double>(0,2)*pow(realt2s[i],4)+
			D6.at<double>(0,3)*pow(realt2s[i],3)+
			D6.at<double>(0,4)*pow(realt2s[i],2)+
			D6.at<double>(0,5)*pow(realt2s[i],1)+
			D6.at<double>(0,6)*realt2s[i]>0)//ÅÐ¶ÏÐÞžÄ
				
			calrealt2s.push_back(realt2s[i]);
	if (calrealt2s.size()==0)
  {
   cout<<"calrealt2s"<<calrealt2s.size()<<endl;

		return false;

  }
	std::vector<Mat_<double> > FoundR;
	FoundR.resize(calrealt2s.size());
	std::vector<Mat_<double> > FoundT;
	FoundT.resize(calrealt2s.size());
	std::vector<double> minr;
	for (int i=0;i<calrealt2s.size();i++)
		minr.push_back(1e+20);

	for (int i=0;i<calrealt2s.size();i++)
	{
		double t2=calrealt2s[i];
		double d2=cg1+t2;
		Mat_<double> x=v2*d2-v1;x=x/norm(x);

		if (fabsf(a_y.dot(x))<fabsf(a_z.dot(x)))
		{
			z=x.cross(a_y);		z=z/norm(z);
			y=z.cross(x);		y=y/norm(y);
		}
		else
		{
			y=a_z.cross(x);		y=y/norm(y);
			z=x.cross(y);		z=z/norm(z);
		}
		Mat_<double> Rx(3,3,CV_64F);
		x.copyTo(Rx.col(0));
		y.copyTo(Rx.col(1));
		z.copyTo(Rx.col(2));
		Mat_<double> D=Mat::zeros(2*n,6,CV_64F);
		Mat_<double> r=Rx.t();
		for(int j=0;j<n;j++)
		{
			double ui=xx.at<double>(0,j); 
			double vi=xx.at<double>(1,j); 
			double xi=world3pts.at<double>(0,j); double yi=world3pts.at<double>(1,j); double zi=world3pts.at<double>(2,j); 
			Mat_<double> tmp=(Mat_<double>(1,6) << -r.at<double>(1,0)*yi+ui*(r.at<double>(1,2)*yi+r.at<double>(2,2)*zi)-r.at<double>(2,0)*zi,
												   -r.at<double>(2,0)*yi+ui*(r.at<double>(2,2)*yi-r.at<double>(1,2)*zi)+r.at<double>(1,0)*zi,
												   -1.0,0.0,ui,(ui*r.at<double>(0,2)-r.at<double>(0,0))*xi);
			tmp.copyTo(D.row(2*j));
			tmp=(Mat_<double>(1,6) << -r.at<double>(1,1)*yi+vi*(r.at<double>(1,2)*yi+r.at<double>(2,2)*zi)-r.at<double>(2,1)*zi,
				-r.at<double>(2,1)*yi+vi*(r.at<double>(2,2)*yi-r.at<double>(1,2)*zi)+r.at<double>(1,1)*zi,
				0.0,-1,vi,(vi*r.at<double>(0,2)-r.at<double>(0,1))*xi);
			tmp.copyTo(D.row(2*j+1));
		}
		Mat_<double> DTD=D.t()*D;
		Mat_<double> e_value, e_vector;
		eigen(DTD,e_value,e_vector);
		Mat_<double> e_vector_v1=e_vector.row(5).t();
		double tmp=e_vector_v1.at<double>(5,0);
		e_vector_v1=e_vector_v1/tmp;
		double c=e_vector_v1.at<double>(0,0);
		double s=e_vector_v1.at<double>(1,0);
		Mat_<double> t;
		e_vector_v1(Range(2,5),Range::all()).copyTo(t);

		Mat_<double> xi=world3pts.row(0);
		Mat_<double> yi=world3pts.row(1);
		Mat_<double> zi=world3pts.row(2);
		Mat_<double> XXcs(3,n,CV_64F);
		Mat(r.at<double>(0,0)*xi+(r.at<double>(1,0)*c+r.at<double>(2,0)*s)*yi+
			(-r.at<double>(1,0)*s+r.at<double>(2,0)*c)*zi+t.at<double>(0,0)).copyTo(XXcs.row(0));
		Mat(r.at<double>(0,1)*xi+(r.at<double>(1,1)*c+r.at<double>(2,1)*s)*yi+
			(-r.at<double>(1,1)*s+r.at<double>(2,1)*c)*zi+t.at<double>(1,0)).copyTo(XXcs.row(1));
		Mat(r.at<double>(0,2)*xi+(r.at<double>(1,2)*c+r.at<double>(2,2)*s)*yi+
			(-r.at<double>(1,2)*s+r.at<double>(2,2)*c)*zi+t.at<double>(2,0)).copyTo(XXcs.row(2));
		Mat_<double> XXc=Mat::zeros(3,n,CV_64F);
		for(int j=0;j<n;j++)
			XXc.col(j)=xxv.col(j)*norm(XXcs.col(j));
		calcampose(XXc,XXw,FoundR[i],FoundT[i]);
		LookMat(FoundR[i],"FoundR");
		LookMat(FoundT[i],"FoundT");

		XXc=FoundR[i]*XXw+FoundT[i]*Mat::ones(1,n,CV_64F);
		Mat_<double> xxc(2,n,CV_64F);
		divide(XXc.row(0),XXc.row(2),xxc.row(0));
		divide(XXc.row(1),XXc.row(2),xxc.row(1));

		Mat_<double> tmpminr;
		reduce((xxc-xx).mul(xxc-xx),tmpminr,0,CV_REDUCE_SUM);
		sqrt(tmpminr,tmpminr);
		Scalar mssim=mean(tmpminr);
		minr[i]=mssim[0];
		Lookvalue(minr[i],"minr");
	}
	double maxvalue=1e+18;
	int flagvalue=-1;
	for (int i=0;i<calrealt2s.size();i++)
	{
		if (minr[i]<maxvalue)
		{
			maxvalue=minr[i];
			flagvalue=i;
		}
	}
	if (flagvalue!=-1)
	{
		Rodrigues(FoundR[flagvalue],rvec);
		tvec=FoundT[flagvalue].clone();
		return true;
	}
	return false;
}
