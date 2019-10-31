//simple compile as g++ main.cpp RPnP.cpp -o main.exe `pkg-config --cflags --libs opencv`

#include "RPnP.h"

int main(int argc, char** argv)
{
    std::vector<Point3d> WorldPts;
    std::vector<Point2d> Imgpts;
    Mat_<double> ALLrotM=Mat(3,3,CV_64F);
    Mat_<double> ALLTransM=Mat(3,1,CV_64F);
    Point2d PrincipalPoint;
    //Image size is 800*600
    PrincipalPoint.x=344.760522;
    PrincipalPoint.y=256.145560;
    //FocalLength should be converted into number in pixels. For example focallenfth is 40mm, and pixe physical size is 5um, the value should be 8000
    double focallength=580;
	/************************************************************************/
	WorldPts.clear();
	WorldPts.push_back(Point3d(-1135.01f,-483.522f,0.0f));//World size is described by mm
	WorldPts.push_back(Point3d(14150.4f,-68.42f,-7350.0f));
	WorldPts.push_back(Point3d(16485.8f,3415.0f,-2148.75f));
	WorldPts.push_back(Point3d(16485.8f,3415.0f,2148.75f));


	Imgpts.clear();
	Imgpts.push_back(Point2d(251.0f,353.0f));
	Imgpts.push_back(Point2d(32.0f,272.0f));
	Imgpts.push_back(Point2d(336.0f,70.0f));
	Imgpts.push_back(Point2d(577.0f,75.0f));
	/************************************************************************/
    if(RPnP(WorldPts,Imgpts,ALLrotM,ALLTransM,PrincipalPoint,focallength))
	{
		Mat tmpr;
		Rodrigues(ALLrotM,tmpr);
		LookMat(tmpr,"Gotten R");
		LookMat(ALLTransM,"Gotten T");
	}
    else
        printf("The pose can not be calculated by RPnP");
    return 0;
}
